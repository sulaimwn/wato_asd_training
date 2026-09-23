#!/usr/bin/env python3
"""Record an exploration run for the README demo.

Runs inside the robot container (it only needs rclpy and the message types).
Every PERIOD seconds it saves a top-down PNG of the map (with the robot,
planned path, frontiers and goal drawn on it), the robot's camera image, and
one line of stats. It also tracks how close the robot's body gets to any
obstacle, using the warehouse's true geometry from make_warehouse.py.

    python3 record_run.py OUT_DIR [PERIOD_S] [TIMEOUT_S]

Stops a few seconds after the explorer reports COMPLETE. make_warehouse.py
must be importable (copy it next to this file). See tools/demo/README.md.
"""
import json
import math
import os
import struct
import sys
import time
import zlib

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_warehouse as world  # noqa: E402

SOLID = [o for o in world.OBSTACLES if o["solid"] and o["h"] > 0.9]
LIDAR_AHEAD = world.LIDAR_AHEAD_OF_AXLE
VIEW = 15.5  # m; frames show the world from -VIEW to +VIEW in x and y

COLORS = {
    "unknown": (29, 33, 41), "free": (233, 236, 239), "obstacle": (201, 42, 42),
    "halo_low": (246, 215, 167), "halo_high": (232, 89, 12),
    "frontier": (21, 170, 191), "trail": (120, 190, 130), "path": (47, 158, 68),
    "goal": (230, 73, 128), "robot": (34, 139, 230), "home": (47, 158, 68),
}


def footprint(lx, ly, yaw):
    """Outline of the robot (chassis + wheels) in the world, from the lidar pose."""
    ax, ay = lx - LIDAR_AHEAD * math.cos(yaw), ly - LIDAR_AHEAD * math.sin(yaw)
    local = [(-0.5 + 2.0 * i / 20, s) for i in range(21) for s in (0.5, -0.5)]
    local += [(x, -0.5 + i / 10) for i in range(11) for x in (-0.5, 1.5)]
    local += [(x, 0.7 * s) for x in (-0.4, 0.0, 0.4) for s in (1, -1)]
    c, s = math.cos(yaw), math.sin(yaw)
    return [(ax + c * x - s * y, ay + s * x + c * y) for x, y in local]


def body_clearance(lx, ly, yaw):
    pts = footprint(lx, ly, yaw)
    return min((min(world.distance_to(o, x, y) for x, y in pts), o["name"]) for o in SOLID)


def write_png(path, w, h, rgb):
    raw = b"".join(b"\x00" + bytes(rgb[y * w * 3:(y + 1) * w * 3]) for y in range(h))

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


class Recorder:
    def __init__(self, node, out_dir):
        self.out_dir = out_dir
        self.map = None
        self.floor_cells = None  # map indices of the building's clear floor, for coverage
        self.pose = None
        self.path = []
        self.status = "?"
        self.frontiers = []
        self.goal = None
        self.home = None
        self.camera = None
        self.trail = []
        self.closest = (float("inf"), None)
        self.frame = 0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(OccupancyGrid, "/map", self.on_map, latched)
        node.create_subscription(Odometry, "/odom/filtered", self.on_odom, 10)
        node.create_subscription(Path, "/path", self.on_path, 10)
        node.create_subscription(String, "/explore/status", self.on_status, latched)
        node.create_subscription(MarkerArray, "/explore/frontiers", self.on_markers, 10)
        node.create_subscription(Image, "/camera", self.on_camera, 1)

    def on_map(self, m):
        self.map = m
        if self.floor_cells is None:
            w, res = m.info.width, m.info.resolution
            ox, oy = m.info.origin.position.x, m.info.origin.position.y
            self.floor_cells = [
                j * w + i
                for j in range(m.info.height) for i in range(w)
                if abs(ox + (i + 0.5) * res) < 14.7 and abs(oy + (j + 0.5) * res) < 14.7
                and all(world.distance_to(o, ox + (i + 0.5) * res, oy + (j + 0.5) * res) > 0 for o in SOLID)
            ]

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)
        if not self.trail or math.hypot(self.trail[-1][0] - p.x, self.trail[-1][1] - p.y) > 0.15:
            self.trail.append((p.x, p.y))
        c, name = body_clearance(*self.pose)
        if c < self.closest[0]:
            self.closest = (c, {"x": round(p.x, 2), "y": round(p.y, 2),
                                "heading_deg": round(math.degrees(yaw)), "obstacle": name})

    def on_path(self, m):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in m.poses]

    def on_status(self, m):
        self.status = m.data

    def on_markers(self, m):
        for mk in m.markers:
            shown = mk.action == 0
            if mk.ns == "frontiers":
                self.frontiers = [(p.x, p.y) for p in mk.points] if shown else []
            elif mk.ns == "goal":
                self.goal = (mk.pose.position.x, mk.pose.position.y) if shown else None
            elif mk.ns == "home" and shown:
                self.home = (mk.pose.position.x, mk.pose.position.y)

    def on_camera(self, m):
        self.camera = m  # converted only when a frame is saved; this arrives at 30 Hz

    def coverage(self):
        d = self.map.data
        return sum(1 for i in self.floor_cells if d[i] >= 0) / max(len(self.floor_cells), 1)

    def save(self, elapsed):
        m = self.map
        w, res = m.info.width, m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        x0, y0 = int((-VIEW - ox) / res), int((-VIEW - oy) / res)
        W = H = int(2 * VIEW / res)
        img = bytearray(W * H * 3)
        lo, hi = COLORS["halo_low"], COLORS["halo_high"]

        for j in range(H):
            src = (j + y0) * w + x0
            dst = (H - 1 - j) * W * 3
            for i in range(W):
                v = m.data[src + i]
                if v < 0:
                    col = COLORS["unknown"]
                elif v == 0:
                    col = COLORS["free"]
                elif v >= 100:
                    col = COLORS["obstacle"]
                else:
                    t = v / 100.0
                    col = tuple(int(a + (b - a) * t) for a, b in zip(lo, hi))
                img[dst + 3 * i:dst + 3 * i + 3] = bytes(col)

        def dot(x, y, col, r=0):
            px, py = int((x - ox) / res) - x0, int((y - oy) / res) - y0
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    qx, qy = px + dx, py + dy
                    if dx * dx + dy * dy <= r * r and 0 <= qx < W and 0 <= qy < H:
                        k = ((H - 1 - qy) * W + qx) * 3
                        img[k:k + 3] = bytes(col)

        for x, y in self.frontiers:
            dot(x, y, COLORS["frontier"])
        for x, y in self.trail:
            dot(x, y, COLORS["trail"])
        for x, y in self.path:
            dot(x, y, COLORS["path"])
        if self.home:
            dot(*self.home, COLORS["home"], 3)
        if self.goal:
            dot(*self.goal, COLORS["goal"], 3)
        if self.pose:
            for x, y in footprint(*self.pose):
                dot(x, y, COLORS["robot"])

        name = f"{self.frame:04d}"
        write_png(os.path.join(self.out_dir, f"map_{name}.png"), W, H, img)
        cam = self.camera
        if cam is not None and cam.encoding in ("rgb8", "bgr8") and cam.step == cam.width * 3:
            data = bytearray(cam.data)
            if cam.encoding == "bgr8":
                data[0::3], data[2::3] = data[2::3], data[0::3]
            write_png(os.path.join(self.out_dir, f"cam_{name}.png"), cam.width, cam.height, data)
        with open(os.path.join(self.out_dir, "frames.jsonl"), "a") as f:
            f.write(json.dumps({
                "frame": self.frame, "t": round(elapsed, 1), "status": self.status,
                "coverage": round(self.coverage(), 4), "camera": self.camera is not None,
                "closest_m": round(self.closest[0], 3),
            }) + "\n")
        self.frame += 1


def main():
    out_dir = sys.argv[1]
    period = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    timeout = float(sys.argv[3]) if len(sys.argv) > 3 else 1200.0
    os.makedirs(out_dir, exist_ok=True)
    open(os.path.join(out_dir, "frames.jsonl"), "w").close()

    rclpy.init()
    node = rclpy.create_node("demo_recorder")
    rec = Recorder(node, out_dir)
    start = time.time()
    next_frame = start
    done_at = None
    while time.time() - start < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.time()
        if now >= next_frame and rec.map is not None and rec.pose is not None:
            next_frame += period
            rec.save(now - start)
            if rec.frame % 10 == 0:
                print(f"t={now - start:5.0f}s {rec.status:15s} coverage={rec.coverage():.0%} "
                      f"closest={rec.closest[0]:.2f}m", flush=True)
        if rec.status == "COMPLETE" and done_at is None:
            done_at = now
        if done_at is not None and now - done_at > 4:
            break

    summary = {"status": rec.status, "seconds": round(time.time() - start, 1), "frames": rec.frame,
               "coverage": round(rec.coverage(), 4), "closest_m": round(rec.closest[0], 3),
               "closest_at": rec.closest[1]}
    with open(os.path.join(out_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print("FINAL", json.dumps(summary), flush=True)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

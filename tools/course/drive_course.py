#!/usr/bin/env python3
"""Drive the robot around a fixed course of goals (or let it explore the whole
world) and measure how it went.

Runs inside the robot container while the stack is up. It only needs rclpy and
the message types. Obstacles are read straight from the world file Gazebo is
running, so every clearance number is against the real shapes, not the map.

    python3 drive_course.py WORLD.sdf course    # the world's goal course
    python3 drive_course.py WORLD.sdf explore   # explore it from an empty map

WORLD.sdf is robot_env.sdf (the original arena), warehouse.sdf or
watonomous.sdf. Start each run from a freshly started stack so the robot is at
the spawn point with an empty map. Prints a line per goal plus a summary and
writes it all to --out (results.json by default). See tools/course/README.md.
"""
import argparse
import json
import math
import os
import time
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

# The robot's shape from robot_env.sdf, in the frame of the middle of its wheel
# axle (x forward, y left). /odom/filtered reports the lidar, 1.3 m ahead of it.
LIDAR_AHEAD = 1.3
PARTS = [
    (-0.5, 1.5, -0.5, 0.5),   # chassis, 2.0 x 1.0 m
    (-0.4, 0.4, 0.5, 0.7),    # left wheel, 0.8 m across, 0.2 m wide
    (-0.4, 0.4, -0.7, -0.5),  # right wheel
]

# Goals for each world, in order, starting from the spawn point. Each one is
# picked to make the robot turn tightly around the end of something, squeeze
# through a gap, or turn around on the spot.
COURSES = {
    "default": [
        ("behind the robot, down past the small cylinder", -6.0, -12.0),
        ("through the 3.75 m gap under the south box", 6.0, -12.0),
        ("between the two east boxes", 12.0, 0.0),
        ("round the NE box to the north wall", 6.0, 12.5),
        ("across to the NW corner", -12.0, 11.0),
        ("1.3 m off the big cylinder", -3.0, 3.0),
        ("back to the start", -6.0, 0.0),
    ],
    "warehouse": [
        ("storage, upper aisle (round the rack's end)", -8.0, 12.6),
        ("pallet room (through the storage opening)", 9.0, 9.0),
        ("loading dock", 5.5, -8.0),
        ("office (through its side door)", -9.0, -7.5),
        ("back to the hall", -11.0, 0.0),
    ],
    "watonomous": [
        ("round the far end of the word", 25.0, 0.0),
        ("under the middle of the word", 0.0, -6.0),
        ("round the near end", -25.0, 0.0),
        ("over the middle of the word", 0.0, 6.0),
        ("back to the start", -24.0, 6.0),
    ],
}
WORLD_FILES = {"robot_env.sdf": "default", "warehouse.sdf": "warehouse", "watonomous.sdf": "watonomous"}

GOAL_TIMEOUT = 150.0   # s per goal
EXPLORE_TIMEOUT = 900.0
ARRIVED = 0.7          # m from the goal (lidar or axle) that counts as there


# ----------------------------------------------------------------- geometry

def load_obstacles(sdf_path):
    """Every box and cylinder the robot can bump into, from the world file."""
    world = ET.parse(sdf_path).getroot().find("world")
    obstacles = []
    for model in world.findall("model"):
        if model.get("name") in ("robot", "ground_plane"):
            continue
        mx, my, _, _, _, myaw = [float(v) for v in (model.findtext("pose") or "0 0 0 0 0 0").split()]
        for link in model.findall("link"):
            lx, ly, _, _, _, lyaw = [float(v) for v in (link.findtext("pose") or "0 0 0 0 0 0").split()]
            c, s = math.cos(myaw), math.sin(myaw)
            x, y, yaw = mx + c * lx - s * ly, my + s * lx + c * ly, myaw + lyaw
            for col in link.findall("collision"):
                box = col.find("geometry/box/size")
                cyl = col.find("geometry/cylinder/radius")
                if box is not None:
                    sx, sy, _ = [float(v) for v in box.text.split()]
                    obstacles.append(dict(name=model.get("name"), shape="box", x=x, y=y, yaw=yaw, sx=sx, sy=sy))
                elif cyl is not None:
                    obstacles.append(dict(name=model.get("name"), shape="cylinder", x=x, y=y, r=float(cyl.text)))
    return obstacles


def distance_to(ob, px, py):
    """Distance from a point to an obstacle's footprint (0 if inside it)."""
    if ob["shape"] == "cylinder":
        return max(0.0, math.hypot(px - ob["x"], py - ob["y"]) - ob["r"])
    c, s = math.cos(ob["yaw"]), math.sin(ob["yaw"])
    dx, dy = px - ob["x"], py - ob["y"]
    ex = max(abs(c * dx + s * dy) - ob["sx"] / 2, 0.0)
    ey = max(abs(-s * dx + c * dy) - ob["sy"] / 2, 0.0)
    return math.hypot(ex, ey)


def outline(step=0.05):
    """Points every `step` m around each part of the robot, in the axle frame."""
    pts = []
    for x0, x1, y0, y1 in PARTS:
        nx, ny = max(1, round((x1 - x0) / step)), max(1, round((y1 - y0) / step))
        pts += [(x0 + (x1 - x0) * i / nx, y) for i in range(nx + 1) for y in (y0, y1)]
        pts += [(x, y0 + (y1 - y0) * j / ny) for j in range(1, ny) for x in (x0, x1)]
    return pts


OUTLINE = outline()


def body_clearance(obstacles, lx, ly, yaw):
    """Smallest gap (m) between the robot's body and any obstacle, and which one."""
    ax, ay = lx - LIDAR_AHEAD * math.cos(yaw), ly - LIDAR_AHEAD * math.sin(yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(ax + c * x - s * y, ay + s * x + c * y) for x, y in OUTLINE]
    best = (float("inf"), None)
    for ob in obstacles:
        # Skip anything that can't be close (a rough bound on its size plus the robot's)
        reach = (ob["r"] if ob["shape"] == "cylinder" else math.hypot(ob["sx"], ob["sy"]) / 2) + 2.2
        if math.hypot(ob["x"] - ax, ob["y"] - ay) - reach > best[0]:
            continue
        d = min(distance_to(ob, x, y) for x, y in pts)
        if d < best[0]:
            best = (d, ob["name"])
    return best


def phantom_cells(m, obstacles, tolerance=0.2):
    """Obstacle cells on the map (cost 100) more than `tolerance` m from any
    real surface: things the robot believes are there but aren't."""
    w, res = m.info.width, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    total, phantom = 0, 0
    for i, v in enumerate(m.data):
        if v >= 100:
            total += 1
            x, y = ox + (i % w + 0.5) * res, oy + (i // w + 0.5) * res
            if all(distance_to(o, x, y) > tolerance for o in obstacles):
                phantom += 1
    return total, phantom


def floor_cells(m, obstacles):
    """Map cells of the world's open floor (inside the outer walls, not under anything)."""
    walls = [o for o in obstacles if o["shape"] == "box" and max(o["sx"], o["sy"]) > 20]
    hx = min(abs(o["x"]) - o["sx"] / 2 for o in walls if o["sx"] < o["sy"])
    hy = min(abs(o["y"]) - o["sy"] / 2 for o in walls if o["sx"] > o["sy"])
    w, h, res = m.info.width, m.info.height, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    under = set()  # cells under an obstacle
    for o in obstacles:
        r = o["r"] if o["shape"] == "cylinder" else math.hypot(o["sx"], o["sy"]) / 2
        for j in range(max(int((o["y"] - r - oy) / res), 0), min(int((o["y"] + r - oy) / res) + 1, h)):
            for i in range(max(int((o["x"] - r - ox) / res), 0), min(int((o["x"] + r - ox) / res) + 1, w)):
                if distance_to(o, ox + (i + 0.5) * res, oy + (j + 0.5) * res) == 0:
                    under.add(j * w + i)
    cells = []
    for j in range(h):
        if abs(oy + (j + 0.5) * res) < hy - 0.3:
            cells += [j * w + i for i in range(w)
                      if abs(ox + (i + 0.5) * res) < hx - 0.3 and j * w + i not in under]
    return cells


# ----------------------------------------------------------------- driving

class Course:
    def __init__(self, node, obstacles):
        self.node = node
        self.obstacles = obstacles
        self.pose = None           # lidar (x, y, yaw) from /odom/filtered
        self.path_len = None       # poses in the latest /path
        self.map = None
        self.status = None
        self.closest = (float("inf"), None, None)  # over the whole run
        self.leg_closest = (float("inf"), None, None)
        self.distance = 0.0        # m driven by the axle
        self.max_tilt = 0.0        # deg, worst pitch or roll (tilting the lidar maps the floor as obstacles)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(Odometry, "/odom/filtered", self.on_odom, 10)
        node.create_subscription(Path, "/path", self.on_path, 10)
        node.create_subscription(OccupancyGrid, "/map", self.on_map, latched)
        node.create_subscription(String, "/explore/status", self.on_status, latched)
        self.goal_pub = node.create_publisher(PointStamped, "/goal_point", 10)
        self.explore_pub = node.create_publisher(Bool, "/explore/enable", 10)

    def axle(self):
        x, y, yaw = self.pose
        return x - LIDAR_AHEAD * math.cos(yaw), y - LIDAR_AHEAD * math.sin(yaw)

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.pose is not None:
            before = self.axle()
            self.pose = (p.x, p.y, yaw)
            after = self.axle()
            self.distance += math.hypot(after[0] - before[0], after[1] - before[1])
        self.pose = (p.x, p.y, yaw)
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        self.max_tilt = max(self.max_tilt, math.degrees(max(abs(pitch), abs(roll))))
        gap, name = body_clearance(self.obstacles, p.x, p.y, yaw)
        where = (round(self.axle()[0], 2), round(self.axle()[1], 2))
        if gap < self.closest[0]:
            self.closest = (gap, name, where)
        if gap < self.leg_closest[0]:
            self.leg_closest = (gap, name, where)

    def on_path(self, m):
        self.path_len = len(m.poses)

    def on_map(self, m):
        self.map = m

    def on_status(self, m):
        self.status = m.data

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_for_stack(self):
        while self.pose is None or self.map is None:
            self.spin(0.2)
        self.spin(3.0)  # let the first scans reach the map

    def drive_to(self, x, y):
        """Send one goal and wait until the planner stops at it (or gives up)."""
        msg = PointStamped()
        msg.header.frame_id = "sim_world"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.point.x, msg.point.y = x, y
        self.leg_closest = (float("inf"), None, None)
        start_dist = self.distance
        self.path_len = None
        self.goal_pub.publish(msg)
        start = last_route = time.time()
        near = float("inf")
        while time.time() - start < GOAL_TIMEOUT:
            self.spin(0.2)
            lx, ly, _ = self.pose
            ax, ay = self.axle()
            near = min(math.hypot(lx - x, ly - y), math.hypot(ax - x, ay - y))
            if self.path_len:
                last_route = time.time()
            # The planner sends an empty path once the goal is reached
            if self.path_len == 0 and near < ARRIVED and time.time() - start > 1.0:
                self.spin(1.0)  # let it come to a stop
                return dict(reached=True, seconds=round(time.time() - start, 1),
                            driven_m=round(self.distance - start_dist, 1), near_m=round(near, 2))
            if time.time() - last_route > 15.0:
                break  # no route for 15 s: the planner gave up on it
        return dict(reached=False, seconds=round(time.time() - start, 1),
                    driven_m=round(self.distance - start_dist, 1), near_m=round(near, 2))

    def coverage(self, cells):
        return sum(1 for i in cells if self.map.data[i] >= 0) / max(len(cells), 1)


def gap_text(closest):
    gap, name, where = closest
    return f"{gap:.2f} m ({name} at {where})" if name else "n/a"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sdf")
    ap.add_argument("mode", choices=["course", "explore"])
    ap.add_argument("--out", default="results.json")
    args = ap.parse_args()

    world = WORLD_FILES.get(os.path.basename(args.sdf), os.path.basename(args.sdf))
    obstacles = load_obstacles(args.sdf)
    rclpy.init()
    node = rclpy.create_node("drive_course")
    run = Course(node, obstacles)
    run.wait_for_stack()
    results = dict(world=world, mode=args.mode, legs=[])
    start = time.time()

    if args.mode == "course":
        for label, x, y in COURSES[world]:
            leg = run.drive_to(x, y)
            leg.update(goal=label, x=x, y=y, closest_m=round(run.leg_closest[0], 3),
                       closest_to=run.leg_closest[1])
            results["legs"].append(leg)
            print(f"{'ok  ' if leg['reached'] else 'FAIL'} {label:48s} {leg['seconds']:6.1f} s  "
                  f"closest {gap_text(run.leg_closest)}", flush=True)
        results["reached"] = sum(leg["reached"] for leg in results["legs"])
        results["goals"] = len(results["legs"])
    else:
        cells = floor_cells(run.map, obstacles)
        run.explore_pub.publish(Bool(data=True))
        last_print = time.time()
        while time.time() - start < EXPLORE_TIMEOUT and run.status != "COMPLETE":
            run.spin(0.5)
            if time.time() - last_print > 30:
                last_print = time.time()
                print(f"t={time.time() - start:4.0f}s {run.status} mapped {run.coverage(cells):.1%} "
                      f"closest {run.closest[0]:.2f} m", flush=True)
        run.spin(2.0)
        results.update(status=run.status, coverage=round(run.coverage(cells), 4))
        run.explore_pub.publish(Bool(data=False))

    total, phantom = phantom_cells(run.map, obstacles)
    results.update(seconds=round(time.time() - start, 1), driven_m=round(run.distance, 1),
                   max_tilt_deg=round(run.max_tilt, 1), map_obstacle_cells=total, map_phantom_cells=phantom,
                   closest_m=round(run.closest[0], 3), closest_to=run.closest[1], closest_at=run.closest[2])
    with open(args.out, "w") as f:
        json.dump(results, f, indent=2)
    print("SUMMARY", json.dumps({k: v for k, v in results.items() if k != "legs"}), flush=True)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

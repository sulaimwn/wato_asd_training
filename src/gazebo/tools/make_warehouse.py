#!/usr/bin/env python3
"""Generate the warehouse world.

Every obstacle is listed once in OBSTACLES below. Running this script writes
two files that must always agree with each other:

  launch/warehouse.sdf        what Gazebo simulates (physics, lidar, camera)
  launch/warehouse_env.urdf   the same shapes, for Foxglove's 3D panel to draw

    python3 src/gazebo/tools/make_warehouse.py

The robot, sensors, lighting and GUI setup are copied from launch/robot_env.sdf
(the assignment's original world), so only the obstacles and the spawn point
differ. Before writing anything it checks that the robot can reach every room.

Layout (30 m x 30 m, x to the right, y up, robot starts in the hall facing +x):

    +-----------------------+-------------------+ y = 15
    |  STORAGE (racks)      .   PALLET ROOM     |
    |  ======rack=====      .                   |
    +------   ---------------------   ----------+ y = 5
    |                  MAIN HALL                |
    |  R->                                      |
    +-----   ----------------------   ----------+ y = -5
    |  OFFICE         .   LOADING DOCK          |
    |                 .           [ truck ]     |
    +-----------------+-------------------------+ y = -15
  x = -15                                     x = 15
"""

import math
import re
from collections import deque
from pathlib import Path

HERE = Path(__file__).resolve().parent
LAUNCH_DIR = HERE.parent / "launch"

# Where the robot appears: (x, y, yaw) of the middle of its wheel axle
SPAWN = (-11.0, 0.0, 0.0)
LIDAR_AHEAD_OF_AXLE = 1.3  # m, from robot_env.sdf (chassis at +0.5, lidar at +0.8)

# Colors (r, g, b)
CONCRETE = (0.72, 0.70, 0.66)
DRYWALL = (0.56, 0.62, 0.70)
RACK = (0.95, 0.45, 0.08)
CARTON = (0.70, 0.52, 0.30)
PALLET = (0.58, 0.40, 0.22)
BOLLARD = (0.98, 0.80, 0.10)
DESK = (0.30, 0.32, 0.36)
CRATE = (0.22, 0.50, 0.32)
TRUCK = (0.82, 0.16, 0.14)
STRIPE = (0.98, 0.85, 0.10)
FLOOR = (0.50, 0.50, 0.52)


def box(name, x, y, sx, sy, height, color, solid=True, z=0.0):
    """A box sitting on the floor (or at height z), centered on (x, y)."""
    return dict(name=name, shape="box", x=x, y=y, sx=sx, sy=sy, h=height, z=z, color=color, solid=solid)


def cylinder(name, x, y, radius, height, color):
    return dict(name=name, shape="cylinder", x=x, y=y, r=radius, h=height, z=0.0, color=color, solid=True)


# Everything the lidar can hit is at least 1.3 m tall: the lidar scans a flat
# plane 0.9 m above the floor, so anything lower is invisible to it (and the
# robot would drive straight into it). Doorways are 5 m wide so the 2 m robot
# can turn in them.
OBSTACLES = [
    # Outer walls
    box("wall_north", 0, 15, 30.5, 0.5, 3.0, CONCRETE),
    box("wall_south", 0, -15, 30.5, 0.5, 3.0, CONCRETE),
    box("wall_east", 15, 0, 0.5, 30.5, 3.0, CONCRETE),
    box("wall_west", -15, 0, 0.5, 30.5, 3.0, CONCRETE),

    # Hall's north wall (y = 5), doorways at x = -10..-5 and 6..11
    box("wall_hall_n1", -12.5, 5, 5.0, 0.3, 3.0, DRYWALL),
    box("wall_hall_n2", 0.5, 5, 11.0, 0.3, 3.0, DRYWALL),
    box("wall_hall_n3", 13.0, 5, 4.0, 0.3, 3.0, DRYWALL),
    # Hall's south wall (y = -5), doorways at x = -11..-6 and 4..9
    box("wall_hall_s1", -13.0, -5, 4.0, 0.3, 3.0, DRYWALL),
    box("wall_hall_s2", -1.0, -5, 10.0, 0.3, 3.0, DRYWALL),
    box("wall_hall_s3", 12.0, -5, 6.0, 0.3, 3.0, DRYWALL),
    # Between storage and the pallet room, open above y = 10.2
    box("wall_storage", 2.0, 7.6, 0.3, 5.2, 3.0, DRYWALL),
    # Between office and loading dock, doorway at y = -12..-7
    box("wall_office_1", -2.0, -13.5, 0.3, 3.0, 3.0, DRYWALL),
    box("wall_office_2", -2.0, -6.0, 0.3, 2.0, 3.0, DRYWALL),

    # Storage: one long rack against the west wall, so the upper aisle is only
    # reachable by going around its east end (or through the pallet room)
    box("rack", -8.75, 10.0, 12.5, 1.0, 2.5, RACK),
    *[box(f"carton_{i}", x, 10.0, 1.6, 0.8, 0.5, CARTON, solid=False, z=2.5)
      for i, x in enumerate([-13.0, -10.5, -8.0, -5.5, -3.5])],

    # Pallet room
    box("pallets_1", 6.5, 10.0, 1.4, 1.4, 1.8, PALLET),
    box("pallets_2", 11.5, 11.5, 1.4, 1.4, 1.8, PALLET),

    # Main hall: safety bollards the robot has to weave past
    cylinder("pillar_1", -2.0, 1.8, 0.35, 3.0, BOLLARD),
    cylinder("pillar_2", 8.0, -1.8, 0.35, 3.0, BOLLARD),
    # Painted lane lines (visual only, too low for the lidar to see)
    box("lane_north", 0, 3.9, 28.0, 0.12, 0.01, STRIPE, solid=False),
    box("lane_south", 0, -3.9, 28.0, 0.12, 0.01, STRIPE, solid=False),

    # Office: desks (1.3 m tall; a real 0.75 m desk would be under the lidar)
    box("desk_1", -11.5, -10.0, 2.4, 1.2, 1.3, DESK),
    box("desk_2", -6.0, -12.0, 2.4, 1.2, 1.3, DESK),

    # Loading dock
    box("truck", 9.5, -12.2, 7.0, 2.6, 3.2, TRUCK),
    box("crate_1", 3.0, -9.0, 1.5, 1.5, 1.5, CRATE),
    box("crate_2", 0.5, -12.5, 1.5, 1.5, 1.5, CRATE),
]

# A point in each area, all of which the robot must be able to reach
PROBES = {
    "hall (east end)": (12.0, 0.0),
    "storage, lower aisle": (-8.0, 7.3),
    "storage, upper aisle": (-8.0, 12.6),
    "pallet room": (9.0, 8.0),
    "office": (-9.0, -7.5),
    "loading dock": (5.5, -8.0),
}


# ----------------------------------------------------------------- checking

def distance_to(ob, px, py):
    """Distance from (px, py) to the obstacle's footprint (0 if inside)."""
    if ob["shape"] == "cylinder":
        return max(0.0, math.hypot(px - ob["x"], py - ob["y"]) - ob["r"])
    dx = max(abs(px - ob["x"]) - ob["sx"] / 2, 0.0)
    dy = max(abs(py - ob["y"]) - ob["sy"] / 2, 0.0)
    return math.hypot(dx, dy)


def check_reachable(clearance=0.8, res=0.1):
    """Flood fill from the spawn over cells at least `clearance` from any
    obstacle (what the planner allows), and check every probe is reached."""
    solid = [o for o in OBSTACLES if o["solid"] and o["h"] > 0.9]
    n = int(30 / res)
    free = [[False] * n for _ in range(n)]
    for j in range(n):
        for i in range(n):
            px, py = -15 + (i + 0.5) * res, -15 + (j + 0.5) * res
            free[j][i] = all(distance_to(o, px, py) >= clearance for o in solid)

    def cell(x, y):
        return int((x + 15) / res), int((y + 15) / res)

    sx = SPAWN[0] + LIDAR_AHEAD_OF_AXLE * math.cos(SPAWN[2])
    sy = SPAWN[1] + LIDAR_AHEAD_OF_AXLE * math.sin(SPAWN[2])
    start = cell(sx, sy)
    if not free[start[1]][start[0]]:
        raise SystemExit(f"Spawn point ({sx:.1f}, {sy:.1f}) is too close to an obstacle")

    seen = {start}
    queue = deque([start])
    while queue:
        i, j = queue.popleft()
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ni, nj = i + di, j + dj
            if 0 <= ni < n and 0 <= nj < n and free[nj][ni] and (ni, nj) not in seen:
                seen.add((ni, nj))
                queue.append((ni, nj))

    ok = True
    for name, (px, py) in PROBES.items():
        reached = cell(px, py) in seen
        ok &= reached
        print(f"  {'ok  ' if reached else 'FAIL'} {name} ({px}, {py})")
    total_free = sum(row.count(True) for row in free)
    print(f"  robot can reach {len(seen) * res * res:.0f} m^2 of {total_free * res * res:.0f} m^2 of clear floor")
    if not ok:
        raise SystemExit("Some areas are unreachable; widen a doorway or move an obstacle")


# ----------------------------------------------------------------- writing

def rgb(color, alpha=1.0):
    return " ".join(f"{c:g}" for c in color) + f" {alpha:g}"


def sdf_geometry(ob):
    if ob["shape"] == "cylinder":
        return f"<cylinder><radius>{ob['r']:g}</radius><length>{ob['h']:g}</length></cylinder>"
    return f"<box><size>{ob['sx']:g} {ob['sy']:g} {ob['h']:g}</size></box>"


def sdf_model(ob):
    material = (f"<material><ambient>{rgb(ob['color'])}</ambient><diffuse>{rgb(ob['color'])}</diffuse>"
                f"<specular>0.1 0.1 0.1 1</specular></material>")
    geometry = sdf_geometry(ob)
    collision = (f"\n                <collision name='collision'><geometry>{geometry}</geometry></collision>"
                 if ob["solid"] else "")
    z = ob["z"] + ob["h"] / 2
    return f"""
        <model name='{ob['name']}'>
            <static>true</static>
            <pose>{ob['x']:g} {ob['y']:g} {z:g} 0 0 0</pose>
            <link name='link'>
                <visual name='visual'><geometry>{geometry}</geometry>{material}</visual>{collision}
            </link>
        </model>
"""


def ground_model():
    return f"""
        <model name="environment">
            <static>true</static>
            <link name="ground">
                <collision name="collision"><geometry><plane><normal>0 0 1</normal></plane></geometry></collision>
                <visual name="visual">
                    <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
                    <material><ambient>{rgb(FLOOR)}</ambient><diffuse>{rgb(FLOOR)}</diffuse><specular>0.1 0.1 0.1 1</specular></material>
                </visual>
            </link>
        </model>
"""


def write_sdf():
    template = (LAUNCH_DIR / "robot_env.sdf").read_text()
    head = template[:template.index('<model name="environment">')]
    tail = template[template.index("<model name='robot'"):]

    x, y, yaw = SPAWN
    tail, count = re.subn(r"(<model name='robot'[^>]*>\s*<pose relative_to='world'>)[^<]*(</pose>)",
                          rf"\g<1>{x:g} {y:g} 1 0 0 {yaw:g}\g<2>", tail, count=1)
    if count != 1:
        raise SystemExit("Couldn't find the robot's spawn pose in robot_env.sdf")

    notice = ("<!-- Generated by src/gazebo/tools/make_warehouse.py from robot_env.sdf.\n"
              "     Edit that script, not this file. -->\n")
    head = head.replace("<sdf version", notice + "<sdf version", 1)
    body = ground_model() + "".join(sdf_model(o) for o in OBSTACLES) + "\n        "
    (LAUNCH_DIR / "warehouse.sdf").write_text(head + body.lstrip("\n") + tail)


def write_urdf():
    lines = [
        '<?xml version="1.0" ?>',
        "<!-- Generated by src/gazebo/tools/make_warehouse.py. Edit that script, not this file. -->",
        '<robot name="warehouse">',
        '  <link name="sim_world"/>',
    ]
    for ob in OBSTACLES:
        if ob["shape"] == "cylinder":
            geometry = f'<cylinder radius="{ob["r"]:g}" length="{ob["h"]:g}"/>'
        else:
            geometry = f'<box size="{ob["sx"]:g} {ob["sy"]:g} {ob["h"]:g}"/>'
        z = ob["z"] + ob["h"] / 2
        lines += [
            f'  <joint name="{ob["name"]}_joint" type="fixed">',
            f'    <parent link="sim_world"/><child link="{ob["name"]}"/>',
            f'    <origin xyz="{ob["x"]:g} {ob["y"]:g} {z:g}"/>',
            "  </joint>",
            f'  <link name="{ob["name"]}">',
            f"    <visual><geometry>{geometry}</geometry>"
            f'<material name="{ob["name"]}_color"><color rgba="{rgb(ob["color"])}"/></material></visual>',
            "  </link>",
        ]
    lines.append("</robot>")
    (LAUNCH_DIR / "warehouse_env.urdf").write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    print("Checking the robot can reach every area:")
    check_reachable()
    write_sdf()
    write_urdf()
    print(f"Wrote {LAUNCH_DIR / 'warehouse.sdf'}")
    print(f"Wrote {LAUNCH_DIR / 'warehouse_env.urdf'}")

#!/usr/bin/env python3
"""Generate the custom Gazebo worlds.

Every obstacle of every world is listed once in this file. For each world,
running it writes two files that must always agree with each other:

  launch/<world>.sdf        what Gazebo simulates (physics, lidar, camera)
  launch/<world>_env.urdf   the same shapes, for Foxglove's 3D panel to draw

    python3 src/gazebo/tools/make_worlds.py              # all worlds
    python3 src/gazebo/tools/make_worlds.py watonomous   # just one

The robot, sensors, lighting and GUI setup are copied from launch/robot_env.sdf
(the assignment's original world), so only the obstacles and the spawn point
differ. Before writing a world it checks the robot can reach every area of it.

Worlds:

  warehouse    30 m x 30 m, four rooms off a main hall. The robot can't see
               most of it from where it starts, so it's good for exploration.

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

  watonomous   55 m x 20 m hall with walls down the middle that spell
               WATONOMOUS from above. From the robot's camera they're just
               walls; the word only shows up in the map as the lidar finds them.
"""

import math
import re
import sys
from collections import deque
from pathlib import Path

HERE = Path(__file__).resolve().parent
LAUNCH_DIR = HERE.parent / "launch"

LIDAR_AHEAD_OF_AXLE = 1.3  # m, from robot_env.sdf (chassis at +0.5, lidar at +0.8)
LIDAR_HEIGHT = 0.9         # m above the floor; anything lower is invisible to it
CLEARANCE = 0.8            # m the planner keeps the lidar from obstacles
BODY_CLEARANCE = 1.2       # m the robot's body needs: half its 1.4 m width plus room to turn

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


# ----------------------------------------------------------------- shapes

def box(name, x, y, sx, sy, height, color, solid=True, z=0.0, yaw=0.0):
    """A box sitting on the floor (or at height z), centered on (x, y), turned by yaw."""
    return dict(name=name, shape="box", x=x, y=y, sx=sx, sy=sy, h=height, z=z, yaw=yaw,
                color=color, solid=solid)


def cylinder(name, x, y, radius, height, color):
    return dict(name=name, shape="cylinder", x=x, y=y, r=radius, h=height, z=0.0, yaw=0.0,
                color=color, solid=True)


def wall(name, a, b, thickness, height, color):
    """A straight wall from point a to point b. It overshoots each end by half
    its thickness, so walls that meet at a corner overlap instead of leaving a notch."""
    (x1, y1), (x2, y2) = a, b
    length = math.hypot(x2 - x1, y2 - y1)
    return box(name, (x1 + x2) / 2, (y1 + y2) / 2, length + thickness, thickness, height, color,
               yaw=math.atan2(y2 - y1, x2 - x1))


def distance_to(ob, px, py):
    """Distance from (px, py) to the obstacle's footprint (0 if inside)."""
    if ob["shape"] == "cylinder":
        return max(0.0, math.hypot(px - ob["x"], py - ob["y"]) - ob["r"])
    # Rotate the point into the box's own frame, then it's an axis-aligned box
    c, s = math.cos(ob["yaw"]), math.sin(ob["yaw"])
    dx, dy = px - ob["x"], py - ob["y"]
    lx, ly = c * dx + s * dy, -s * dx + c * dy
    ex = max(abs(lx) - ob["sx"] / 2, 0.0)
    ey = max(abs(ly) - ob["sy"] / 2, 0.0)
    return math.hypot(ex, ey)


# ----------------------------------------------------------------- warehouse

WAREHOUSE = dict(
    name="warehouse",
    spawn=(-11.0, 0.0, 0.0),  # (x, y, yaw) of the middle of the robot's wheel axle
    half_size=(15.0, 15.0),   # the building spans -15..15 in x and y
    floor=FLOOR,
    # Everything the lidar can hit is at least 1.3 m tall, and doorways are
    # 5 m wide so the 2 m robot can turn in them.
    obstacles=[
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
    ],
    # A point in each area, all of which the robot must be able to reach
    probes={
        "hall (east end)": (12.0, 0.0),
        "storage, lower aisle": (-8.0, 7.3),
        "storage, upper aisle": (-8.0, 12.6),
        "pallet room": (9.0, 8.0),
        "office": (-9.0, -7.5),
        "loading dock": (5.5, -8.0),
    },
)


# ----------------------------------------------------------------- watonomous

# Block capitals, drawn as lines through points on a grid 6 units tall.
# Each letter is (width, [polyline, ...]); a polyline is a list of points
# joined by straight walls. Corners are cut at 45 degrees to round A, O, U, S.
# Every letter's outer sides are vertical: a slanted side (like a normal A's
# legs) leaves a wedge-shaped gap next to the neighbouring letter that is wide
# enough for the planner but not for the robot's body. Found the hard way:
# the robot took that shortcut between the A and the T and hit the T.
FONT = {
    "W": (5, [[(0, 6), (0, 0), (2.5, 3.2), (5, 0), (5, 6)]]),
    "A": (4, [[(0, 0), (0, 5.2), (0.8, 6), (3.2, 6), (4, 5.2), (4, 0)], [(0, 2.6), (4, 2.6)]]),
    "T": (4, [[(0, 6), (4, 6)], [(2, 6), (2, 0)]]),
    "O": (4, [[(1, 0), (3, 0), (4, 1), (4, 5), (3, 6), (1, 6), (0, 5), (0, 1), (1, 0)]]),
    "N": (4, [[(0, 0), (0, 6), (4, 0), (4, 6)]]),
    "M": (5, [[(0, 0), (0, 6), (2.5, 2.5), (5, 6), (5, 0)]]),
    "U": (4, [[(0, 6), (0, 1), (1, 0), (3, 0), (4, 1), (4, 6)]]),
    "S": (4, [[(4, 6), (0.8, 6), (0, 5.2), (0, 3.8), (0.8, 3), (3.2, 3), (4, 2.2),
               (4, 0.8), (3.2, 0), (0, 0)]]),
}


def lettering(text, height, gap, thickness, wall_height, color):
    """Walls that spell `text`, centered on (0, 0), letters `height` m tall."""
    scale = height / 6.0
    total = sum(FONT[ch][0] * scale for ch in text) + gap * (len(text) - 1)
    walls = []
    x0 = -total / 2
    for i, ch in enumerate(text):
        width, lines = FONT[ch]
        for j, line in enumerate(lines):
            for k in range(len(line) - 1):
                (ax, ay), (bx, by) = line[k], line[k + 1]
                a = (x0 + ax * scale, (ay - 3) * scale)
                b = (x0 + bx * scale, (by - 3) * scale)
                walls.append(wall(f"letter{i}_{ch}_{j}_{k}", a, b, thickness, wall_height, color))
        x0 += width * scale + gap
    return walls


WATONOMOUS = dict(
    name="watonomous",
    spawn=(-24.0, 6.0, 0.0),  # top-left corner, facing along the word
    half_size=(27.5, 10.0),
    floor=FLOOR,
    # Letters 5 m tall with 1 m between them: too narrow for the 1.4 m wide
    # robot, so it drives around the word rather than between the letters.
    obstacles=[
        box("wall_north", 0, 10, 55.5, 0.5, 3.0, CONCRETE),
        box("wall_south", 0, -10, 55.5, 0.5, 3.0, CONCRETE),
        box("wall_east", 27.5, 0, 0.5, 20.5, 3.0, CONCRETE),
        box("wall_west", -27.5, 0, 0.5, 20.5, 3.0, CONCRETE),
        *lettering("WATONOMOUS", height=5.0, gap=1.0, thickness=0.35, wall_height=2.0, color=CONCRETE),
    ],
    probes={
        "above the word": (0.0, 6.0),
        "below the word": (0.0, -6.0),
        "left end": (-25.0, 0.0),
        "right end": (25.0, 0.0),
    },
)

WORLDS = {w["name"]: w for w in (WAREHOUSE, WATONOMOUS)}


# ----------------------------------------------------------------- checking

def solid_obstacles(world):
    """What the lidar can hit and the robot can bump into."""
    return [o for o in world["obstacles"] if o["solid"] and o["h"] > LIDAR_HEIGHT]


def check_reachable(world, res=0.1):
    """Two checks, on a grid of the world at `res` m per cell:

    1. Every probe is reachable from the spawn through cells at least
       CLEARANCE from any obstacle (where the planner may put the robot).
    2. The planner has no shortcuts the robot's body can't take: between any
       two probes, the planner's shortest route is no more than 3 m shorter
       than the shortest route through cells BODY_CLEARANCE from everything.
       Otherwise the planner squeezes the robot through a gap it doesn't fit.
    """
    solid = solid_obstacles(world)
    hx, hy = world["half_size"]
    nx, ny = int(2 * hx / res), int(2 * hy / res)
    clear = [[min(distance_to(o, -hx + (i + 0.5) * res, -hy + (j + 0.5) * res) for o in solid)
              for i in range(nx)] for j in range(ny)]

    def cell(x, y):
        return int((x + hx) / res), int((y + hy) / res)

    def flood(start, min_clear):
        """Steps from start to every cell at least min_clear from obstacles."""
        dist = {start: 0}
        queue = deque([start])
        while queue:
            i, j = queue.popleft()
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                a, b = i + di, j + dj
                if 0 <= a < nx and 0 <= b < ny and (a, b) not in dist and clear[b][a] >= min_clear:
                    dist[(a, b)] = dist[(i, j)] + 1
                    queue.append((a, b))
        return dist

    x, y, yaw = world["spawn"]
    lidar = (x + LIDAR_AHEAD_OF_AXLE * math.cos(yaw), y + LIDAR_AHEAD_OF_AXLE * math.sin(yaw))
    start = cell(*lidar)
    if clear[start[1]][start[0]] < CLEARANCE:
        raise SystemExit(f"{world['name']}: spawn ({lidar[0]:.1f}, {lidar[1]:.1f}) is too close to an obstacle")

    reach = flood(start, CLEARANCE)
    ok = True
    for name, (px, py) in world["probes"].items():
        reached = cell(px, py) in reach
        ok &= reached
        print(f"  {'ok  ' if reached else 'FAIL'} reach {name} ({px}, {py})")
    total = sum(1 for row in clear for c in row if c >= CLEARANCE)
    print(f"  robot can reach {len(reach) * res * res:.0f} m^2 of {total * res * res:.0f} m^2 of clear floor")
    if not ok:
        raise SystemExit(f"{world['name']}: some areas are unreachable; widen a gap or move an obstacle")

    probes = list(world["probes"].items())
    for k, (name_a, pa) in enumerate(probes):
        planner = flood(cell(*pa), CLEARANCE)
        body = flood(cell(*pa), BODY_CLEARANCE)
        for name_b, pb in probes[k + 1:]:
            c = cell(*pb)
            p_len = planner[c] * res
            b_len = body[c] * res if c in body else float("inf")
            if p_len < b_len - 3.0:
                raise SystemExit(f"{world['name']}: from {name_a} to {name_b} the planner can squeeze through a "
                                 f"gap too narrow for the robot's body ({p_len:.0f} m instead of {b_len:.0f} m)")
    print(f"  ok   no shortcuts too narrow for the robot's body ({len(probes)} probes, pairwise)")


# ----------------------------------------------------------------- writing

def rgb(color, alpha=1.0):
    return " ".join(f"{c:g}" for c in color) + f" {alpha:g}"


def sdf_geometry(ob):
    if ob["shape"] == "cylinder":
        return f"<cylinder><radius>{ob['r']:g}</radius><length>{ob['h']:g}</length></cylinder>"
    return f"<box><size>{ob['sx']:.5g} {ob['sy']:.5g} {ob['h']:g}</size></box>"


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
            <pose>{ob['x']:.5g} {ob['y']:.5g} {z:g} 0 0 {ob['yaw']:.5g}</pose>
            <link name='link'>
                <visual name='visual'><geometry>{geometry}</geometry>{material}</visual>{collision}
            </link>
        </model>
"""


def ground_model(color):
    return f"""
        <model name="environment">
            <static>true</static>
            <link name="ground">
                <collision name="collision"><geometry><plane><normal>0 0 1</normal></plane></geometry></collision>
                <visual name="visual">
                    <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
                    <material><ambient>{rgb(color)}</ambient><diffuse>{rgb(color)}</diffuse><specular>0.1 0.1 0.1 1</specular></material>
                </visual>
            </link>
        </model>
"""


def write_sdf(world):
    template = (LAUNCH_DIR / "robot_env.sdf").read_text()
    head = template[:template.index('<model name="environment">')]
    tail = template[template.index("<model name='robot'"):]

    x, y, yaw = world["spawn"]
    tail, count = re.subn(r"(<model name='robot'[^>]*>\s*<pose relative_to='world'>)[^<]*(</pose>)",
                          rf"\g<1>{x:g} {y:g} 1 0 0 {yaw:g}\g<2>", tail, count=1)
    if count != 1:
        raise SystemExit("Couldn't find the robot's spawn pose in robot_env.sdf")

    notice = ("<!-- Generated by src/gazebo/tools/make_worlds.py from robot_env.sdf.\n"
              "     Edit that script, not this file. -->\n")
    head = head.replace("<sdf version", notice + "<sdf version", 1)
    body = ground_model(world["floor"]) + "".join(sdf_model(o) for o in world["obstacles"]) + "\n        "
    (LAUNCH_DIR / f"{world['name']}.sdf").write_text(head + body.lstrip("\n") + tail)


def write_urdf(world):
    lines = [
        '<?xml version="1.0" ?>',
        "<!-- Generated by src/gazebo/tools/make_worlds.py. Edit that script, not this file. -->",
        f'<robot name="{world["name"]}">',
        '  <link name="sim_world"/>',
    ]
    for ob in world["obstacles"]:
        if ob["shape"] == "cylinder":
            geometry = f'<cylinder radius="{ob["r"]:g}" length="{ob["h"]:g}"/>'
        else:
            geometry = f'<box size="{ob["sx"]:.5g} {ob["sy"]:.5g} {ob["h"]:g}"/>'
        z = ob["z"] + ob["h"] / 2
        lines += [
            f'  <joint name="{ob["name"]}_joint" type="fixed">',
            f'    <parent link="sim_world"/><child link="{ob["name"]}"/>',
            f'    <origin xyz="{ob["x"]:.5g} {ob["y"]:.5g} {z:g}" rpy="0 0 {ob["yaw"]:.5g}"/>',
            "  </joint>",
            f'  <link name="{ob["name"]}">',
            f"    <visual><geometry>{geometry}</geometry>"
            f'<material name="{ob["name"]}_color"><color rgba="{rgb(ob["color"])}"/></material></visual>',
            "  </link>",
        ]
    lines.append("</robot>")
    (LAUNCH_DIR / f"{world['name']}_env.urdf").write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    names = sys.argv[1:] or list(WORLDS)
    for name in names:
        if name not in WORLDS:
            raise SystemExit(f"Unknown world '{name}', expected one of: {', '.join(WORLDS)}")
        world = WORLDS[name]
        print(f"{name}: checking the robot can reach every area")
        check_reachable(world)
        write_sdf(world)
        write_urdf(world)
        print(f"  wrote launch/{name}.sdf and launch/{name}_env.urdf")

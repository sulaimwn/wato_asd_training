# Course runs

How the numbers in the main README's results table are measured. Each world
has a fixed list of goals, picked to be awkward: turning tightly around the end
of something, squeezing through a gap, turning round on the spot. The script
sends them one at a time and times how long the robot takes to reach each one.
It can also switch exploration on and time how long the robot takes to map the
whole world.

The important part is that nothing is measured from the robot's own map.
Obstacles are read straight from the world file Gazebo is running, and the
robot's shape (chassis and wheels) from `robot_env.sdf`. So "closest" is the
real gap between the robot's body and the nearest real thing, not what the
robot thinks it is.

## Running it

Start the stack fresh, so the robot is at the spawn point with an empty map,
then run the course for whichever world is up (`WORLD` in `watod-config.sh`):

```bash
./watod down && ./watod up -d
tools/course/run_course.sh            # the goal course
tools/course/run_course.sh explore    # or: explore the whole world
```

Don't touch the robot while it runs. A course takes 2 to 4 minutes, and so
does exploring.

## What you get

A line per goal as it goes, then a summary, all saved to
`tools/course/results/<world>_<mode>.json` (git ignores that folder):

```
ok   office (through its side door)                     23.5 s  closest 0.34 m (crate_1 at (5.49, -7.45))
ok   back to the hall                                   12.9 s  closest 1.02 m (wall_hall_s1 at (-9.4, -4.23))
SUMMARY {"world": "warehouse", "mode": "course", "reached": 5, "goals": 5, "seconds": 123.6, ...}
```

| Field | Meaning |
|---|---|
| `reached` / `goals` | Goals reached. A goal counts once the planner says it's there and the robot is within 0.7 m of it. It fails if it takes over 150 s, or the planner has no route for 15 s. |
| `seconds`, `driven_m` | Time for the whole run and how far the axle travelled. |
| `closest_m`, `closest_to`, `closest_at` | Smallest gap between the body and any obstacle over the whole run, what it was, and where the robot was. Each goal also has its own. |
| `map_phantom_cells` | Obstacle cells on the robot's map more than 0.2 m from any real surface: walls it thinks are there but aren't. Out of `map_obstacle_cells`. |
| `max_tilt_deg` | Worst pitch or roll. If the robot tips, the lidar sees the floor as a wall. |
| `status`, `coverage` | Exploring only: whether the explorer finished, and how much of the open floor ended up on the map. |

## The courses

| World | Goals |
|---|---|
| `default` | Behind the robot past the small cylinder, through the gap under the south box, between the two east boxes, round the NE box, across to the NW corner, 1.3 m off the big cylinder, back to the start. |
| `warehouse` | Round the end of a rack into the storage room's upper aisle, through the storage room's opening into the pallet room, the loading dock, the office through its side door, back to the hall. |
| `watonomous` | Round the far end of the word, under the middle of it, round the near end, over the middle, back to the start. |

`drive_course.py` is plain `rclpy` and runs inside the robot container, which
is all `run_course.sh` does: copy it and the world file in, run it, copy the
results out.

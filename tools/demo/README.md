# Demo recording tools

How [`docs/demo.gif`](../../docs/demo.gif) is made. With the stack running in the
warehouse world (`./watod up -d`), run:

```bash
tools/demo/make_demo.sh
```

It turns exploration on, records the run, and writes `docs/demo.gif` and
`docs/demo.mp4` (about 5 minutes, most of it the robot exploring). Don't drive
the robot while it records.

| File | What it does |
|---|---|
| `record_run.py` | Runs inside the robot container. Once a second it saves a top-down PNG of `/map` (with the robot, planned path, frontiers and goal drawn on it), the robot's `/camera` image, and a line of stats. It also measures how close the robot's **body** gets to any obstacle, using the true obstacle positions from `src/gazebo/tools/make_warehouse.py`, and writes a `summary.json` at the end. |
| `compose_frames.py` | Lays each frame out with the camera, explorer state, progress and legend (needs Pillow; `make_demo.sh` installs it in a throwaway container). |
| `make_demo.sh` | Runs the two above, then encodes the frames with ffmpeg (from the Gazebo image). |

`record_run.py` works for any run, not just demos: `summary.json` reports the
time taken, map coverage, and the closest the robot came to hitting something.

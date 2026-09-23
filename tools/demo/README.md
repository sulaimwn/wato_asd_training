# Demo recording tools

How the GIFs in [`docs/`](../../docs) are made. Start the stack in the world you
want to record (`WORLD` in `watod-config.sh`: `watonomous` or `warehouse`), then run:

```bash
./watod up -d
tools/demo/make_demo.sh
```

It turns exploration on, records the run, and writes `docs/demo_<world>.gif` and
`docs/demo_<world>.mp4`. That takes about 5 minutes, most of it the robot
exploring. Don't drive the robot while it records.

| File | What it does |
|---|---|
| `record_run.py` | Runs inside the robot container. Once a second it saves a top-down PNG of `/map` (with the robot, planned path, frontiers, and goal drawn on it), the robot's `/camera` image, and a line of stats. It also measures how close the robot's **body** gets to any obstacle, using the world's true geometry from `src/gazebo/tools/make_worlds.py`, and writes a `summary.json` at the end. |
| `compose_frames.py` | Lays out each frame with the camera, explorer state, progress, and legend. Square maps put everything else in a column on the right; wide maps put it underneath. Needs Pillow, which `make_demo.sh` installs in a throwaway container. |
| `make_demo.sh` | Runs the two above, then encodes the frames with ffmpeg (from the Gazebo image). |

`record_run.py` works for any run, not just demos. Its `summary.json` reports the
time taken, map coverage, and the closest the robot came to hitting something.

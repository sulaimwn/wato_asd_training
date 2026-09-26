# WATonomous ASD Admission Assignment

A simulated robot that drives wherever you click without hitting anything, and can **explore and map a building it's never seen on its own**. ROS 2 Humble, Gazebo, Foxglove, all in Docker.

![The robot exploring on its own until the map spells WATONOMOUS](docs/demo_watonomous.gif)

*Exploring from an empty map, sped up 12×. From above, the walls spell a word. (Recorded on my previous version, at half the current speed.)*

## What's in it

| Node | What it does |
|---|---|
| `costmap` | Lidar scan → 30 m grid of walls, empty space, and "not seen yet". |
| `map_memory` | Stitches those grids into one 60 × 40 m map, placing each scan where the robot was when it was taken. |
| `planner` | A\* for the robot's **whole body** (2 m × 1.4 m), kept 0.3 m from everything. Corners become arcs; it only turns on the spot where there's room. |
| `control` | Pure pursuit from the wheel axle at up to 1 m/s. Checks it has room before turning on the spot. |
| `explorer` | *(extra)* Keeps driving to the edge of the unknown until the map is done, then goes home. |

Also extra: two new worlds (`warehouse`, and `watonomous`, which spells the word), generated from one Python script.

## Running it

```bash
./watod build && ./watod up
```

Open [Foxglove](https://app.foxglove.dev) → `ws://localhost:20000` → import `config/wato_asd_training_foxglove_config.json`. Click with *Publish point* to send it somewhere, or hit **Start exploring**.

Pick a world with `WORLD` in `watod-config.sh` (`watonomous`, `warehouse`, or `default`), then `./watod down && ./watod up`.

**Tests:**

```bash
docker run --rm ghcr.io/watonomous/wato_asd_training/robot:main ros2 run planner planner_test
docker run --rm ghcr.io/watonomous/wato_asd_training/robot:main ros2 run explorer explorer_test
tools/course/run_course.sh   # drives a fixed course in the running world and measures it
```

## Bugs I hit and how I fixed them

| What I saw | Why | Fix |
|---|---|---|
| Drove straight into the big cylinder | First lidar scans are empty, and one became the whole map | Ignore empty scans |
| Body scraped the cylinder (0.06 m) | Planner treated the robot as a dot | Now plans for the whole body |
| Back of the robot cut corners like a trailer (0.11 m from a wall) | Controller steered the lidar, 1.3 m ahead of where the robot turns | Plan and steer from the wheel axle; round corners into arcs the body fits |
| Walls on the map that weren't there (up to 82 cells a run) | Scans placed with odometry older than the scan; while turning that's over 1 m off | Wait for odometry on both sides of the scan and interpolate → 0 fake walls |
| Stuck at the end of a wall for 2.5 min | Planner put a turn-on-the-spot where there was no room | Only plan turns where the body fits, checked every 5° |
| Rocked back and forth instead of turning | Picked a turn direction fresh every 0.1 s | Commit to a direction |
| Flip-flopped between two routes | Replans picked whichever was marginally cheaper | Keep the current route unless a new one is 1 m shorter |
| Looped around goals, passed 0.12 m from the cylinder | Insisted on the exact goal cell | Anywhere within 0.25 m counts |
| Timed out on long trips | 0.5 m/s and a 2 min limit | 1 m/s, 5 min limit |
| Map knew what was behind unseen walls | Everything in range counted as empty | Only cells a beam passed through count |
| Explorer skipped a room | Fake frontier from the safety zone around unseen walls | Only frontiers on floor the lidar actually saw |
| Explorer gave up on goals behind the word | Measured progress in a straight line | Measure it along the planned route |
| Crashed through a gap between the A and the T | Gap fit the old planner but not the robot | Straight-sided letters; world generator rejects gaps like that |

## Results

Measured with `tools/course/`: the robot's real shape against the real obstacles from the world file, not its own map. One run each.

| World | Test | Before | After |
|---|---|---|---|
| Original arena | 7 goals | 3:41, closest 0.52 m | **2:04**, closest **0.55 m** |
| | Explore | 3:39, closest 0.30 m | **1:56**, closest **0.81 m** |
| Warehouse | 5 goals | 3:27, closest 0.28 m | **2:04**, closest **0.34 m** |
| | Explore | 2:52, closest 0.16 m | **1:40**, closest **0.53 m** |
| Watonomous | 5 goals | 4/5 (one timed out), 6:52, closest 0.37 m | **5/5, 3:38**, closest **0.73 m** |
| | Explore | 4:44, closest 0.28 m | **2:35**, closest **0.60 m** |

Every closest call went up, and fake walls went from up to 82 per run to 0. Most of the time saved is just the higher speed. Unit tests: planner 8/8, explorer 8/8.

## Not perfect yet

- Pure pursuit still shaves arcs slightly (the 0.3 m margin covers it).
- It only plans driving forwards; backing up is a last resort.
- The lidar can't see anything under 0.9 m tall.

## Where things are

```
src/robot/       costmap, map_memory, planner, control, explorer (+ unit tests)
src/gazebo/      worlds and the script that generates them
tools/course/    measured test runs
tools/demo/      GIF recording
```

## Credits

Built on the [WATonomous](https://www.watonomous.ca/) ASD admission assignment (robot, Docker setup, node skeletons). Simulation by [Gazebo](https://gazebosim.org/), visualization by [Foxglove](https://foxglove.dev/).

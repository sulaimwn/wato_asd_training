#!/bin/bash
# Record an autonomous exploration run and turn it into docs/demo.gif (for the
# README) and docs/demo.mp4 (for sharing).
#
# Start the stack in the warehouse world first, and don't touch the robot
# while it records:
#     ./watod up -d          (WORLD=warehouse in watod-config.sh)
#     tools/demo/make_demo.sh
#
# Takes about 5 minutes: ~4 of exploring, then composing and encoding.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
ROBOT="${ROBOT_CONTAINER:-watod_${USER}-robot-1}"
ROBOT_IMAGE="ghcr.io/watonomous/wato_asd_training/robot:main"
GAZEBO_IMAGE="ghcr.io/watonomous/wato_asd_training/gazebo_server:main"
WORK="$(mktemp -d)"
ME="$(id -u):$(id -g)"

ros() { docker exec "$ROBOT" bash -c "source /opt/watonomous/setup.bash && $1"; }

echo "==> Recording from $ROBOT"
docker cp "$REPO/tools/demo/record_run.py" "$ROBOT:/tmp/record_run.py"
docker cp "$REPO/src/gazebo/tools/make_warehouse.py" "$ROBOT:/tmp/make_warehouse.py"
docker exec "$ROBOT" rm -rf /tmp/demo_run
docker exec -d "$ROBOT" bash -c \
  "source /opt/watonomous/setup.bash && python3 /tmp/record_run.py /tmp/demo_run 1 1200 > /tmp/demo_run.log 2>&1"
sleep 3
ros "ros2 topic pub --once /explore/enable std_msgs/msg/Bool '{data: true}'" > /dev/null
echo "    exploring... (progress every 10 s)"
until docker exec "$ROBOT" test -f /tmp/demo_run/summary.json; do
  sleep 10
  docker exec "$ROBOT" tail -n 1 /tmp/demo_run.log 2>/dev/null | sed 's/^/    /' || true
done
docker cp "$ROBOT:/tmp/demo_run" "$WORK/run"
echo "==> Run summary"; cat "$WORK/run/summary.json"; echo

echo "==> Composing frames"
docker run --rm -v "$WORK:/work" -v "$REPO/tools/demo:/tools:ro" "$ROBOT_IMAGE" bash -c \
  "apt-get update -qq && apt-get install -qq -y python3-pil fonts-dejavu-core > /dev/null \
   && python3 /tools/compose_frames.py /work/run /work/frames && chown -R $ME /work"

echo "==> Encoding docs/demo.gif and docs/demo.mp4"
mkdir -p "$REPO/docs"
# 1 frame per second of the run, played at 12 fps (12x speed); hold the end 3 s
docker run --rm --user "$ME" -v "$WORK:/work" -v "$REPO/docs:/out" --entrypoint bash "$GAZEBO_IMAGE" -c "
  ffmpeg -loglevel error -y -framerate 12 -i /work/frames/frame_%04d.png \
    -vf 'tpad=stop_mode=clone:stop_duration=3,scale=780:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle' \
    -loop 0 /out/demo.gif &&
  ffmpeg -loglevel error -y -framerate 12 -i /work/frames/frame_%04d.png \
    -vf 'tpad=stop_mode=clone:stop_duration=3' -c:v libx264 -pix_fmt yuv420p -crf 20 -movflags +faststart /out/demo.mp4"

ls -lh "$REPO/docs/demo.gif" "$REPO/docs/demo.mp4"
rm -rf "$WORK"
echo "==> Done"

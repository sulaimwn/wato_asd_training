#!/bin/bash
# Drive the running world's goal course (or explore the whole world) and
# measure how it went. See tools/course/README.md.
#
# Start the stack fresh first, so the robot is at the spawn point with an
# empty map, and leave the robot alone while it runs:
#     ./watod down && ./watod up -d
#     tools/course/run_course.sh            # the goal course
#     tools/course/run_course.sh explore    # explore the whole world instead
set -euo pipefail

MODE="${1:-course}"
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
ROBOT="${ROBOT_CONTAINER:-watod_${USER}-robot-1}"
GAZEBO="${GAZEBO_CONTAINER:-watod_${USER}-gazeboserver-1}"
WORLD="$(docker exec "$GAZEBO" printenv WORLD)"  # the world that is actually running
case "$WORLD" in
  default) SDF=robot_env.sdf ;;
  *) SDF="$WORLD.sdf" ;;
esac
OUT="$REPO/tools/course/results/${WORLD}_${MODE}.json"

echo "==> Running the $WORLD $MODE from $ROBOT"
docker cp "$REPO/tools/course/drive_course.py" "$ROBOT:/tmp/drive_course.py"
docker cp "$REPO/src/gazebo/launch/$SDF" "$ROBOT:/tmp/$SDF"
docker exec "$ROBOT" bash -c \
  "source /opt/watonomous/setup.bash && python3 -u /tmp/drive_course.py /tmp/$SDF $MODE --out /tmp/course.json"
mkdir -p "$(dirname "$OUT")"
docker cp "$ROBOT:/tmp/course.json" "$OUT" > /dev/null
echo "==> Saved ${OUT#"$REPO"/}"

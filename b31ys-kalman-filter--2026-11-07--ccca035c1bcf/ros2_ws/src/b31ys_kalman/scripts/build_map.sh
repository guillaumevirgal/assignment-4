#!/bin/bash
# Build a map of the simulated world with slam_toolbox, driving the robot
# with the keyboard, then save it with the Nav2 map saver.
#
# Usage:
#   ros2 run b31ys_kalman build_map.sh [map_name] [mapping.launch.py args...]
#
# Examples:
#   ros2 run b31ys_kalman build_map.sh
#   ros2 run b31ys_kalman build_map.sh my_map gz_headless_mode:=False
#   ros2 run b31ys_kalman build_map.sh my_map sim:=false      # sim already running
#
# The map is saved as <map_name>.pgm + <map_name>.yaml in $MAP_DIR,
# default /output (Docker) or the current folder.
# Do not run student_trajectory at the same time: it also drives the robot.

MAP_NAME="${1:-husarion_world_map}"
shift || true
if [ -n "$MAP_DIR" ]; then
    OUT_DIR="$MAP_DIR"
elif [ -d /output ] && [ -w /output ]; then
    OUT_DIR=/output
else
    OUT_DIR="$(pwd)"
fi
mkdir -p "$OUT_DIR"
LOG="$OUT_DIR/${MAP_NAME}_mapping.log"

# Make sure ROS and the workspace are sourced
if ! command -v ros2 >/dev/null 2>&1; then
    source /opt/ros/humble/setup.bash
fi
if ! ros2 pkg prefix b31ys_kalman >/dev/null 2>&1; then
    for ws in /ros2_ws ~/ros2_ws; do
        [ -f "$ws/install/setup.bash" ] && source "$ws/install/setup.bash" && break
    done
fi

# 1. Simulation + slam_toolbox + RViz, in their own process group so the
#    keyboard does not reach them
setsid ros2 launch b31ys_kalman mapping.launch.py "$@" > "$LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() {
    echo "Stopping simulation and slam_toolbox..."
    kill -INT -- -"$LAUNCH_PID" 2>/dev/null
    for _ in $(seq 1 20); do
        kill -0 "$LAUNCH_PID" 2>/dev/null || return
        sleep 0.5
    done
    kill -KILL -- -"$LAUNCH_PID" 2>/dev/null
}
trap cleanup EXIT
trap 'true' INT

# 2. Wait until slam_toolbox publishes a map
echo "Starting simulation and slam_toolbox (log: $LOG)..."
for i in $(seq 1 120); do
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo "Launch failed, see $LOG"; exit 1
    fi
    if ros2 topic info /map 2>/dev/null | grep -q "Publisher count: [1-9]"; then
        break
    fi
    if [ "$i" -eq 120 ]; then
        echo "No /map after 120 s, see $LOG"; exit 1
    fi
    sleep 1
done
sleep 3

# 3. Drive
cat <<EOF

=====================================================================
 Mapping is running. Drive the robot around the whole environment.
 Watch the map grow in RViz. Go slowly, and revisit the start area
 at the end so slam_toolbox can close the loop.

 Keys (this terminal must have focus):
   i forward   , backward   j / l turn   k stop
   q / z faster / slower (keep linear speed around 0.2-0.3 m/s)

 Press Ctrl+C when done: the map is then saved to
   $OUT_DIR/$MAP_NAME.pgm / .yaml
=====================================================================

EOF
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -p speed:=0.25 -p turn:=0.8 || true

# Stop the robot
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1

# 4. Save with the Nav2 map saver
echo "Saving map to $OUT_DIR/$MAP_NAME ..."
for attempt in 1 2 3; do
    if ros2 run nav2_map_server map_saver_cli -f "$OUT_DIR/$MAP_NAME" \
            --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0; then
        # In Docker we run as root: give the files to the folder's owner
        chown --reference="$OUT_DIR" "$OUT_DIR/$MAP_NAME".* "$LOG" 2>/dev/null
        echo "Map saved: $OUT_DIR/$MAP_NAME.pgm and $OUT_DIR/$MAP_NAME.yaml"
        exit 0
    fi
    echo "Map saver attempt $attempt failed, retrying..."
    sleep 2
done
echo "Could not save the map, see $LOG"
exit 1

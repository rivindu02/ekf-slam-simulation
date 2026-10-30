#!/usr/bin/env bash
# Run INSIDE devbox. Uses its ROS Python, without importing Matplotlib.
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-74}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11355}"
export GAZEBO_MODEL_DATABASE_URI=""
DEMO_OUTPUT_DIR="${SLAM_OUTPUT_DIR:-$PROJECT_DIR/results/gazebo}"
export ROS_LOG_DIR="$DEMO_OUTPUT_DIR/ros_logs"
mkdir -p "$ROS_LOG_DIR"
MODE="${1:-gui}"
DURATION="${2:-105}"
gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so "$PROJECT_DIR/gazebo/slam_lab.world" > "$DEMO_OUTPUT_DIR/server.log" 2>&1 &
SERVER_PID=$!
CLIENT_PID=""
RVIZ_PID=""
cleanup() {
    if [[ -n "$RVIZ_PID" ]]; then kill "$RVIZ_PID" 2>/dev/null || true; fi
    if [[ -n "$CLIENT_PID" ]]; then kill "$CLIENT_PID" 2>/dev/null || true; fi
    kill "$SERVER_PID" 2>/dev/null || true
    sleep 1
    kill -KILL "$SERVER_PID" 2>/dev/null || true
    wait "$SERVER_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
if [[ "$MODE" == gui ]]; then
    # Gazebo Classic OGRE embeds X11 windows; native Wayland breaks the handle.
    QT_QPA_PLATFORM=xcb gzclient --verbose > "$DEMO_OUTPUT_DIR/client.log" 2>&1 &
    CLIENT_PID=$!
    if command -v rviz2 >/dev/null 2>&1; then
        QT_QPA_PLATFORM=xcb rviz2 -d "$PROJECT_DIR/gazebo/slam.rviz" > "$DEMO_OUTPUT_DIR/rviz.log" 2>&1 &
        RVIZ_PID=$!
        echo "Gazebo shows the physical robot. RViz shows green detection rays, blue landmarks, IDs and EKF path."
    fi
fi
python3 "$PROJECT_DIR/gazebo/slam_node.py" --duration "$DURATION" --output-dir "$DEMO_OUTPUT_DIR"

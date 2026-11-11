#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${1:-${ROS_DISTRO:-}}"

if [[ -z "$ROS_DISTRO" ]]; then
    echo "Usage: ./scripts/check.sh noetic|humble|jazzy"
    exit 1
fi

source "/opt/ros/$ROS_DISTRO/setup.bash"

WORKSPACE="$HOME/turtlebot3_lab_ws"

if [[ "$ROS_DISTRO" == "noetic" ]]; then
    source "$WORKSPACE/devel/setup.bash"

    echo "=== ROS 1 TOPICS ==="
    rostopic list
    echo
    echo "=== LASER SCAN ==="
    rostopic echo -n 1 /scan
else
    source "$WORKSPACE/install/setup.bash"

    echo "=== ROS 2 TOPICS ==="
    ros2 topic list
    echo
    echo "=== LASER SCAN ==="
    ros2 topic echo /scan --once
fi

#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${1:-${ROS_DISTRO:-}}"
WORLD="${2:-world}"

if [[ -z "$ROS_DISTRO" ]]; then
    echo "Usage: ./scripts/launch.sh noetic|humble|jazzy [world|empty]"
    exit 1
fi

source "/opt/ros/$ROS_DISTRO/setup.bash"

WORKSPACE="$HOME/turtlebot3_lab_ws"

if [[ "$ROS_DISTRO" == "noetic" ]]; then
    source "$WORKSPACE/devel/setup.bash"
else
    source "$WORKSPACE/install/setup.bash"
fi

export TURTLEBOT3_MODEL=burger

echo "Starting TurtleBot3 simulation..."
echo "ROS: $ROS_DISTRO"
echo "World: $WORLD"

case "$ROS_DISTRO:$WORLD" in
    noetic:world)
        roslaunch turtlebot3_gazebo turtlebot3_world.launch
        ;;
    noetic:empty)
        roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
        ;;
    humble:world|jazzy:world)
        ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
        ;;
    humble:empty|jazzy:empty)
        ros2 launch turtlebot3_gazebo empty_world.launch.py
        ;;
    *)
        echo "Unsupported ROS/world combination."
        exit 1
        ;;
esac

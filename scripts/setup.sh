#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${1:-${ROS_DISTRO:-}}"

if [[ -z "$ROS_DISTRO" ]]; then
    echo "Usage: ./scripts/setup.sh noetic|humble|jazzy"
    exit 1
fi

case "$ROS_DISTRO" in
    noetic)
        UBUNTU_VERSION="20.04"
        BUILD_TOOL="catkin"
        ;;
    humble)
        UBUNTU_VERSION="22.04"
        BUILD_TOOL="colcon"
        ;;
    jazzy)
        UBUNTU_VERSION="24.04"
        BUILD_TOOL="colcon"
        ;;
    *)
        echo "Unsupported ROS distribution: $ROS_DISTRO"
        exit 1
        ;;
esac

source "/opt/ros/$ROS_DISTRO/setup.bash"

if [[ ! -f /etc/os-release ]]; then
    echo "Cannot detect Ubuntu version."
    exit 1
fi

. /etc/os-release

if [[ "$VERSION_ID" != "$UBUNTU_VERSION" ]]; then
    echo "Expected Ubuntu $UBUNTU_VERSION for ROS $ROS_DISTRO."
    echo "Detected: $PRETTY_NAME"
    exit 1
fi

echo "======================================"
echo " TurtleBot3 Lab Setup"
echo " ROS: $ROS_DISTRO"
echo " Ubuntu: $VERSION_ID"
echo "======================================"

sudo apt update

COMMON_PACKAGES=(
    git
    python3-rosdep
    python3-numpy
    python3-pip
    ros-${ROS_DISTRO}-robot-state-publisher
    ros-${ROS_DISTRO}-joint-state-publisher
    ros-${ROS_DISTRO}-xacro
)

if [[ "$ROS_DISTRO" == "noetic" ]]; then
    sudo apt install -y \
        "${COMMON_PACKAGES[@]}" \
        python3-catkin-tools \
        ros-noetic-gazebo-ros-pkgs \
        ros-noetic-gazebo-ros-control \
        ros-noetic-teleop-twist-keyboard
elif [[ "$ROS_DISTRO" == "humble" ]]; then
    sudo apt install -y \
        "${COMMON_PACKAGES[@]}" \
        python3-colcon-common-extensions \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-ros-control
elif [[ "$ROS_DISTRO" == "jazzy" ]]; then
    sudo apt install -y \
        "${COMMON_PACKAGES[@]}" \
        python3-colcon-common-extensions \
        ros-jazzy-ros-gz \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-ros-gz-bridge
fi

# Initialize rosdep only if it has not already been initialized.
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
fi

rosdep update

WORKSPACE="$HOME/turtlebot3_lab_ws"
SRC="$WORKSPACE/src"

mkdir -p "$SRC"
cd "$SRC"

clone_repo() {
    local repo="$1"
    local branch="$2"
    local url="$3"

    if [[ -d "$repo/.git" ]]; then
        echo "Already cloned: $repo"
    else
        git clone --depth 1 -b "$branch" "$url" "$repo"
    fi
}

# The repositories are kept on matching ROS branches.
clone_repo DynamixelSDK "$ROS_DISTRO" \
    https://github.com/ROBOTIS-GIT/DynamixelSDK.git

clone_repo turtlebot3_msgs "$ROS_DISTRO" \
    https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git

clone_repo turtlebot3 "$ROS_DISTRO" \
    https://github.com/ROBOTIS-GIT/turtlebot3.git

clone_repo turtlebot3_simulations "$ROS_DISTRO" \
    https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

cd "$WORKSPACE"

rosdep install \
    --from-paths src \
    --ignore-src \
    -r -y \
    --rosdistro "$ROS_DISTRO"

if [[ "$BUILD_TOOL" == "catkin" ]]; then
    catkin_make
else
    colcon build --symlink-install
fi

echo
echo "======================================"
echo " TurtleBot3 setup completed!"
echo "======================================"

echo
echo "To launch the simulation:"
echo "  ./scripts/launch.sh"
echo
echo "To check topics:"
echo "  ./scripts/check.sh"

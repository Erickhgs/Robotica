#!/usr/bin/env bash
set -e

cd "$(dirname "$(readlink -f "$0")")"
source /opt/ros/humble/setup.bash

SUDO=sudo
[ "$(id -u)" -eq 0 ] && SUDO=

build() {
    colcon build --symlink-install
    source install/setup.bash
}

comando="${1:-sim}"
[ $# -gt 0 ] && shift

case "$comando" in
    deps)
        $SUDO apt-get update
        $SUDO apt-get install -y python3-rosdep python3-colcon-common-extensions python3-pip
        [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || $SUDO rosdep init
        rosdep update
        rosdep install --from-paths src --ignore-src -y
        ;;
    deps04)
        $SUDO apt-get update
        $SUDO apt-get install -y ros-humble-gazebo-plugins ros-humble-rviz2 \
            ros-humble-plotjuggler-ros ros-humble-tf2-tools ros-humble-ros2bag \
            ros-humble-rosbag2-storage-default-plugins graphviz
        ;;
    build)
        build
        ;;
    test)
        build
        colcon test --packages-select robot_control robot_state_estimation
        colcon test-result --verbose
        ;;
    sim)
        build
        ros2 launch robot_description bringup.launch.py "$@"
        ;;
    sdf)
        build
        ros2 launch robot_description gazebo_sdf.launch.py "$@"
        ;;
    teleop)
        ros2 run teleop_twist_keyboard teleop_twist_keyboard
        ;;
    pratica03)
        build
        ros2 launch robot_control pratica03.launch.py gazebo:=true output_dir:="$PWD/resultados" "$@"
        ;;
    pratica04)
        build
        ros2 launch robot_control pratica04.launch.py output_dir:="$PWD/resultados/pratica04" "$@"
        ;;
    analyze04)
        source install/setup.bash
        ros2 run robot_control analyze_sensors "$@"
        ;;
    *)
        echo "uso: ./run.sh [deps|deps04|build|test|sim|sdf|teleop|pratica03|pratica04|analyze04] [argumentos]"
        exit 1
        ;;
esac

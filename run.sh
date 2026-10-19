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
    build)
        build
        ;;
    test)
        build
        colcon test --packages-select robot_control
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
    *)
        echo "uso: ./run.sh [deps|build|test|sim|sdf|teleop|pratica03] [argumentos do launch]"
        exit 1
        ;;
esac

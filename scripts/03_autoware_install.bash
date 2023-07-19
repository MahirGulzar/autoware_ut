#!/bin/bash
set -e

# This script automates the following instructions:
# https://gitlab.com/autowarefoundation/autoware.ai/autoware/wikis/Source-Build

# This script works on ubuntu 16 and 18

main()
{
    UBUNTU_RELEASE=$(lsb_release -s -c)

    case $UBUNTU_RELEASE in
        xenial)
            ROS_DISTRO=kinetic;;
        bionic)
            ROS_DISTRO=melodic;;
        *)
            echo "Unable to match Ubuntu version '$UBUNTU_RELEASE' to a ROS version"
        exit 1
    esac

    dependencies
    autoware
}

dependencies() {
    echo "Installing autoware dependencies ..."
    # Common deps
    sudo apt-get -qq install python-catkin-pkg python-rosdep ros-$ROS_DISTRO-catkin
    sudo apt-get -qq install python3-pip python3-colcon-common-extensions python3-setuptools python3-vcstool
    # uninstall whatever setuptools was installed before
    # new setuptools may end up installed side-by-side with previous one
    # and the versions may interfere
    pip3 uninstall -y setuptools
    pip3 install -U --user setuptools==57.5.0
    pip3 install -U --user pandacan
    # radar driver
    sudo apt-get -qq install ros-$ROS_DISTRO-delphi-esr

    # kinetic only
    if [ "$ROS_DISTRO" == "kinetic" ]; then
        sudo apt-get -qq install gksu
    fi
}

autoware() {

    # Absolute path to this script. /home/user/bin/foo.sh
    SCRIPT=$(readlink -f $0)
    # Absolute path this script is in. /home/user/bin
    SCRIPTPATH=$(dirname $SCRIPT)
    MISC_DIR=$(dirname "$SCRIPTPATH")

    mkdir -p "$HOME/autoware.ai/src"
    cd "$HOME/autoware.ai"

    echo "Downloading autoware repos ..."
    vcs import src < $MISC_DIR/autoware.ai.repos -w 1

    source /opt/ros/$ROS_DISTRO/setup.bash
    rosdep update
    rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

    AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release |& tee    build.log
}

main

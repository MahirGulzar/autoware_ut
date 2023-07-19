#!/bin/bash
set -e

# Installing dependancies
# This line is duplicate of the dep install of the autoware update script, if you change this one, update it there as well.
export LIBRARY_PATH=$LIBRARY_PATH:/usr/local/cuda-10.0/lib64/
export CPATH=$CPATH:/usr/local/cuda-10.0/targets/x86_64-linux/include
pip install keras==2.2.4 pycuda pyproj==2.2.2
pip3 install rospkg

# adding ouster and velodyne dependencies
echo "Installing Ouster and Velodyne dependencies ..."
sudo apt install build-essential cmake libglfw3-dev libglew-dev libeigen3-dev libjsoncpp-dev libtclap-dev

create_ws () {
    mkdir $PKGS_NAME
    cd $PKGS_NAME
    mkdir src
    cp $SCRIPTPATH/autoware_ut.repos .
    vcs import src < autoware_ut.repos
    # changing from local_setup.bash to setup.bash
    source ~/autoware.ai/install/setup.bash
    rosdep update && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
}

# Assuming that PWD is in $PKGS_NAME
build_ws () {
    catkin build
}

source_ws () {
    printf "Would you like to add $PKGS_NAME/devel/setup.bash into ~/.bashrc? (y/N) "
    read answer
    if [ "$answer" != "${answer#[Yy]}" ] ;then
        echo "source $ABS_PATH_WS/$PKGS_NAME/devel/setup.bash" >> ~/.bashrc
        echo "Added setup.bash into ~/.bashrc"
    fi
    echo Done!
}

# Get path
DEF_PATH=~/
PKGS_NAME=autoware_ut
printf "Specify path for $PKGS_NAME: (default $DEF_PATH)`echo $'\n> '`"
read -e PATH_WS
PATH_WS=${PATH_WS:-$DEF_PATH}
PATH_WS="${PATH_WS/#\~/$HOME}"
ABS_PATH_WS=$(readlink -f $PATH_WS)

SCRIPTPATH="$(dirname $( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P ))"

echo scriptpath: "$SCRIPTPATH"
echo Using path: "$ABS_PATH_WS"

# Check directory
if [[ -d "$ABS_PATH_WS" ]]
then
    echo "Creating workspace $ABS_PATH_WS/$PKGS_NAME"
    cd $ABS_PATH_WS
    create_ws

    cd $ABS_PATH_WS/$PKGS_NAME
    build_ws

    echo $ABS_PATH_WS/$PKGS_NAME/devel/setup.bash
    source_ws

else
    echo "Directory $ABS_PATH_WS doesn't exist. Exiting."
fi

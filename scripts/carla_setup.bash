#!/bin/bash
set -e

################################
#### Create Carla Workspace ####
################################

create_ws() {

    if [ ! -d "$CARLA_WS" ]; then
        printf "Creating workspace at $CARLA_WS \n"
        mkdir -p $CARLA_WS
    fi

    if [ -z "$(ls -A $CARLA_WS)" ]; then
        cd $CARLA_WS
        mkdir src
        vcs import --recursive $CARLA_WS/src <$SCRIPTPATH/carla.repos
    else
        echo "Repositories already exists.."
    fi

}

###############################
#### Setup Scenario Runner ####
###############################

setup_scenario_runner(){
    cd ~/
    if [ -z "$(ls -A ~/scenario_runner)" ]; then
        echo "Acquiring scenario runner repository.."
        git clone https://github.com/carla-simulator/scenario_runner.git
    else
        echo "Scenario runner already exist.."
    fi
    cd scenario_runner
    pip install --user -r requirements.txt
}

############################################################
#### Prepare virtual envrionment with required packages ####
############################################################

prepare_virtual_env() {
    cd $SCRIPTPATH
    echo "Preparing virtual environment.."
    if conda env list | grep ".*$ENV_NAME.*" >/dev/null 2>&1; then
        echo "$ENV_NAME already exists.. skipping creation.."
        eval "$(conda shell.bash hook)"
        conda activate $ENV_NAME
    else
        conda create -n $ENV_NAME python=3.7 pyaml numpy opencv pexpect pyproj -y
        eval "$(conda shell.bash hook)"
        conda activate $ENV_NAME
        # Install required packages by carla ros in this virtual environment
        pip install --user -r carla_requirements.txt
        # Setup Scenario Runner i.e. install required pacakges by scenario runner within this virtual environment
        setup_scenario_runner
    fi
    # Use python 3 as default for building this workspace
    export ROS_PYTHON_VERSION=3
    # Remove specific python version to avoid cmake build errors
    sed -i 's/python37/python3/g' $CARLA_WS/src/cv_bridge/cv_bridge/CMakeLists.txt
}

# Assuming that CONDA_ROOT points to installed path of virtual envronment (currently its ~/miniconda3/envs)
build_ws() {
    echo "Building workspace.."
    cd $CARLA_WS
    rosdep update && rosdep install --from-paths src --ignore-src -r
    catkin_make -DPYTHON_EXECUTABLE:FILEPATH=$CONDA_ROOT/$ENV_NAME/bin/python
    echo "Carla Workspace built successfully!"
}

CARLA_WS=~/carla_ws
SCRIPTPATH=$(pwd)
#TODO find a better way to do this
CONDA_ROOT=~/miniconda3/envs
ENV_NAME=carla_env

create_ws
prepare_virtual_env
build_ws

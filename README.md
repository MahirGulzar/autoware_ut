# UT ADL Autoware launch files


## Intro and Installation

### Introduction

`autoware_ut` package contains all the necessary launch and configuration files to run **autoware.ai** on the UT ADL Lexus car or in the simulation mode with our custom configuration. It consists of the following folders:

- `/config` - includes parameters for nodes organized in separate files that will be loaded at the end of corresponding launch files, for example `control.launch` will load `control.yaml` file.
- `/launch` - launch files to run different platforms (`sim_mode`, `lexus`), planners (`openplanner`, `decisionmaker`) or just sensors visualization with the car.
- `/maps` - includes map files (Autoware vector maps, Lanelet2 maps, pointcloud maps and waypoint files)
- `/nodes` - folder for nodes performing specific tasks, for example: lidar and radar fusion, traffic lights visualization, etc.
- `/rviz` - Rviz configuration files
- `/scripts` - smaller scripts not directly related to enabling autonomy on the car. 

### Installation

We have created some installation scripts for you to build this version of autoware_ut along with our custom forked repostories of autoware.ai. Kindly follow these steps to build autoware_ut.

- autoware_ut requires CUDA-10, you can use the helper script [`01_cuda_10_install.bash`](https://github.com/MahirGulzar/autoware_ut/tree/master/scripts/01_cuda_10_install.bash) for installing the required specific version of CUDA
- If tensorflow is not installed then use the helper script [`02_tensorflow_install.bash`](https://github.com/MahirGulzar/autoware_ut/tree/master/scripts/02_tensorflow_install.bash) for installing it.
- Next, the helper script [`03_autoware_install.bash`](https://github.com/MahirGulzar/autoware_ut/tree/master/scripts/03_autoware_install.bash) clones and builds autoware.ai with required packages from both original stack and ours.
- Lastly, the helper script [`04_ut_pkgs_install.bash`](https://github.com/MahirGulzar/autoware_ut/tree/master/scripts/04_ut_pkgs_install.bash) clones and builds autoware_ut required repositories. During build step, this script chains the autoware_ut's workspace with autoware.ai that is build in the previous step.
 
## Launch Files

### Main launch files and their structure
- `startup.launch` - launches the entire system with the option to set different arguments (`map_name`, `planner`, `localizer`, `controller`, etc.)
	- **`platform.launch`** - launches ROS drivers for sensors and static transforms describing the vehicle platform if `platform:=lexus`. Loads also parameters from `platform.yaml`.
	- `autonomy.launch` - includes lower level launch files depending on what options are set in `startup.launch`.
		- `map.launch` - launches different maps.
		- `localization.launch` - launches either GNSS or lidar based localization (ndt_matching).
		- `planning.launch` - depending on parameters either of the planners will be launched:
            - `openplanner.launch`
            - `decisionmaker.launch`
		- `control.launch` - launches one of the selected controllers: `mpc`, `purepursuit` or `stanley` and other nodes required for path following and control commands.
		- `detection.launch` - launches obstacle detection and traffic light recognition nodes.
		- `nodes.launch` - launches additional nodes like `webapp` for selecting driving destinations.
    - **`parameters.launch`** - loads all the necessary parameter from yaml files. It is the last file in the `autonomy.launch` to make sure that these are the last parameters loaded and set as values in ros parameter server and used by the nodes.


Most frequent parameters are included in the launch files. Many additional parameters are stored in [configuration files](#configuration-files) and they are loaded using the `parameters.launch` file.


### Other launch files

There are multiple other launch files in `/launch/tools`. Some examples:
- `decompress_image.launch` - launches nodes to decompress images for mako cameras.
- `record_waypoints.launch` - run nodes necessary to record waypoints.
- `joy_ssc.launch` - SSC joystick demo launch.
- `test_calibration.launch` - To quickly check the camera to lidar calibration.
- And others...


## Configuration files

Configuration files contain the main parameters for running the system that have been tuned for UT ADL Lexus car. The main configuration files are:
- `platform.yaml` - car / platform specific parameters. 
- `control.yaml` - car steering and trajectory following.
- `detection.yaml` - obstacle detection and traffic light detection.
- `localization.yaml` - localizing the car relative to the map. 
- `openplanner.yaml` - OpenPlanner related parameters. Currently, the main planner used.
- `decision_maker.yaml` - Decision Maker parameters, an alternative planner to OpenPlanner.


## Using the launch files

### Main arguments

Running the **autoware.ai** on the car or in the simulation mode would require setting the different parameters in `startup.launch`. This can be done by editing the launch file or setting the required parameter values on the command line.

Argument called `platform` is exactly for that with its three possible values:
- `lexus` - run drivers of the sensors in the real car and nodes related to the real car
- `sim_mode` - run nodes simulating the car's pose and movement
- `svl_sim` - run **autoware.ai** against svl platform

Command line example of running the real car on a demo route map would be:

```
roslaunch autoware_ut startup.launch platform:=lexus planner:=openplanner maxVelocity:=30 use_tl_detection:=true
```
In this example:
- `planner:=openplanner` select OpenPlanner to find the route (global planning) and do the local planning.
- `maxVelocity:=30` - do not allow to drive autonomously over 30 km/h
- `use_tl_detection:=true` - this enables traffic light detection

Similarly to `platform` there are three more important arguments where you can choose multiple options:
```
    <arg name="planner"           default="openplanner"   doc="{openplanner, decisionmaker}"/>
    <arg name="controller"        default="mpc"           doc="{purepursuit, stanley, mpc}" />
    <arg name="localizer"         default="gpsins"        doc="{gpsins, ndt}" />
```

- `planner` - select which planning stack is used for planning. Currently, there are two options:
  - `openplanner` - OpenPlanner stack ([OP global planner](https://gitlab.cs.ut.ee/autonomous-driving-lab/autoware.ai/core_planning/-/tree/ut/master/op_global_planner), [OP local planner](https://gitlab.cs.ut.ee/autonomous-driving-lab/autoware.ai/core_planning/-/tree/ut/master/op_local_planner), [op_planner](https://gitlab.cs.ut.ee/autonomous-driving-lab/autoware.ai/common/-/tree/ut/master/op_planner)). This has been mainly used in ADL.
  - `decisionmaker` - [Decision Maker](https://gitlab.cs.ut.ee/autonomous-driving-lab/autoware.ai/core_planning/-/tree/ut/master/decision_maker) stack
- `controller` - select which controller will be responsible for the path following. In general, when the destination is set for the car (in simulation or real world) the planner will do the global routing and find the path. The controller will be used to calculate the driving commands (mainly steering) to follow that path. Three options for the controller algorithms are:
  - `purepursuit` - [Pure Pursuit](https://gitlab.cs.ut.ee/autonomous-driving-lab/autoware.ai/core_planning/-/tree/ut/master/pure_pursuit)
  - `stanley` - [Stanley](https://gitlab.cs.ut.ee/autonomous-driving-lab/autoware.ai/core_planning/-/tree/ut/master/stanley_controller)
  - `mpc` - [MPC follower](https://gitlab.cs.ut.ee/autonomous-driving-lab/autoware.ai/core_planning/-/tree/ut/master/mpc_follower)
- `localizer` - select which localizer is used
  - `gpsins` - [gpsins_localizer](https://gitlab.cs.ut.ee/autonomous-driving-lab/autoware.ai/core_perception/-/tree/ut/master/gpsins_localizer) - works with the real car in outside (open sky visibility) conditions. Will use different GNSS satellite constellations to determine the car's position. This position will be used to locate the car on the map.
  - `ndt` - will launch [lidar_localizer](https://gitlab.cs.ut.ee/autonomous-driving-lab/autoware.ai/core_perception/-/tree/ut/master/lidar_localizer) in ndt_matching mode. This localization method assumes the presence of a pointcloud map and the localization is done by matching the current lidar scan (can be in real car or from simulation environment) with the pointcloud map. This match determines the location of the lidar, and by using the transform tree, it can also be transferred to the `base_link` frame.

### Run Openplanner in simulation mode

* To run Openplanner in simulation mode, we need to define `planner:=openplanner platform:=sim_mode`

```
roslaunch autoware_ut startup.launch planner:=openplanner platform:=sim_mode
```

* Let's also disable traffic lights detection and use Autoware vector map called `tartu` (most complete map with all the mapped lanes in Tartu).

```
roslaunch autoware_ut startup.launch planner:=openplanner platform:=sim_mode use_tl_detection:=false map_name:=tartu
```

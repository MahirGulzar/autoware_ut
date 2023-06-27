# Autoware Platform Package

This ros package contains info and launch files for running autoware.

# Launch Files

The launch files are organized using simple composition.
Individual launch files are created for each logical division of software, and then these files are combined together to launch the entire system.
For example there is the `localization.launch` file responsible for launching autoware localization modules, and then there is an `autonomy.launch` file responsible for launching all autoware components required for automated driving.
At the highest level is the `startup.launch` file which launches the entire system, but this file should only be used once the system is tested and stable.

**Most Important Launch Files**:  

- `startup.launch` - The entire system.
	- `platform.launch` - Launches ROS drivers and static transforms describing the vehicle platform.
	- `autonomy.launch` - Nodes responsible for vehicle autonomy. The included launch files match the autoware standard launch files (first tab of gui).
		- `map.launch`
		- `localization.launch`
		- `mission_planning.launch`
		- `motion_planning.launch`
		- `detection.launch`
		- `sensing.launch`

There are also some launch files for testing:

- `joy_ssc.launch` - SSC joystick demo.
- `test_calibration.launch` - To quickly check the camera to lidar calibration.
- `aw_gui_ssc.launch` - To control the vehicle from the autoware gui interface tab. This launch file requires the user to also launch the autoware gui in a separate terminal.

# Configs

Below is a list of configs that may need to change depending on the type of vehicle platform, map, and waypoints in use.

- `config/autonomy_params.yaml` - Configurations related to the Autoware autonomy stack such as map files to use, etc.
- `config/platform_params.yaml` - Platform-specific parameters such as vehicle wheelbase, sensor positions, etc. Copy the default param file and modify as needed.
- `config/calibration.yaml` - The autoware sensor calibration file.
- `config/yolov3.weights` - A symlink to (or the file itself) the weights file for yolo-based object classification. Only required when using classification.
- `config/tlr_model.hdf5` - The tensorflow model file for the traffic_light_recognition_tensorflow node.
- `map` - A symlink to the desired map folder to load. See the aw_data package README for more details.

To update the map symlink to point to the desired map, use the following command:
```
roscd common_platform
ln -sf ../aw_data/maps/desired_map map
```

# Using the launch files

### Following waypoints

1. Change the `&waypoints_file` parameter (found at the top of `config/autonomy_params.yaml`) to point to your desired waypoint file.
2. If needed, change any PCD or vector map files as described in the `Configs` section of this README.
3. Run the following command:

		roslaunch common_platform startup.launch
		roslaunch common_platform autonomy.launch use_gpsins_localizer:=true

4. Help localize the vehicle using the rviz `2D Pose Estimate` tool.
5. Drive the vehicle to a starting point on the waypoint path.
6. Enable autonomous control by running the following command:

	`rostopic pub /vehicle/engage std_msgs/Bool "data: true"`

To resume driving after reaching a `stop` status :

```text
rostopic pub /state_cmd std_msgs/String "data: 'operation_start'"
rostopic pub /state_cmd std_msgs/String "data: 'mission_is_compatible'"
rostopic pub /state_cmd std_msgs/String "data: 'received_mission_order'"
```


### Collecting Waypoints

1. Launch the following launch files:

		roslaunch common_platform platform.launch
		roslaunch common_platform autonomy_localization.launch

2. Help localize the vehicle using the rviz `2D Pose Estimate` tool.
3. Position the vehicle at the start of your desired path.
4. Run the following launch file:

		roslaunch common_platform record_waypoints.launch

5. Drive the desired path manually.
6. When complete, kill the `record_waypoints.lauch` launch file and copy the waypoint file from `/tmp/saved_waypoints.csv` to a safe location.

### Creating a PCD map

1. Start up the system platform to receive lidar and odometry data:

		roslaunch common_platform platform.launch

2. Make sure the E-Stop button is not pressed so that the PacMod can provide wheel speed feedback during mapping.
3. Use the following script to record a rosbag while slowly driving the vehicle around the mapped area. You do not need to drive back over areas twice, however keep loop closure in mind. Try to think of a route before starting.

		bash scripts/record_mapping_bag.bash

4. After the rosbag is collected, shutdown all the rosnodes and then only launch the following:

		roslaunch common_platform create_pcd_map.launch

5. Playback the rosbag and watch in rviz as the map is created, watch the output of the ndt_mapping node to ensure all scans are processed.
6. Leave the node running even after ndt_mapping has completed processing all the scans, and save the PCD using the following script:

		bash save_pcd_map.bash [/optional/path/to/file.pcd]

7. Move the PCD map to your desired map folder in the `aw_data` package and name it `map.pcd`

		roscd aw_data/maps/desired_map/pointcloud_map
		mv /path/to/new/map.pcd map.pcd
8. If you are mapping a new area, you may also need to add a `tf.launch` to go along with the pcd map.
The `tf.launch` file is useful for defining any additional transforms associated with the map.
See the `aw_data` package for existing examples and more details.

### ROS bag replay

If you have an existing rosbag and want to test autoware offline, below is the general procedure.

1. Filter your bag using the `scripts/filter_sensor_bag.bash` script.
This will reduce it to sensor information only.
See the README in the `scripts` folder for more details.
2. Ensure that you have the same vector or PCD map setup that was used in the bag.
3. Playback the filtered bag using `rosbag play -l --clock <file_name>` and launch `testing/bag_replay.launch`.

**Note**: `testing/bag_replay.launch` will set the ROS global parameter `use_sim_time` to `true` at the top. However, if you launch any other tools like `rviz` or `rqt` prior to this, you will need to set the parameter first in order to get proper `tf` lookups. In addition, the `--clock` parameter to `rosbag` is required for `tf` to work properly.

### Creating a map frame for the gpsins_localizer

The gpsins_localizer requires a transform from earth -> map in order to operate.
It is suggested to create this map frame somewhere close to where you will be operating.

1. Keep the vehicle stationary and verify a good GPS solution.
2. Run the following launch file to create a transform.

		roslaunch common_platform create_gps_map_frame.launch

3. Copy the example map frame launch file from [here](https://gitlab.com/astuff/autoware.ai/core_perception/blob/as/master/gpsins_localizer/launch/map_frame_example.launch) to your desired map folder and name it `tf.launch`.

4. Use the following command to echo the `/tf` topic to get the earth -> gps_measured transform.
Use ctrl-C to stop the output and inspect one of the results.

		rostopic echo /tf | grep -A 10 "child_frame_id: \"gps_measured"

5. Copy the `translation` and `rotation`fields into the `tf.launch` file you created in step 3.

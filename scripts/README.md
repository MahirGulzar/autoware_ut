# Scripts

### **filter_sensor_bag.bash**
Used for reducing a rosbag down to only the sensor data.
This script is useful for testing autoware functionality offline.

Example usage:
```
bash filter_sensor_bag.bash ~/path/to/bag.bag
```
The filtered bag will be placed in the same directory as the source rosbag with `.filtered` added to the filename.


### **record_mapping_bag.bash**
Run this script to record a rosbag of the topics required for autoware map making.
The data will be saved to the file provided by the argument.
If no argument is provided, a default location will be used.

Example usage:
```
bash record_mapping_bag.bash ~/path/to/map_bag.bag
```


### **save_pcd_map.bash**
Run this after ndt_mapping is complete to save the PCD map.
If no argument is provided, a default location will be used.

Example usage:
```
bash save_pcd_map.bash ~/path/to/pcd_map.pcd
```

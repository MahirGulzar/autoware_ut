#!/bin/bash
set -e

PCD_FILE="$1"

if [ -z "$PCD_FILE" ]; then
  FILENAME="$(date '+%Y-%m-%d_%H-%M-%S')_pcd_map.pcd"
  PCD_FILE="/tmp/$FILENAME"
  echo "No file save path provided. File will be saved to:"
  echo "$PCD_FILE"
fi

rostopic pub /config/ndt_mapping_output autoware_config_msgs/ConfigNDTMappingOutput \
"header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
filename: '$PCD_FILE'
filter_res: 0.2"

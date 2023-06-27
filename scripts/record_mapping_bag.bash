#!/bin/bash
set -e

BAG_FILE="$1"

if [ -z "$BAG_FILE" ]; then
  BAG_DIR="$HOME/aw_standard_ws/src/ut_aw_pkgs/aw_data/rosbags/Map_Generation"
  FILE_NAME="$(date '+%Y-%m-%d_%H-%M-%S')_map_bag.bag"
  BAG_FILE="$BAG_DIR/$FILE_NAME"
  echo "No file save path provided. File will be saved to:"
  echo "$BAG_FILE"
  mkdir -p "$BAG_DIR"
fi

rosbag record -O "$BAG_FILE" rosout tf points_raw vehicle/odom imu_raw

#!/bin/bash
set -e

# This script is for reducing a rosbag down to only the sensor data so that it
# can be used to test autoware functionality offline.

IN_BAG="$1"
OUT_BAG="$2"

if [ -z "$IN_BAG" ]; then
  echo "Please provide a path to a rosbag file"
  exit
fi

if [ -z "$2" ]; then
  OUT_BAG="$1.filtered"
fi

DESIRED_TOPICS=(
/camera_fl/image_raw
/camera_fl/image_raw/compressed
/gps/fix
/gps/imu
/gps/inscov
/gps/inspva
/image_raw
/image_raw/compressed
/imu_raw
/nmea_sentence
/points_raw
/tf_static
/vehicle/odom
/vehicle/twist
/clock
)

FILTER_STRING="topic == '/rosout'"

for topic in "${DESIRED_TOPICS[@]}"; do
  FILTER_STRING="$FILTER_STRING or topic == '$topic'"
done

rosbag filter "$IN_BAG" "$OUT_BAG" "$FILTER_STRING"

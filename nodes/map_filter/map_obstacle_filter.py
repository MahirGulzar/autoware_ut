#!/usr/bin/env python
# coding=utf-8

from __future__ import print_function
from __future__ import division
from geometry_msgs.msg import PoseStamped
from autoware_msgs.msg import DetectedObjectArray
from tf import TransformListener, LookupException, ExtrapolationException
import numpy as np
import rospy
import math
import cv2
import os


class MapObstacleFilter:

    def __init__(self):
        self.listener = TransformListener()

        # Placeholder for latest ego pose & transform between map & detected object's frame
        self.transform_matrix = None
        self.curr_ego_pose = None

        # Get top-left x & y map coordinates of semantic raster
        self.map_tl_x = rospy.get_param("map_tl_x", default=8926)
        self.map_tl_y = rospy.get_param("map_tl_y", default=10289)
        # Pixel per meter resolution
        self.pixel_resolution = rospy.get_param(
            "pixel_resolution", default=0.2)

        self.semantic_raster_path = rospy.get_param("~semantic_raster_path", default=None)
        self.lane_map = cv2.imread(os.path.join(self.semantic_raster_path,'drivable_area_mask_updated.png'), 0)

        # Subscribers
        rospy.Subscriber("/detection/lidar/objects",
                         DetectedObjectArray, self.callback_detections)
        rospy.Subscriber("current_pose", PoseStamped, self.curr_pose_callback)

        # Filtered obstacles publisher
        self.filtered_objects_pub = rospy.Publisher(
            "/detection/lidar/objects_filtered", DetectedObjectArray, queue_size=1)

    def callback_detections(self, msg):
        """ Callback to get detected objects

        Parameters
        ----------
        msg : DetectedObjectArray
        """

        if len(msg.objects) == 0:
            return

        if self.curr_ego_pose is None:
            rospy.logwarn("No ego car pose detected")
            return

        if "map" not in msg.header.frame_id:
            try:
                # Transform matrix to map frame
                self.transform_matrix = self.listener.asMatrix(
                    "map", msg.header)
            except (LookupException, ExtrapolationException) as e:
                rospy.logwarn("Transform not found " + str(e))

        filter_mask = []
        for detection_object in msg.objects:

            curr_obs = np.array([[detection_object.pose.position.x,
                                detection_object.pose.position.y, detection_object.pose.position.z]])

            if "map" not in msg.header.frame_id:
                # Transform detected object centroids to map frame using last known transform
                curr_obs = np.concatenate(
                    [curr_obs, np.ones((curr_obs.shape[0], 1))], axis=1)
                curr_obs = np.dot(
                    curr_obs, self.transform_matrix.T)[:, :3]

            # Get corresponding pixel index value of the obstacle centroid in map frame
            img_idx = [
                int(math.ceil(
                    abs(self.map_tl_x - curr_obs[0][0]) / self.pixel_resolution)),
                int(math.ceil(
                    abs(self.map_tl_y - curr_obs[0][1]) / self.pixel_resolution))
            ]

            if img_idx[1] >= self.lane_map.shape[0] or img_idx[0] >= self.lane_map.shape[1]:
                # rospy.logwarn("Skipping obstacle at position img_idx_x=%.2f, img_idx_y=%.2f because its outside the filter bounds", img_idx[0], img_idx[1])
                filter_mask.append(False)
                continue

            # Filter out obstacles w.r.t semantic raster mask where pixel value 1 means drivable area
            if self.lane_map[img_idx[1], img_idx[0]] == 0:
                filter_mask.append(False)
            else:
                filter_mask.append(True)

        msg.objects = np.array(msg.objects)[filter_mask].tolist()
        self.filtered_objects_pub.publish(msg)

    def curr_pose_callback(self, msg):
        """ Callback to get current pose of ego vehicle

        Parameters
        ----------
        msg : PoseStamped
        """
        self.curr_ego_pose = msg

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('map_obstacle_filter',
                    anonymous=False,
                    log_level=rospy.INFO)
    node = MapObstacleFilter()
    node.run()

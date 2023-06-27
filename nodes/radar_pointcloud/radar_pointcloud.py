#!/usr/bin/env python2

from __future__ import print_function
from __future__ import division
import rospy
import numpy as np

from std_msgs.msg import Header
from derived_object_msgs.msg import ObjectWithCovarianceArray
from derived_object_msgs.msg import ObjectWithCovariance
from radar_msgs.msg import RadarTrackArray
from radar_msgs.msg import RadarTrack
from autoware_msgs.msg import DetectedObjectArray
from autoware_msgs.msg import DetectedObject
from autoware_msgs.msg import CloudCluster
from autoware_msgs.msg import CloudClusterArray
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField

from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3

import time

import message_filters
import itertools
import tf
import struct


class radar_pointcloud:
    target_frame = None
    pointcloud2_pub = None
    tf_listener = None
    cloud_size = None  # Number of points in one dimension
    cloud_dimension = None  # Size in meter of one side of the cube

    def __init__(self):
        rospy.loginfo(self.__class__.__name__ + " - node started")

        # Parameters
        objects_input_topic = rospy.get_param("~objects_input_topic", 'radar_fc/as_tx/objects')
        pointcloud2_output_topic = rospy.get_param("~pointcloud2_output_topic", "radar_fc/points_raw")
        self.target_frame = rospy.get_param("~target_frame", "lidar")
        self.cloud_size = rospy.get_param("~cloud_size", 3)
        self.cloud_dimension = 2/rospy.get_param("~cloud_dimension", .2)

        # Subscribers and tf listeners
        rospy.Subscriber(objects_input_topic, ObjectWithCovarianceArray, self.process_object)
        self.tf_listener = tf.TransformListener()

        # Publishers
        self.pointcloud2_pub = rospy.Publisher(pointcloud2_output_topic, PointCloud2, queue_size=10)

    def process_object(self, objects):
        source_frame = objects.header.frame_id

        clusters = []
        cloud = self.generate_cloud_from_objects(objects.objects, source_frame)
        cloud.header.stamp = objects.header.stamp
        self.pointcloud2_pub.publish(cloud)

    def generate_cluster(self, cloud, centroid, id):
        cluster = CloudCluster(
            header=Header(frame_id='lidar', stamp=rospy.Time.now())
        )

        cluster.centroid_point = PointStamped(
            point=centroid,
            header=Header(frame_id=self.target_frame, stamp=rospy.Time.now())
        )

        cluster.cloud = cloud
        cluster.dimensions = Vector3(self.cloud_dimension, self.cloud_dimension, self.cloud_dimension)
        cluster.id = id
        return cluster


    def get_tfed_pose_from_pose_with_cov(self, pose_with_cov, source_frame):
        """
        :type pose_with_cov: PoseWithCovariance
        :type source_frame: str
        :returns: Pose tfed to the target_frame
        """

        # To apply a TF we need a pose stamped
        pose_stamped = PoseStamped(pose=pose_with_cov.pose)
        pose_stamped.header.frame_id = source_frame
        tfed_pose = None
        while tfed_pose is None :
            try:
                tfed_pose = self.tf_listener.transformPose(self.target_frame, pose_stamped).pose
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn(self.__class__.__name__ + " - Could not get TF")
                continue
        return tfed_pose

    def generate_cloud(self, pose):
        """
        :type pose: Pose
        :returns: pc2
        """
        points = []
        i = 0
        for x, y, z in itertools.product(*map(xrange, ([self.cloud_size] * self.cloud_size))):
            points.append(
                [
                    pose.position.x + x / self.cloud_dimension,
                    pose.position.y + y / self.cloud_dimension,
                    pose.position.z + z / self.cloud_dimension
                ]
            )

        return points

    def generate_cloud_from_objects(self, objects, source_frame):
        """
        :type objects: ObjectWithCovariance
        :type source_frame: str
        :returns: pc2
        """

        points = []
        for obj in objects:
            pose_target_frame = self.get_tfed_pose_from_pose_with_cov(obj.pose, source_frame)
            points = points + (self.generate_cloud(pose_target_frame))

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]

        header = Header(frame_id=self.target_frame, stamp=rospy.Time.now())
        return point_cloud2.create_cloud(header, fields, points)

    @staticmethod
    def run():
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('radar_clusters', anonymous=True, log_level=rospy.INFO)
    node = radar_pointcloud()
    node.run()

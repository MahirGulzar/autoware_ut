#!/usr/bin/env python
# coding=utf-8

import rospy
import readchar
from threading import Thread
from geometry_msgs.msg import PointStamped
from autoware_msgs.msg import DetectedObject, DetectedObjectArray
from visualization_msgs.msg import Marker


class ObstacleGenerator:
    def __init__(self):

        # Run separate thread for controlling point obstacle using keyboard
        thread = Thread(target=self.control_point_obs, args=())
        thread.start()

        # Bool to check if simulation mode is enabled
        self.enable_simulation_mode = rospy.get_param(
            "~enableSimulationMode", default=False)

        # Subscribers
        rospy.Subscriber("/clicked_point",
                         PointStamped, self.clicked_point_callback)

        # If simulation mode is enabled then subscribe to simulated objects and append point obstacle in DetectedObjectArray
        if self.enable_simulation_mode:
            rospy.Subscriber("/detection/contour_tracker/objects",
                             DetectedObjectArray, self.detected_objects_callback)

        # Publishers
        self.point_marker_pub = rospy.Publisher(
            "/point_marker", Marker, queue_size=1)

        self.updated_detections = rospy.Publisher(
            "/detection/objects", DetectedObjectArray, queue_size=1)

        self.point_obstacle = DetectedObject()

    def detected_objects_callback(self, msg):
        """ Callback to recieve simulated tracked vehicle obstacles

        Parameters
        ----------
        msg : DetectedObjectArray
            Array of all simulated tracked objects
        """

        self.point_obstacle.id = msg.objects[-1].id + 1
        self.point_obstacle.pose.position.z = msg.objects[0].pose.position.z
        msg.objects.append(self.point_obstacle)

        self.point_marker_pub.publish(self.get_curr_marker(
            msg.header.stamp, msg.header.frame_id))
        self.updated_detections.publish(msg)

    def clicked_point_callback(self, msg):
        """ Callback to get latest published point in rviz

        Parameters
        ----------
        msg : PointStamped
            Location of published point in rviz 
        """

        self.point_obstacle.pose.position = msg.point

    def get_curr_marker(self, stamp, frame_id):
        """ Creates marker message for visualizing static obstacle in rviz

        Parameters
        ----------
        stamp : rospy.Time
            Time stamp on which the marker should be visualized
        frame_id : str
            Frame ID of the marker


        Returns
        -------
        Marker
            A sphere marker created over the latest published point location
        """

        marker = Marker()
        marker.type = marker.SPHERE
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.scale.x = 2
        marker.scale.y = 2
        marker.scale.z = 2
        marker.color.r = 0.8
        marker.color.g = 0
        marker.color.b = 1
        marker.color.a = 1
        marker.pose = self.point_obstacle.pose

        return marker

    def publish_point_obs_loop(self):
        """  
        Publishes point obstacle if simulation mode is not enabled 
        """

        if not self.enable_simulation_mode:
            while not rospy.is_shutdown():
                msg = DetectedObjectArray()
                msg.header.frame_id = "map"
                msg.header.stamp = rospy.Time.now()
                self.point_obstacle.id = 1
                self.point_obstacle.pose.position.z = 34.0
                msg.objects.append(self.point_obstacle)

                self.point_marker_pub.publish(
                    self.get_curr_marker(rospy.Time.now(), "map"))

                self.updated_detections.publish(msg)
                rospy.Rate(10).sleep()

    def control_point_obs(self):
        """  
        Control current point obstacle using keyboard
        """

        while not rospy.is_shutdown():
            key = repr(readchar.readchar())

            if key == repr('\x03'):
                rospy.loginfo("Shutting down ...")
                rospy.signal_shutdown("")
                exit(0)

            if key == repr('8'):
                self.point_obstacle.pose.position.y += 0.5
            elif key == repr('5'):
                self.point_obstacle.pose.position.y -= 0.5
            elif key == repr('4'):
                self.point_obstacle.pose.position.x -= 0.5
            elif key == repr('6'):
                self.point_obstacle.pose.position.x += 0.5

    def run(self):
        self.publish_point_obs_loop()
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('obstacle_generator',
                    anonymous=False, log_level=rospy.INFO)
    node = ObstacleGenerator()
    node.run()

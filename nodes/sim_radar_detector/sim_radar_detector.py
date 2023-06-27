#!/usr/bin/python

from __future__ import print_function

import rospy
import signal
import tf
import tf2_py
import tf_conversions
from autoware_msgs.msg import DetectedObjectArray, DetectedObject
from lgsvl_msgs.msg import DetectedRadarObjectArray, DetectedRadarObject
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3, Quaternion, Polygon, Twist, Vector3Stamped
from genpy import Duration
from std_msgs.msg import ColorRGBA, Header
from numpy import cross, cos, sin, sqrt, linalg

import radar_tf


def terminate_node(signum=None, frame=None, exit_code=0):
    rospy.loginfo("Shutting down ...")
    rospy.signal_shutdown("")
    exit(exit_code)


class sim_radar_detector:
    object_pub = None
    marker_pub = None
    tf_listener = None

    map_frame = "map"
    radar_frame = "radar_fc"

    def __init__(self):
        rospy.Subscriber('/radar_fc', DetectedRadarObjectArray, self.radar_obj_array_cb)
        self.object_pub = rospy.Publisher('/detection/objects', DetectedObjectArray, queue_size=10)
        self.marker_pub = rospy.Publisher('/detection/lidar/objects_markers', MarkerArray, queue_size=10)

        # TF
        self.tf_listener = tf.TransformListener()
        while not rospy.is_shutdown():
            try:
                rospy.loginfo_once(self.__class__.__name__ + " - Trying to get map->radar TF ...")
                self.tf_listener.lookupTransform(self.map_frame, self.radar_frame, rospy.Time(0))
                rospy.loginfo(self.__class__.__name__ + " - TF Acquired !")
                break
            except tf2_py.TransformException:
                rospy.loginfo_once(self.__class__.__name__ + " - Unable to get TF, waiting ...")
                rospy.sleep(1)

    rospy.loginfo("Starting node.")

    def radar_obj_array_cb(self, radar_objects):
        """
        :type radar_objects: DetectedRadarObjectArray
        :return:
        """
        markers = MarkerArray()
        detected_objects = DetectedObjectArray()

        detected_objects.header.stamp = rospy.Time.now()
        detected_objects.header.frame_id = self.map_frame

        i = 0
        for radar_obj in radar_objects.objects:  # type: DetectedRadarObject
            # markers.markers.append(self.generate_marker(i, radar_obj))
            markers.markers += self.generate_marker(i, radar_obj)
            detected_objects.objects.append(self.generate_object(i, radar_obj))
            i += 1

        self.marker_pub.publish(markers)
        # self.object_pub.publish(detected_objects)

    def generate_object(self, object_id, radar_obj):
        detected_object = DetectedObject()
        object_point = self.get_radar_object_point(radar_obj)

        detected_object.header.frame_id = self.map_frame
        detected_object.id = object_id + 1  # We make sure the id is never 0 because OP would ignore it.
        detected_object.pose.position = object_point
        detected_object.pose.orientation = Quaternion(w=1., x=0., y=0., z=0.)
        # detected_object.velocity = Twist(linear=prepared_track['vel'], angular=Vector3(x=0, y=0, z=0))
        # detected_object.velocity_reliable = True
        # detected_object.acceleration = Twist(linear=prepared_track['accel'], angular=Vector3(x=0, y=0, z=0))
        # detected_object.acceleration_reliable = True
        # detected_object.dimensions = prepared_track['dimensions']
        # detected_object.convex_hull.polygon = Polygon(points=prepared_track['points'])

        # attempt 1 with cos
        # transform = self.tf_listener.lookupTransform(self.map_frame, self.radar_frame, rospy.Time(0))
        # euler = tf_conversions.transformations.euler_from_quaternion(transform[1])
        # print("(%.3f, %.3f)" % (cos(euler[2]), sin(euler[2])))

        # attempt 2 with tf
        # v3 = self.tf_listener.transformVector3(
        #     self.radar_frame,
        #     Vector3Stamped(header=Header(frame_id=self.map_frame),
        #                    # vector=radar_obj.object_relative_velocity,
        #                    vector=Vector3(
        #                        x=radar_obj.object_relative_velocity.x,
        #                        y=radar_obj.object_relative_velocity.y,
        #                        z=radar_obj.object_relative_velocity.z,
        #                    ),
        #                    )
        # ).vector
        # print(radar_obj.object_relative_velocity, "\n-----")

        return detected_object

    def generate_marker(self, marker_id, radar_obj):
        marker = Marker()
        object_point = self.get_radar_object_point(radar_obj)
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.map_frame
        marker.id = marker_id
        marker.pose.position = object_point
        marker.lifetime = Duration(secs=1.)
        marker.type = 2  # SPHERE
        marker.action = 0  # ADD
        marker.scale = Vector3(x=1., y=1., z=1.)
        marker.ns = "simulated radar"
        marker.color = ColorRGBA(r=1., g=0.08, b=0.58, a=1.)

        marker_text = Marker()
        object_point = self.get_radar_object_point(radar_obj)
        marker_text.header.stamp = rospy.Time.now()
        marker_text.header.frame_id = self.map_frame
        marker_text.id = marker_id
        marker_text.pose.position = Point(x=object_point.x+1, y=object_point.y+1, z=object_point.z)
        marker_text.lifetime = Duration(secs=1.)
        marker_text.type = 9  # TEXT
        marker_text.action = 0  # ADD
        marker_text.scale = Vector3(x=1., y=1., z=1.)
        marker_text.ns = "simulated radar vel"
        marker_text.color = ColorRGBA(r=1., g=0.08, b=0.58, a=1.)
        marker_text.text = "%.2f" % radar_obj.sensor_velocity.y

        print("relative_velocity: [%d] %.2f" % (marker_id, linalg.norm([radar_obj.object_relative_velocity.x, radar_obj.object_relative_velocity.y, radar_obj.object_relative_velocity.z])))
        print("velocity:          [%d] %.2f" % (marker_id, linalg.norm([radar_obj.object_velocity.x, radar_obj.object_velocity.y, radar_obj.object_velocity.z])))
        print("----")

        return [marker, marker_text]

    def get_radar_object_point(self, radar_obj):
        tfed_point = Point()
        tfed_point = radar_tf.tf_point(Point(), self.tf_listener, self.map_frame,
                                       self.radar_frame)
        return Point(
            x=tfed_point.x + radar_obj.object_relative_position.x,
            y=tfed_point.y + radar_obj.object_relative_position.y,
            z=tfed_point.z + radar_obj.object_relative_position.z
        )

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    signal.signal(signal.SIGINT, terminate_node)
    rospy.init_node('sim_radar_detector', anonymous=True, log_level=rospy.INFO)
    node = sim_radar_detector()
    node.run()

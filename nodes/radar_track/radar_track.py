#!/usr/bin/env python2

from __future__ import division
from __future__ import print_function

import copy
import itertools

import message_filters
import rospy
import tf
import tf2_py
from autoware_msgs.msg import DetectedObjectArray, DetectedObject
from derived_object_msgs.msg import ObjectWithCovarianceArray
from genpy import Duration
from geometry_msgs.msg import Vector3, Polygon, Twist, Quaternion, TwistStamped
from radar_msgs.msg import RadarTrackArray
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from delphi_esr_msgs.msg import  EsrVehicle2

import radar_filter
import radar_tf


class radar_track:
    def __init__(self):
        # Parameters
        object_topic = rospy.get_param("~object_topic", '/radar_fc/as_tx/objects')
        tracks_topic = rospy.get_param("~tracks_topic", '/radar_fc/as_tx/radar_tracks')
        twist_topic = rospy.get_param("~twist_topic", '/vehicle/twist')
        tracked_object_topic = rospy.get_param("~tracked_object_topic", 'tracked_objects_radar')
        viz_topic = rospy.get_param("~viz_topic", 'detected_polygons_radar')
        esr_vehicle2_topic = "/radar_fc/parsed_rx/vehicle2_msgs"

        self. id_count = {} # dictionary that keeps track of radar objects and their id count. Used for checking consistency of object ids in consistency filter
        self.radar_frame = rospy.get_param("~radar_frame", 'radar_fc')
        self.map_frame = rospy.get_param("~map_frame", 'map')
        self.max_detection_distance = rospy.get_param("~/detection/radar_track/max_detection_distance", 60.0)  # max distance to include radar detection
        self.speed_deadzone = rospy.get_param("~/detection/radar_track/speed_deadzone", 0.5)  # Deadzone for the speed measurement.
        self.enable_raw_radar = rospy.get_param("~/detection/radar/radar_track/enable_raw_radar", True)

        assert type(object_topic) == str
        assert type(tracks_topic) == str
        assert type(twist_topic) == str
        assert type(self.radar_frame) == str
        assert type(self.map_frame) == str
        assert type(self.max_detection_distance) == float and self.max_detection_distance >= 0.0
        assert type(self.speed_deadzone) == float

        rospy.loginfo(self.__class__.__name__ + " - object_topic : " + str(object_topic))
        rospy.loginfo(self.__class__.__name__ + " - tracks_topic : " + str(tracks_topic))
        rospy.loginfo(self.__class__.__name__ + " - twist_topic : " + str(twist_topic))
        rospy.loginfo(self.__class__.__name__ + " - radar_frame : " + str(self.radar_frame))
        rospy.loginfo(self.__class__.__name__ + " - map_frame : " + str(self.map_frame))
        rospy.loginfo(self.__class__.__name__ + " - speed_deadzone : " + str(self.speed_deadzone))
        rospy.loginfo(self.__class__.__name__ + " - max_detection_distance : " + str(self.max_detection_distance))

        # Publishers
        self.marker_array_pub = rospy.Publisher(viz_topic, MarkerArray, queue_size=1)
        self.detected_object_array_pub = rospy.Publisher(tracked_object_topic, DetectedObjectArray, queue_size=1)
        self.esr_vehicle2_pub = rospy.Publisher(esr_vehicle2_topic, EsrVehicle2, queue_size=10, latch=True)

        # Enabling raw radar detections. To be set in the launch file being launched
        if self.enable_raw_radar:
            self.activate_raw_radar()

        # Subscribers and tf listeners
        self.tf_listener = tf.TransformListener()

        object_sub = message_filters.Subscriber(object_topic, ObjectWithCovarianceArray)
        tracks_sub = message_filters.Subscriber(tracks_topic, RadarTrackArray)
        twist_sub = message_filters.Subscriber(twist_topic, TwistStamped)

        while not rospy.is_shutdown():
            try:
                rospy.loginfo_once(self.__class__.__name__ + " - Trying to get map->radar TF ...")
                self.tf_listener.lookupTransform(self.map_frame, self.radar_frame, rospy.Time(0))
                rospy.loginfo(self.__class__.__name__ + " - TF Acquired !")
                break
            except tf2_py.TransformException:
                rospy.loginfo_once(self.__class__.__name__ + " - Unable to get TF, waiting ...")
                rospy.sleep(1)

        # Sync
        ts = message_filters.ApproximateTimeSynchronizer(
            [object_sub, tracks_sub, twist_sub],
            queue_size=5, slop=0.01)
        ts.registerCallback(self.radar_data_callback)

    def radar_data_callback(self, objects, tracks, twist):
        """
        :type objects: ObjectWithCovarianceArray
        :type tracks: RadarTrackArray
        :type twist: TwistStamped
        """
        rospy.loginfo_once(self.__class__.__name__ + " - Receiving synced radar data")

        prepared_tracks, prepared_objects = self.prepare_radar_data(objects, tracks, twist)

        assert len(prepared_tracks) == len(prepared_objects)  # Tracks and Objects should match

        detected_objects, markers = self.generate_messages_to_publish(prepared_tracks, prepared_objects)
        detected_objects.header.stamp = objects.header.stamp

        self.detected_object_array_pub.publish(detected_objects)
        self.marker_array_pub.publish(markers)

    def prepare_radar_data(self, objects, tracks, twist):
        """
        :type objects: ObjectWithCovarianceArray
        :type tracks: RadarTrackArray
        :type twist: TwistStamped
        :returns Tuple of lists. Lists contains dicts of tracks and objects
        """
        prepared_tracks = []
        prepared_objects = []

        id_list = [track.track_id for track in tracks.tracks]
        radar_filter.remove_old_tracks(id_list, self.id_count)

        for radar_trk, radar_ob in itertools.izip(tracks.tracks, objects.objects):
            prepared_object = self.prepare_object(radar_ob)

            # Filtering based on distance from radar
            if not radar_filter.is_within_range(prepared_object, self.tf_listener, self.map_frame, self.radar_frame,
                                                self.max_detection_distance):
                continue

            # Filtering based consistency of object
            if not radar_filter.is_consistent(prepared_object, self.id_count):
                continue

            prepared_tracks.append(self.prepare_track(radar_trk, twist))
            prepared_objects.append(prepared_object)
        return prepared_tracks, prepared_objects

    def prepare_track(self, track, twist):
        """
        Extracts relevant data and tf it to self.map_frame
        :param track: RadarTrack
        :param twist: TwistStamped
        :type track: RadarTrack
        :returns dict
            WHERE
             - int track_id Represents the ID of the tracked object
             - Vector3 accel Acceleration of the tracked object (m.s-2)
             - Vector3 vel Velocity of the tracked object (m.s-1)
             - Points32[] points List of Points32 representing the polygon of the object
        """
        points = [radar_tf.tf_point32(point, self.tf_listener, self.map_frame, self.radar_frame) for point in
                  track.track_shape.points]

        # Computing speed relative to map.
        actual_vel_x = track.linear_velocity.x + twist.twist.linear.x
        actual_vel_y = track.linear_velocity.y + twist.twist.linear.y
        actual_vel_z = track.linear_velocity.z + twist.twist.linear.z

        # Applying deadzone
        vel = Vector3()
        vel.x = actual_vel_x if abs(actual_vel_x) > self.speed_deadzone else 0.01  # Not zero not to trigger STOP_HACK
        vel.y = actual_vel_y if abs(actual_vel_y) > self.speed_deadzone else 0.01
        vel.z = actual_vel_z if abs(actual_vel_z) > self.speed_deadzone else 0.01

        return {
            "track_id": track.track_id,
            "accel": radar_tf.tf_vector3(track.linear_acceleration, self.tf_listener, self.map_frame, self.radar_frame),
            "vel": radar_tf.tf_vector3(vel, self.tf_listener, self.map_frame, self.radar_frame),
            "points": points,
            "dimensions": self.get_surrounding_cuboid_size(points)
        }
    def prepare_object(self, radar_object):
        """
        Extracts relevant data and tf it to self.map_frame
        :param radar_object: ObjectWithCovariance
        :type radar_object: ObjectWithCovariance
        :returns dict
            WHERE
             - Point position Represents the position of the centroid of the tracked object.
        """
        return {
            "position": radar_tf.tf_point(radar_object.pose.pose.position, self.tf_listener, self.map_frame,
                                          self.radar_frame),
            "id": radar_object.id,
            "timestamp": radar_object.header.stamp
        }

    def generate_messages_to_publish(self, prepared_tracks, prepared_objects):
        """
        Takes prepared_tracks prepared_objects lists. Iterate over them and creates a detected object
        :type prepared_tracks: list
        :type prepared_objects: list
        :returns DetectedObjectArray, MarkerArray
        """
        markers = MarkerArray()
        detected_objects = DetectedObjectArray()
        detected_objects.header.frame_id = self.map_frame
        for prepared_track, prepared_object in itertools.izip(prepared_tracks, prepared_objects):
            if prepared_track['track_id'] != prepared_object['id']: # Never experienced this happening but keeping it for now as a caution
                rospy.logerr(self.__class__.__name__ + " - RADAR OUT OF SYNC, DROPPING !")
                continue

            detected_objects.objects.append(self.generate_detected_object(prepared_track, prepared_object))
            markers.markers += self.generate_rviz_marker(prepared_track, prepared_object)

        return detected_objects, markers

    def generate_detected_object(self, prepared_track, prepared_object):
        """
        Generate a DetectedObject to be used by OP.
        Some DetectedObject can be filtered out.
        :type prepared_track: dict
        :type prepared_object: dict
        :returns DetectedObject
        """

        # type: DetectedObject
        detected_object = DetectedObject()
        detected_object.header.frame_id = self.map_frame
        detected_object.id = prepared_track['track_id']
        detected_object.pose.position = prepared_object['position']
        detected_object.pose.orientation = Quaternion(w=1., x=0., y=0., z=0.)
        detected_object.velocity = Twist(linear=prepared_track['vel'], angular=Vector3(x=0, y=0, z=0))
        detected_object.velocity_reliable = True
        detected_object.acceleration = Twist(linear=prepared_track['accel'], angular=Vector3(x=0, y=0, z=0))
        detected_object.acceleration_reliable = True
        detected_object.valid = True
        detected_object.label = 'unknown'
        detected_object.dimensions = prepared_track['dimensions']
        detected_object.convex_hull.polygon = Polygon(points=prepared_track['points'])
        return detected_object



    def generate_rviz_marker(self, prepared_track, prepared_object):
        """
        We generate several marker per object so we need to return a list of Marker.
        XX : centroid
        1XX : Polygon
        10XX : text
        :type prepared_track: dict
        :type prepared_object: dict
        :returns Marker[]
        """
        marker_list = []

        centroid = Marker()
        centroid.header.frame_id = self.map_frame
        centroid.ns = "radar_centroid"
        centroid.type = centroid.SPHERE
        centroid.action = centroid.ADD
        centroid.scale = Vector3(x=1., y=1., z=1.)
        centroid.color = ColorRGBA(r=1., g=0.08, b=0.58, a=1.)
        centroid.pose.position = prepared_object['position']
        centroid.pose.orientation = Quaternion(w=1., x=0., y=0., z=0.)
        centroid.id = prepared_object['id']
        centroid.lifetime = Duration(secs=0.1)

        text = Marker()
        text.header.frame_id = self.map_frame
        text.ns = "radar_text"
        text.type = text.TEXT_VIEW_FACING
        text.action = text.ADD
        text.scale = Vector3(x=0., y=0., z=1.)
        text.color = ColorRGBA(r=1., g=1., b=1., a=1.)
        text.pose.position = copy.copy(prepared_object['position'])
        text.pose.position.x += 2.5
        text.pose.orientation = Quaternion(w=1., x=0., y=0., z=0.)
        text.text = "#%02d_(%05.2f)" % (prepared_object['id'], prepared_track['vel'].x)  # Id & speed
        text.id = prepared_object['id'] + 1000
        text.lifetime = Duration(secs=0.1)

        marker_list += [centroid, text]
        return marker_list

    def activate_raw_radar(self):

        # Sending boolean True to raw_data_enabled field of the EsrVehicle2
        # message to turn off tracking and instead receive raw radar detections
        esrvehicle2_msg = EsrVehicle2()
        esrvehicle2_msg.raw_data_enabled = True
        self.esr_vehicle2_pub.publish(esrvehicle2_msg)

    @staticmethod
    def get_surrounding_cuboid_size(points):
        """
        Computes the dimensions of the smallest volume rectangular cuboid surrounding points
        :param points: Point32[]
        :type points: Point32[]
        :returns Vector3
        """
        dims = ['x', 'y', 'z']
        coords_max = [0., 0., 0.]
        coords_min = [float('inf'), float('inf'), float('inf')]

        for point in points:
            for i in range(len(dims)):
                if coords_max[i] < getattr(point, dims[i]):
                    coords_max[i] = getattr(point, dims[i])
                if coords_min[i] > getattr(point, dims[i]):
                    coords_min[i] = getattr(point, dims[i])
        dimensions = ([ma - mi for ma, mi in itertools.izip(coords_max, coords_min)])
        return Vector3(x=dimensions[0], y=dimensions[1], z=dimensions[2])

    @staticmethod
    def run():
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('radar_track', anonymous=True, log_level=rospy.INFO)
    node = radar_track()
    node.run()

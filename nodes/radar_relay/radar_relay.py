#!/usr/bin/env python

from __future__ import print_function
import rospy

from geometry_msgs.msg import Vector3, Vector3Stamped, Twist, TwistStamped, PoseWithCovariance, PoseStamped, Polygon, Point32, Quaternion, Point, PointStamped
from derived_object_msgs.msg import ObjectWithCovariance, ObjectWithCovarianceArray
from autoware_msgs.msg import DetectedObject, DetectedObjectArray
from radar_msgs.msg import RadarTrack, RadarTrackArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from genpy import Duration
import message_filters
import numpy as np
import itertools
import tf
import copy

class radar_relay:

    def __init__(self):
        rospy.loginfo(self.__class__.__name__ + " - node started")

        # Parameters
        self.target_frame = rospy.get_param("~target_frame", "map")
        objects_input_topic = rospy.get_param("~objects_input_topic", '/radar_fc/as_tx/objects')
        radar_tracks_input_topic = rospy.get_param("~radar_tracks_input_topic", '/radar_fc/as_tx/radar_tracks')
        ego_speed_topic = rospy.get_param("~twist_topic", '/current_velocity')
        detected_objects_topic = rospy.get_param("~detected_objects_output_topic", "objects")
        visualisation_topic = rospy.get_param("~visualisation_output_topic", "detected_polygons")
        self.consistency_check = rospy.get_param("~consistency_check", 5) # number of frames a radar detection is received before it is considered  true radar detection. Based on ID count
        self.id_count = {} # dictionary that keeps track of radar objects and their id count. Used for checking consistency of object ids in consistency filter


        # Subscribers and tf listeners
        objects_sub = message_filters.Subscriber(objects_input_topic, ObjectWithCovarianceArray)
        tracks_sub = message_filters.Subscriber(radar_tracks_input_topic, RadarTrackArray)
        ego_speed_sub = message_filters.Subscriber(ego_speed_topic, TwistStamped)

        self.tf_listener = tf.TransformListener()

        # Publishers
        self.detected_objs_pub = rospy.Publisher(detected_objects_topic, DetectedObjectArray, queue_size=1)
        self.markers_pub = rospy.Publisher(visualisation_topic, MarkerArray, queue_size=1)

        # Strict Time Sync
        ts = message_filters.ApproximateTimeSynchronizer([objects_sub, tracks_sub, ego_speed_sub], queue_size=5, slop=0.01)
        ts.registerCallback(self.syncronised_callback)

        rospy.loginfo(self.__class__.__name__ + " - Target Frame : " + self.target_frame)
        rospy.loginfo(self.__class__.__name__ + " - Objects Input Topic : " + objects_input_topic)
        rospy.loginfo(self.__class__.__name__ + " - Radar Tracks Input Topic : " + radar_tracks_input_topic)

        rospy.loginfo(self.__class__.__name__ + " - Detected Objs Topic : " + detected_objects_topic)
        rospy.loginfo(self.__class__.__name__ + " - Visualisation Topic : " + visualisation_topic)

    def syncronised_callback(self, objects, tracks, ego_speed):
        """
        :type objects: ObjectWithCovarianceArray
        :type tracks: RadarTrackArray
        """
        # Skips if the number of objects and tracks are different
        if len(objects.objects) != len(tracks.tracks):
            rospy.logwarn(self.__class__.__name__ + " - Radar objects and tracks not in sync, skipping...")
            return

        detected_objs, markers = self.generate_detected_objects_array_and_markers(objects, tracks, ego_speed)

        # Checks is there is something to publish before publishing
        if detected_objs is not None and markers is not None:
            self.detected_objs_pub.publish(detected_objs)
            self.markers_pub.publish(markers)

    def is_consistent(self, track_id):
        if track_id not in self.id_count.keys():
            self.id_count[track_id] = 1
            return False

        self.id_count[track_id] += 1
        return self.id_count[track_id] > self.consistency_check

    def remove_old_tracks(self, id_list):
        lost_tracks = self.id_count.viewkeys() - id_list
        self.remove_entries_from_dict(list(lost_tracks))

    def remove_entries_from_dict(self, entries):
        for key in entries:
            if key in self.id_count:
                del self.id_count[key]

    def generate_detected_objects_array_and_markers(self, objects, tracks, ego_speed):
        """
        :param objects: Array of ObjectWithCovariance
        :param tracks: Array of RadarTrack
        :param source_frame: Source frame of the messages, here it's the radar
        :param ego_speed: speed of the ego vehicle
        :type objects: ObjectWithCovarianceArray
        :type tracks: RadarTrackArray
        :type source_frame: str
        :returns: DetectedObjectArray and MarkerArray tfed to the target_frame class variable
        """

        # palceholders for markers and detected objects
        markers = MarkerArray()
        detected_objects_array = DetectedObjectArray()
        detected_objects_array.header.frame_id = self.target_frame
        detected_objects_array.header.stamp = tracks.header.stamp

        # frame_id of recieved objects
        source_frame = objects.header.frame_id

        # removing tracks that have been lost (ids not detected anymore)
        id_list = [track.track_id for track in tracks.tracks]
        self.remove_old_tracks(id_list)

        for obj, track in itertools.izip(objects.objects, tracks.tracks):  # type: ObjectWithCovariance, RadarTrack
            # If the ids of objects and tracks does not match we are out of sync so we return nothing
            if track.track_id != obj.id:
                rospy.logwarn(self.__class__.__name__ + " - Radar objects and tracks not in sync, skipping...")
                return None, None

            ## Check if the radar id is consstent over a of frames. Dictated by the param named consistency_check
            if not self.is_consistent(track.track_id):
                continue

            points = [self.get_tfed_point32(point, source_frame) for point in track.track_shape.points]

            # Detected object
            detected_object = DetectedObject()
            detected_object.header.frame_id = self.target_frame
            detected_object.header.stamp = tracks.header.stamp
            detected_object.id = obj.id
            detected_object.label = 'unknown'
            detected_object.valid = True
            detected_object.pose_reliable = True
            detected_object.pose = self.get_tfed_pose_from_pose_with_cov(obj.pose, source_frame)
            detected_object.velocity_reliable = True
            detected_object.velocity = self.get_vel_vector_in_map(track, ego_speed, source_frame)
            detected_object.acceleration_reliable = True
            detected_object.acceleration = Twist(linear=self.get_tfed_vector3_from_vector3(track.linear_acceleration, source_frame))
            detected_object.dimensions = self.get_surrounding_cuboid_size(points)
            detected_object.convex_hull.polygon = Polygon(points=points)

            # Marker for rviz visualisation
            markers.markers += self.generate_markers(detected_object)
            detected_objects_array.objects.append(detected_object)

        return detected_objects_array, markers

    def generate_markers(self, detected_object):

        marker_list = []

        centroid = Marker()
        centroid.header.frame_id = self.target_frame
        centroid.ns = "radar_centroid"
        centroid.type = centroid.SPHERE
        centroid.action = centroid.ADD
        centroid.scale = Vector3(x=1., y=1., z=1.)
        centroid.color = ColorRGBA(r=1., g=0.08, b=0.58, a=1.)
        centroid.pose.position = detected_object.pose.position
        centroid.pose.orientation = Quaternion(w=1., x=0., y=0., z=0.)
        centroid.id = detected_object.id
        centroid.lifetime = Duration(secs=0.1)

        text = Marker()
        text.header.frame_id = self.target_frame
        text.ns = "radar_text"
        text.type = text.TEXT_VIEW_FACING
        text.action = text.ADD
        text.scale = Vector3(x=0., y=0., z=1.)
        text.color = ColorRGBA(r=1., g=1., b=1., a=1.)
        text.pose.position = copy.copy(detected_object.pose.position)
        text.pose.position.x += 2.5
        text.pose.orientation = Quaternion(w=1., x=0., y=0., z=0.)
        # printing the norm of a radar object's velocity in the label to ensure it matches with the values published by fusion node. Will come in handy when debugging
        text.text = "#%02d_(%05.2f)" % (detected_object.id,
                                        np.linalg.norm((detected_object.velocity.linear.x, detected_object.velocity.linear.y, detected_object.velocity.linear.z)))  # Id & speed
        text.id = detected_object.id + 1000
        text.lifetime = Duration(secs=0.1)

        marker_list += [centroid, text]
        return marker_list

    def get_vel_vector_in_map(self, track, ego_speed, source_frame):

        """
        :param track: radar track message
        :type track: RadarTrack
        :param ego_speed: speed of our vehicle
        :type ego_speed: std_msgs/TwistStamped
        :param source_frame: frame in which to transform the velocity vector. Map in  this case
        :type source_frame: string
        :return: velocity vector transformed to map frame
        :rtype: std_msgs/Twist
        """

        # correcting a radar object's velocity - computing actual velocity along radar_fc x-axis
        if track.linear_velocity.x == 0:
            corrected_vel = 0
        else:
            corrected_vel = (track.linear_velocity.x**2 + track.linear_velocity.y**2)/(track.linear_velocity.x)
        # Computing speed relative to map.
        velocity_x = ego_speed.twist.linear.x + corrected_vel
        velocity_y = 0
        velocity_z = 0

        velocity_vector = Vector3(velocity_x, velocity_y, velocity_z)

        # transforming the velocity vector to map frame
        velocity_in_map = Twist(linear = self.get_tfed_vector3_from_vector3(velocity_vector, source_frame))

        return velocity_in_map

    def get_tfed_pose_from_pose_with_cov(self, pose_with_cov, source_frame):
        """
        :type pose_with_cov: PoseWithCovariance
        :type source_frame: str
        :returns: tfed Pose to the target_frame
        """

        # To apply a TF we need a pose stamped
        pose_stamped = PoseStamped(pose=pose_with_cov.pose)
        pose_stamped.header.frame_id = source_frame
        tfed_pose = self.tf_listener.transformPose(self.target_frame, pose_stamped).pose
        return tfed_pose

    def get_tfed_vector3_from_vector3(self, vector3, source_frame):
        """
        :type vector3: Vector3
        :type source_frame: str
        :returns: tfed Vector3 to the target_frame class variable
        """

        # To apply a TF we need a Vector3 stamped
        vector3_stamped = Vector3Stamped(vector=vector3)
        vector3_stamped.header.frame_id = source_frame
        tfed_vector3 = self.tf_listener.transformVector3(self.target_frame, vector3_stamped).vector
        return tfed_vector3

    def get_tfed_point32(self, p32, source_frame):
        """

        :param p32: point to be transformed
        :type p32:
        :param source_frame: radar_frame
        :type source_frame: string
        :return: transformed Point32
        :rtype: GeometryMsgs Point32
        """
        point_stamped = PointStamped()
        point_stamped.header.frame_id = source_frame
        point_stamped.point = Point(x=p32.x, y=p32.y, z=p32.z)
        tfed_point = self.tf_listener.transformPoint(self.target_frame, point_stamped).point
        return Point32(x=tfed_point.x, y=tfed_point.y, z=tfed_point.z)

    def get_surrounding_cuboid_size(self, points):
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

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('radar_relay', anonymous=True, log_level=rospy.INFO)
    node = radar_relay()
    node.run()

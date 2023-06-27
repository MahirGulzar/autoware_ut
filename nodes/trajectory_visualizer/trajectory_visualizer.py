#!/usr/bin/python
# coding=utf-8

from __future__ import print_function
from collections import defaultdict
import rospy
from geometry_msgs.msg import Point, Vector3
from autoware_msgs.msg import DetectedObjectArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class TrajectoryVisualizer:
    def __init__(self):

        self.prev_track_ids = set()
        self.mode_set = set()
        self.last_attention_ids = []

        # Subscribers
        rospy.Subscriber("predicted_objects",
                         DetectedObjectArray, self.predicted_objects_callback)

        rospy.Subscriber("yield_eval_attention",
                         DetectedObjectArray, self.yield_eval_attention_callback)

        # Publishers
        self.marker_array_pub = rospy.Publisher(
            "/trajectory_markers", MarkerArray, queue_size=10)

    def predicted_objects_callback(self, msg):
        """
        :type msg: DetectedObjectArray
        """

        trajectory_marker_array = MarkerArray()
        mark_attention = False

        curr_modes = defaultdict(list)
        track_ids = []

        for predicted_object in msg.objects:

            traj_size = len(predicted_object.candidate_trajectories.lanes)

            track_ids.append(predicted_object.id)

            if predicted_object.id in self.last_attention_ids:
                mark_attention = True
            
            if traj_size == 0 and predicted_object.velocity.linear.x > 0:
                rospy.logwarn("No trajectory for ObstacleID %s with speed %s", str(predicted_object.id), str(predicted_object.velocity.linear.x))

            for i in range(traj_size):

                line_marker = Marker()
                line_marker.id = predicted_object.id
                line_marker.ns = "mode_"+str(i)
                self.mode_set.add("mode_"+str(i))
                curr_modes[predicted_object.id].append("mode_"+str(i))
                line_marker.header.frame_id = "map"
                line_marker.type = Marker.LINE_STRIP
                line_marker.action = line_marker.ADD
                line_marker.pose.orientation.w = 1.0
                line_marker.scale = Vector3(x=0.5, y=0.5, z=0.5)
                if mark_attention:
                    line_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
                else:
                    if predicted_object.candidate_trajectories.lanes[i].cost <= 0.5:
                        line_marker.color = ColorRGBA(
                            r=1.0, g=1.0, b=0.0, a=1.0)
                    else:
                        line_marker.color = ColorRGBA(
                            r=0.0, g=1.0, b=1.0, a=1.0)
                for waypoint in predicted_object.candidate_trajectories.lanes[i].waypoints:
                    line_marker.points.append(Point(
                        waypoint.pose.pose.position.x, waypoint.pose.pose.position.y, waypoint.pose.pose.position.z))

                trajectory_marker_array.markers.append(line_marker)

            mark_attention = False

        new_tracks = set(track_ids)
        tracks_to_delete = self.prev_track_ids - new_tracks
        self.prev_track_ids = new_tracks

        for ns in self.mode_set:
            for old_track in tracks_to_delete:

                line_marker = Marker()
                line_marker.id = old_track
                line_marker.header.frame_id = "map"
                line_marker.type = Marker.LINE_STRIP
                line_marker.action = line_marker.DELETE
                line_marker.ns = ns
                line_marker.pose.orientation.w = 1.0
                line_marker.scale = Vector3(x=0.5, y=0.5, z=0.5)
                trajectory_marker_array.markers.append(line_marker)

            for key in curr_modes:
                if ns not in curr_modes[key]:

                    line_marker = Marker()
                    line_marker.id = key
                    line_marker.header.frame_id = "map"
                    line_marker.type = Marker.LINE_STRIP
                    line_marker.action = line_marker.DELETE
                    line_marker.ns = ns
                    line_marker.pose.orientation.w = 1.0
                    line_marker.scale = Vector3(x=0.5, y=0.5, z=0.5)
                    trajectory_marker_array.markers.append(line_marker)

        self.marker_array_pub.publish(trajectory_marker_array)

    def yield_eval_attention_callback(self, msg):
        """
        :type msg: DetectedObjectArray
        """

        attention_ids = []

        for predicted_object in msg.objects:
            attention_ids.append(predicted_object.id)

        self.last_attention_ids = attention_ids

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('trajectory_visualizer',
                    anonymous=False, log_level=rospy.INFO)
    node = TrajectoryVisualizer()
    node.run()

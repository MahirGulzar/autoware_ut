#!/usr/bin/env python2
# coding=utf-8

import numpy as np
import rospy
from autoware_msgs.msg import DetectedObjectArray
from tf import TransformListener, LookupException, ExtrapolationException
from track_gen import NNDATracker


class Tracker:
    def __init__(self):
        # ----------------
        # Node attributes
        # ----------------
        # Time stamp of DetectedObjectArray msg in previous frame
        self.last_detection_msg_stamp = None
        self.curr_simulated_object_msg = None   # Latest simulated objects msg
        self.listener = TransformListener()

        # ----------------------------------------------------------
        # Tracker Modes, Both modes can be enabled at the same time
        # ----------------------------------------------------------
        self.sensor_mode = rospy.get_param('~sensor_mode', default=True)
        self.simulation_mode = rospy.get_param('~simulation_mode', default=False)

        # ------------
        # Subscribers
        # ------------

        # NOTE: No need to time synchronize two callbacks, Sensor objects will always have priority

        if self.simulation_mode:
            # Sensor's Detected object subscriber
            rospy.Subscriber("input_simulated_objects", DetectedObjectArray, self.simulated_objects_callback)

        if self.sensor_mode:
            # Sensor's Detected object subscriber
            rospy.Subscriber("input_objects", DetectedObjectArray, self.detected_objects_callback)

        # -----------
        # Publishers
        # -----------
        self.tracked_objects_pub = rospy.Publisher("objects", DetectedObjectArray, queue_size=1)

        # ------------------------------------
        # General parameters of tracking node
        # ------------------------------------

        # Frame id in which tracking should be applied
        self.tracking_frame = rospy.get_param('~tracking_frame', default="map")
        min_life_threshold = rospy.get_param('~min_life_threshold', default=0.7)
        min_detection_thresh = rospy.get_param('~min_detection_thresh', default=0)
        max_misdetection_thresh = rospy.get_param('~max_misdetection_thresh', default=0.3)
        gating_treshold = rospy.get_param('~gating_treshold', default=0.99)

        # NNDA tracker instance
        self.tracker = NNDATracker(min_life_threshold=min_life_threshold, min_detection_thresh=min_detection_thresh,
                                   max_misdetection_thresh=max_misdetection_thresh, gating_treshold=gating_treshold)

    def detected_objects_callback(self, msg):
        """ Callback to get detected objects

        Parameters
        ----------
        msg : DetectedObjectArray
        """

        # Separating objects and positions because tracking & association will be applied on object positions
        object_positions = []
        detected_objects = []

        # --------------------------
        # Acquire simulated objects
        # --------------------------
        if self.simulation_mode:

            if self.curr_simulated_object_msg is not None:
                for detected_object in self.curr_simulated_object_msg.objects:
                    detected_objects.append(detected_object)
                    object_positions.append(
                        [detected_object.pose.position.x, detected_object.pose.position.y, detected_object.pose.position.z])

        # ---------------------------------
        # Acquire realtime sensors objects
        # ---------------------------------
        if self.sensor_mode:

            for detected_object in msg.objects:
                detected_objects.append(detected_object)
                object_positions.append(
                    [detected_object.pose.position.x, detected_object.pose.position.y, detected_object.pose.position.z])

        # lists to numpy array for similar slicing operations in Tracker
        object_positions = np.array(object_positions)
        detected_objects = np.array(detected_objects)


        # For first message use a 0.1 sec as time delta
        if self.last_detection_msg_stamp is None:
            dt = 0.1
        else:
            dt = (msg.header.stamp - self.last_detection_msg_stamp).to_sec()

        self.last_detection_msg_stamp = msg.header.stamp

        # Transform detected object centroids to tracking frame
        if self.tracking_frame is not None and len(object_positions) != 0:
            try:
                # Transform matrix from msg frame to tracking frame
                transform_matrix = self.listener.asMatrix(self.tracking_frame, msg.header)
            except (LookupException, ExtrapolationException) as e:
                rospy.logwarn("Transform not found " + str(e))
                return

            object_positions = np.concatenate([object_positions, np.ones((object_positions.shape[0], 1))], axis=1)
            object_positions = np.dot(object_positions, transform_matrix.T)[:, :3]

        # Apply stage-1 of tracking
        tracks = self.tracker.track_stage_one(object_positions, detected_objects, dt=dt)

        # TODO: Implement stage-2 refinement

        # Clear data placeholder
        detected_objects = []

        # Extract tracking results in separate lists
        for track in tracks:
            detected_objects.append(track.get_detected_object())

        # Populate autoware detected objects array
        tracked_objects_autoware = DetectedObjectArray()
        tracked_objects_autoware.header = msg.header
        tracked_objects_autoware.objects = detected_objects
        self.tracked_objects_pub.publish(tracked_objects_autoware)

    def simulated_objects_callback(self, msg):
        """ Callback to get simulated detected objects

        Parameters
        ----------
        msg : DetectedObjectArray
        """
        self.curr_simulated_object_msg = msg

        # As callbacks are not syncrhonized, call detected object processing explicitly
        if not self.sensor_mode:
            template_msg = DetectedObjectArray()
            template_msg.header.frame_id = self.curr_simulated_object_msg.header.frame_id
            template_msg.header.stamp = self.curr_simulated_object_msg.header.stamp
            self.detected_objects_callback(template_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('cvkf_track', anonymous=False, log_level=rospy.INFO)
    node = Tracker()
    node.run()
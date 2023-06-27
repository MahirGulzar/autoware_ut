#!/usr/bin/env python
from autoware_msgs.msg import Signals, TrafficLight, ExtractedPosition
import rospy
import std_msgs.msg

class RoiMerger:
    def __init__(self):

        self.use_yolo = rospy.get_param('~use_yolo', False)

        # Subscribers and tf listeners
        self.ttl1 = 1 # Fl timeout
        self.ttl2 = 1 # Fr timeout
        self.ttl3 = 3 # API timeout

        self.signals1 = list()
        self.signals2 = list()
        self.signals3 = list()
        self.cur_stpl = list()

        self.signals1_stamp = 0 
        self.signals2_stamp = 0 
        self.signals3_stamp = 0 

        self.light1= 2 # UNKNOWN
        self.light2= 2 # UNKNOWN
        self.prev_light = 2 # UNKNOWN

        if self.use_yolo:
            roi1_sub = rospy.Subscriber('camera_fl/roi_yolo_signal', Signals, self.callback1)
            roi2_sub = rospy.Subscriber('camera_fr/roi_yolo_signal', Signals, self.callback2)

        else:
            roi1_sub = rospy.Subscriber('camera_fl/roi_astuff_signal', Signals, self.callback1)
            roi2_sub = rospy.Subscriber('camera_fr/roi_astuff_signal', Signals, self.callback2)
            

        roi3_sub = rospy.Subscriber('ib/roi_signal', Signals, self.callback3)

        # Publishers - will aggregate into stopline level
        self.roi_signal_pub = rospy.Publisher('roi_signal', Signals, queue_size=1)
        self.light_color_pub = rospy.Publisher('light_color', TrafficLight, queue_size=1)
        self.r = rospy.Rate(10) # 10hz

    def run(self):
        while not rospy.is_shutdown():
            self.combine_and_publish()
            self.r.sleep()

    def callback1(self, roi_msg):
        self.signals1 = roi_msg.Signals
        if len(self.signals1) > 0:
            self.signals1_stamp = rospy.get_time()
    
    def callback2(self, roi_msg):
        self.signals2 = roi_msg.Signals
        if len(self.signals2) > 0:
            self.signals2_stamp = rospy.get_time()
    
    def callback3(self, roi_msg):
        self.signals3 = roi_msg.Signals
        if len(self.signals3) > 0:
            self.signals3_stamp = rospy.get_time()

    def combine_and_publish(self):
        roi_signals_msg = Signals()
        comb_signals = list()

        # combine signals and merge
        # TODO: should check timestamps?
        comb_signals = self.signals3 + self.signals1 + self.signals2
        merged_signals = {}
          
        # add signals to dict and sum the radiuses of different light types
        for signal in comb_signals:
            if signal.linkId  not in merged_signals.keys():
                merged_signals[signal.linkId] = [0,0,0,0]
            merged_signals[signal.linkId][signal.type] += signal.radius

        # container for signals aggregated in stopline level 
        unique_signals = list()

        for linkId in merged_signals:
            exp = ExtractedPosition()

            # find index with max radius - determines the signal type
            max_index = max(range(len(merged_signals[linkId])), key=merged_signals[linkId].__getitem__)

            exp.linkId = linkId
            exp.type = max_index
            exp.radius = merged_signals[linkId][max_index]
            
            unique_signals.append(exp)

        roi_signals_msg.Signals = unique_signals

        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        roi_signals_msg.header = h
        self.roi_signal_pub.publish(roi_signals_msg)


if __name__ == '__main__':
    rospy.init_node('roi_merger', anonymous=True)
    node = RoiMerger()
    node.run()

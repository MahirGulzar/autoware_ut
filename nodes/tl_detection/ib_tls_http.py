#!/usr/bin/env python

from autoware_msgs.msg import Signals,ExtractedPosition
import rospy
import roslib
roslib.load_manifest('diagnostic_updater')
import diagnostic_msgs
import diagnostic_updater
import std_msgs.msg
import requests
import json
import signal
from timeit import default_timer as timer

ib_light_to_op = {"RED" : 2,
        "GREEN": 0,
        "GREEN FLASH": 2,
        "AMBER": 1,
        "AMBER-RED": 2,
        "RED/AMB": 2,
        "AMB FLASH" : 0,
        "AMBER FLASH" : 0,
        "AMBERRED" : 2}

class IBLights:
    def __init__(self):
        self.request_timeout = 1.5
        self.user_timeout = 2 # Must be integer for signal.alarm
        self.api_url = rospy.get_param('~api_url')
        tl_grouping_file = rospy.get_param('~tl_grouping_file')

        self.ib_id_to_op = None
        with open(tl_grouping_file) as json_file:
            self.ib_id_to_op = json.load(json_file)

        self.pub = rospy.Publisher('ib/roi_signal', Signals, queue_size=10)
        self.r = rospy.Rate(10) # 10hz
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("none")

        self.is_unknown_state = False
        self.unknown_state = None
        self.missing_tls = []
        self.num_of_timeouts = 0
        self.updater.add("Status", self.diag_task)
        self.session = requests.session()
        self.session.keep_alive = True # explicitly set to true

        rospy.loginfo("Starting TRAFFEST API")
        self.count = 0
        self.total = 0.0
        self.response_time = 0.0
        self.tl_avail_dict = {k:0.0 for (k,v) in self.ib_id_to_op.items()}
        self.tl_avail_count = {k:0 for (k,v) in self.ib_id_to_op.items()}

        signal.signal(signal.SIGALRM, self.timeout_handler)


    def run(self):
        while not rospy.is_shutdown():
            self.combine_and_publish()
            self.updater.update()
            self.r.sleep()

    def get_signals(self):
        signals = list()
        self.response_time = 0
        try:
            signal.alarm(self.user_timeout)
            start = timer()
            self.res = self.session.get(self.api_url, timeout = self.request_timeout).json()
            end = timer()
            signal.alarm(0)
            self.response_time = end - start
            self.num_of_timeouts = 0
        except Exception as e:
            rospy.logwarn('Traffic lights API exception: ' + str(e))
            self.res = {}
            self.num_of_timeouts += 1
        self.count += 1
        self.missing_tls = []
        for idx in self.ib_id_to_op.keys():
            if idx in self.res and len(self.res[idx][0]) > 0:
                self.tl_avail_count[idx] += 1
                state = self.res[idx][0]
                if state not in ib_light_to_op.keys():
                    print state
                    rospy.logwarn("TRAFFEST: Unknown state: " + state)
                    self.is_unknown_state = True
                    self.unknown_state = state
                    state = "RED"
            else:
                self.missing_tls.append(int(idx))
                continue
            for op_id in self.ib_id_to_op[idx]:
                exp = ExtractedPosition()
                exp.linkId = op_id
                exp.radius = 10000
                exp.type = ib_light_to_op[state]
                signals.append(exp)
        self.tl_avail_dict = {k:str(float(self.tl_avail_count[k])/self.count * 100) + '%' for (k,v) in self.ib_id_to_op.items()}
        return signals

    def combine_and_publish(self):
        ib_signals = self.get_signals()
        msg = Signals()
        msg.Signals = ib_signals
        #h = std_msgs.msg.Header()
        #h.stamp = rospy.Time.now()
        msg.header.stamp = rospy.Time.now()
        self.pub.publish(msg)

    def diag_task(self, stat):
        if self.num_of_timeouts >= 3:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR,
                    "API down for %d requests" % self.num_of_timeouts)
        elif self.res == {}:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, 
                        "Did not receive API data")
        elif len(self.missing_tls) > 0:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN,
                    "Missing some traffic lights")
        elif self.is_unknown_state:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, 
                        "Unknown traffic light state")
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 
                        "OK!")
        stat.add("API Down", self.num_of_timeouts >= 3)
        stat.add("Received Data", len(self.res) > 0)
        stat.add("Unknown TL state", self.unknown_state)
        stat.add("Missing traffic light IDs", self.missing_tls)
        stat.add("API response time", self.response_time)
        stat.add("% of traffic light availability", self.tl_avail_dict)
        
        if self.is_unknown_state:
            self.is_unknown_state = False
            self.unknown_state = None
        return stat

    def timeout_handler(self, signum, frame):
        raise Exception("TRAFFEST: user level timeout (={} seconds)".format(self.user_timeout))

if __name__ == '__main__':
    rospy.init_node('ib_tls', anonymous=True)
    node = IBLights()
    node.run()

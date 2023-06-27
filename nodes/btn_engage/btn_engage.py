#!/usr/bin/env python3

import rospy
from panda import Panda
from std_msgs.msg import Bool
import time

class btn_engage:
    panda = None
    rate = None
    engage_pub = None
    cc_status = None
    time_since_last = None

    def __init__(self):
        rospy.loginfo(self.__class__.__name__ + " - node started")
        self.rate = rospy.Rate(20)

        try:
            self.panda = Panda()
            self.panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
        except AssertionError:
            rospy.logerr(self.__class__.__name__ + " - Panda USB not found, exiting ...")
            rospy.signal_shutdown("Panda USB not found.")
        else:
            self.engage_pub = rospy.Publisher("vehicle/engage", Bool, queue_size=10)

    def run(self):
        while not rospy.is_shutdown():
            rospy.loginfo_once(self.__class__.__name__ + " - Initialized")
            can_recv = self.panda.can_recv()
            for address, _, dat, src in can_recv:
                if address == 467:
                    cc_status_old = self.cc_status
                    self.cc_status = dat[1] & 128 == 128

                    if cc_status_old != self.cc_status and cc_status_old is not None:

                        if self.time_since_last is not None and time.time() - self.time_since_last < 2.:
                            break

                        self.engage_pub.publish(Bool(data=True))
                        rospy.loginfo("Pandacan engage triggered by : '%s'" % dat[1])
                        self.time_since_last = time.time()
                    break
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('btn_engage', anonymous=True, log_level=rospy.INFO)
    node = btn_engage()
    node.run()

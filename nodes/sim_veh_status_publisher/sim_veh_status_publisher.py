#!/usr/bin/python

from __future__ import print_function
import rospy
import signal
from nav_msgs.msg import Odometry
from autoware_msgs.msg import VehicleStatus


def terminate_node(signum=None, frame=None):
    rospy.loginfo("Shutting down ...")
    rospy.signal_shutdown("")
    exit(0)


class sim_veh_status_publisher:

    pub = rospy.Publisher('vehicle_status', VehicleStatus, queue_size=10)

    def __init__(self):
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.loginfo("Starting node.")

    def odom_callback(self, odom):
        # speed = odom.twist.linear.x
        self.build_status(round(odom.twist.twist.linear.x, 2))

    def build_status(self, speed):
        veh_status = VehicleStatus(speed=speed*3.6)
        self.pub.publish(veh_status)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    signal.signal(signal.SIGINT, terminate_node)
    rospy.init_node('sim_veh_status_publisher', anonymous=True, log_level=rospy.INFO)

    node = sim_veh_status_publisher()

    node.run()

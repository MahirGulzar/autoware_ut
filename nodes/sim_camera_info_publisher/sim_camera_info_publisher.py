#!/usr/bin/python

from __future__ import print_function
import rospy
import signal
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from math import tan, radians


def terminate_node(signum=None, frame=None, exit_code=0):
    rospy.loginfo("Shutting down ...")
    rospy.signal_shutdown("")
    exit(exit_code)


class sim_camera_info_publisher:
    cam_info = CameraInfo()

    pub = None
    fovx = None
    fovy = None
    resx = None
    resy = None
    cam_id = None
    fx = None
    fy = None

    def __init__(self):
        rospy.Subscriber('/camera_fr/image/compressed', CompressedImage, self.compressed_img_callback)
        rospy.loginfo("Starting node.")

        self.fovx = rospy.get_param('~fovx', None)
        self.fovy = rospy.get_param('~fovy', None)
        self.resx = rospy.get_param('~resx', None)
        self.resy = rospy.get_param('~resy', None)
        self.cam_id = rospy.get_param('~cam_id', None)

        # Asserting that all params are set.
        if self.fovx is None or self.fovy is None or self.resx is None or self.resy is None or self.cam_id is None:
            rospy.logfatal(rospy.get_name() + "All params are not set.")
            terminate_node(exit_code=1)

        rospy.loginfo(rospy.get_name() + " fovx: " + str(self.fovx))
        rospy.loginfo(rospy.get_name() + " fovy: " + str(self.fovy))
        rospy.loginfo(rospy.get_name() + " resx: " + str(self.resx))
        rospy.loginfo(rospy.get_name() + " resy: " + str(self.resy))

        self.pub = rospy.Publisher('/' + self.cam_id + '/camera_info', CameraInfo, queue_size=10)

        self.compute_camera_info()

    def compute_camera_info(self):
        """
        Computes the camera info message for a perfect camera with the focals and image dimensions
        """
        self.cam_info.header.frame_id = self.cam_id
        self.cam_info.distortion_model = "plumb_bob"
        self.cam_info.height = self.resy
        self.cam_info.width = self.resx

        self.fx = self.resx / (2. * tan(radians(self.fovx) * .5))
        self.fy = self.resy / (2. * tan(radians(self.fovy) * .5))

        self.cam_info.D = [0., 0., 0., 0., 0.]

        self.cam_info.K = [self.fx, 0.0,        self.resx / 2.,
                           0.0,     self.fy,    self.resy / 2.,
                           0.0,     0.0,        1.0]

        self.cam_info.P = [self.fx, 0.0,        self.resx / 2., 0.0,
                           0.0,     self.fy,    self.resy / 2., 0.0,
                           0.0,     0.0,        1.0,            0.0]

        self.cam_info.R = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def compressed_img_callback(self, img):
        """
        Gets a compressed image, and will use its header to publish a synced camera info.
        :type img: CompressedImage
        """
        self.cam_info.header.stamp = img.header.stamp
        self.pub.publish(self.cam_info)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    signal.signal(signal.SIGINT, terminate_node)
    rospy.init_node('sim_camera_info_publisher', anonymous=True, log_level=rospy.INFO)
    node = sim_camera_info_publisher()
    node.run()

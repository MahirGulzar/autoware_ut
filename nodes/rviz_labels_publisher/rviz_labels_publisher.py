#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import ColorRGBA
from autoware_msgs.msg import Lane, VehicleStatus
from visualization_msgs.msg import MarkerArray
from jsk_rviz_plugins.msg import OverlayText

class RvizLabelsPublisher:

    def __init__(self):

        self.dist_pub = rospy.Publisher('closest_object_distance', Float32, queue_size=1)
        self.velocity_pub = rospy.Publisher('closest_object_velocity', Float32, queue_size=1)
        self.vehicle_drive_mode_autonomy_pub = rospy.Publisher('vehicle_autonomy_drive_mode', String, queue_size=1)
        self.vehicle_drive_mode_manual_pub = rospy.Publisher('vehicle_manual_drive_mode', String, queue_size=1)
        self.vehicle_behavior_state_pub = rospy.Publisher('behavior_dash', OverlayText, queue_size=1)
        self.rviz_info_1st_stopline_pub = rospy.Publisher('rviz_info_1st_stopline', OverlayText, queue_size=1)
        self.rviz_info_2nd_stopline_pub = rospy.Publisher('rviz_info_2nd_stopline', OverlayText, queue_size=1)

        rospy.init_node('rviz_labels_publisher', anonymous=False)

        self.local_traj_cost_sub = rospy.Subscriber('/local_trajectory_cost', Lane, self.local_traj_cost_callback)
        self.ego_status_sub = rospy.Subscriber('/vehicle_status', VehicleStatus, self.ego_status_callback)
        self.behavior_state_sub = rospy.Subscriber('/behavior_state', MarkerArray, self.behavior_state_callback)
        self.stopline_info_sub = rospy.Subscriber('/rviz_info_stop_lines_tfls', String, self.stopline_info_callback)

    def local_traj_cost_callback(self, msg):
        self.dist_pub.publish(Float32(msg.closest_object_distance))
        self.velocity_pub.publish(Float32(msg.closest_object_velocity * 3.6))

    def ego_status_callback(self, msg):
        if msg.drivemode == 1:
            self.vehicle_drive_mode_autonomy_pub.publish(String("Driver: AUTONOMY"))
            self.vehicle_drive_mode_manual_pub.publish(String(""))
        else:
            self.vehicle_drive_mode_manual_pub.publish(String("Driver: MANUAL"))
            self.vehicle_drive_mode_autonomy_pub.publish(String(""))

    def behavior_state_callback(self, msg):
        # read in the data and process
        string = msg.markers[0].text.split(")")
        fg_color = msg.markers[0].color
        text_state = "State: " + string[-1]

        self.dash = OverlayText()

        self.dash.text = text_state
        self.dash.fg_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        #self.dash.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.8)
        self.dash.width = 250
        self.dash.height = 25
        self.dash.left = 10
        self.dash.top = 180
        self.dash.text_size = 11
        self.dash.line_width = 2
        self.dash.font = "DejaVu Sans Mono"
        self.vehicle_behavior_state_pub.publish(self.dash)

    def stopline_info_callback(self, msg):     

        self.stopline = OverlayText()

        # self.stopline.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.8)
        self.stopline.width = 250
        self.stopline.height = 20
        self.stopline.left = 25
        self.stopline.top = 205
        self.stopline.text_size = 10
        self.stopline.line_width = 2
        self.stopline.font = "DejaVu Sans Mono"

        self.stopline.fg_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        
        # input string is in format
        # "GRE api (4 m);"                   split(";") len 2
        # "GRE api (53 m);RED api (129 m);"  split(";") len 3
        # ""                                 split(";") len 1  - no tfls or stoplines

        string = msg.data.split(";")
        
        if len(string) == 2:
            # first stopline
            if string[0].startswith("R"):
                self.stopline.fg_color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
            elif string[0].startswith("G"):
                self.stopline.fg_color = ColorRGBA(0.0, 1.0, 0.0, 1.0)
            self.stopline.text = "        " + string[0]
            self.rviz_info_1st_stopline_pub.publish(self.stopline)
            
            # 2nd stopline
            self.stopline.top = self.stopline.top + self.stopline.height
            self.stopline.text = ""
            self.rviz_info_2nd_stopline_pub.publish(self.stopline)
        elif len(string) >= 3:
            # first stopline
            if string[0].startswith("R"):
                self.stopline.fg_color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
            elif string[0].startswith("G"):
                self.stopline.fg_color = ColorRGBA(0.0, 1.0, 0.0, 1.0)
            self.stopline.text = string[0]
            self.rviz_info_1st_stopline_pub.publish(self.stopline)
            
            # 2nd stopline
            self.stopline.top = self.stopline.top + self.stopline.height
            if string[1].startswith("R"):
                self.stopline.fg_color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
            elif string[1].startswith("G"):
                self.stopline.fg_color = ColorRGBA(0.0, 1.0, 0.0, 1.0)
            else:
                self.stopline.fg_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
            self.stopline.text = string[1]
            self.rviz_info_2nd_stopline_pub.publish(self.stopline)
        else:
            # first stopline
            self.stopline.fg_color = ColorRGBA(0.5, 0.5, 0.5, 1.0)
            self.stopline.text = "no stoplines within horizon"
            self.rviz_info_1st_stopline_pub.publish(self.stopline)
            
            # 2nd stopline
            self.stopline.text = ""
            self.stopline.top = self.stopline.top + self.stopline.height
            self.rviz_info_2nd_stopline_pub.publish(self.stopline)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = RvizLabelsPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass

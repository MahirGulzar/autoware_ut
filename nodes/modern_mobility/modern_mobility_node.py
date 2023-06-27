#!/usr/bin/env python

import requests
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Joy
import math
import os
import numpy as np
import json
from pyproj import CRS, Transformer

class ModernMobilityNode():
    def __init__(self):
        self.api_key = rospy.get_param("~api_key")
        self.vehicleId = rospy.get_param("~vehicleId")
        self.areaId = rospy.get_param("~areaId")
        self.service_url = rospy.get_param("~service_url")
        self.complete_distance = rospy.get_param("~complete_distance")
        self.complete_speed = rospy.get_param("~complete_speed")
        self.stops_file = rospy.get_param("~stops_file")
        self.stopId = None
        self.stop_coords = None
        self.position = []
        self.orientation = []
        self.name = ""
        self.car_pose_x = 0
        self.car_pose_y = 0
        self.overlay_message = ""
        self.speed = 0
        self.dropoff_points = []
        self.dropoffs = []
        self.read_file()

        crs_4326 = CRS.from_epsg(4326)
        crs_3301 = CRS.from_epsg(3301)

        self.gps_to_ros = Transformer.from_crs(crs_4326, crs_3301)
        self.ros_to_gps = Transformer.from_crs(crs_3301, crs_4326)

        # block nav goal publisher until the goal is sent to all subscribers
        self.goal_publisher = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=None)
        self.mm_stop_overlay_publisher = rospy.Publisher("modern_mobility_node", String, queue_size=1)
        self.mm_status_overlay_publisher = rospy.Publisher("modern_mobility_node/status", String, queue_size=1)
        self.op_last_goal_publisher = rospy.Publisher("op_take_last_goal", Bool, queue_size=1)
        self.op_global_replan_publisher = rospy.Publisher("op_global_replan", Bool, queue_size=1)
        self.pose_subscriber = rospy.Subscriber("current_pose", PoseStamped, self.pose_callback)
        self.event_subscriber = rospy.Subscriber("joy", Joy, self.joy_callback)
        self.speed_subscriber = rospy.Subscriber("current_velocity", TwistStamped, self.speed_callback)

    def read_file(self):
        with open(os.path.join(self.stops_file)) as data_file:
            self.dropoffs = json.load(data_file)
            for dropoff in self.dropoffs["dropoff"]:
                dropoff_coordinates = (dropoff["position"][0], dropoff["position"][1])
                self.dropoff_points.append(dropoff_coordinates)
        self.dropoff_points = np.asarray(self.dropoff_points)

    def calculate(self, point):
        dist_2 = np.sqrt(np.sum((self.dropoff_points - point)**2, axis=1))
        closest_index = np.argmin(dist_2)
        dropoff_point = self.dropoffs["dropoff"][closest_index]
        return dropoff_point["position"], dropoff_point["orientation"], dropoff_point["name"]

    def to_ros(self, lat, lng):
        converted_coords = self.gps_to_ros.transform(lat, lng)

        converted_lat = converted_coords[0] - 6465000
        converted_lng = converted_coords[1] - 650000
        return (converted_lat, converted_lng)
    
    def to_gps(self, lat, lng):
        lat += 6465000
        lng += 650000

        converted_coords = self.ros_to_gps.transform(lat, lng)
        return converted_coords

    def speed_callback(self, data):
        speed_m_s = data.twist.linear.x
        self.speed = speed_m_s * 3.6
    
    def pose_callback(self, data):
        self.car_pose_x = data.pose.position.x
        self.car_pose_y = data.pose.position.y

    def joy_callback(self, data):
        if data.buttons[1] == 1:
            self.global_replan()
        #elif data.buttons[2] == 0:
        #    self.stop_cancelled()
        elif data.buttons[3] == 1:
            self.stop_completed()

    def global_replan(self):
        self.op_global_replan_publisher.publish(Bool(True))

    def publish_2d_nav_goal(self, pose_x, pose_y, pose_z, orient_x, orient_y, orient_z, orient_w):
        goal = PoseStamped()

        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "world"

        goal.pose.position.x = pose_x
        goal.pose.position.y = pose_y
        goal.pose.position.z = pose_z

        goal.pose.orientation.x = orient_x
        goal.pose.orientation.y = orient_y
        goal.pose.orientation.z = orient_z
        goal.pose.orientation.w = orient_w

        self.goal_publisher.publish(goal)
        rospy.loginfo("Published goal")
        self.mm_status_overlay_publisher.publish("Destination sent to OpenPlanner")
    
    def distance(self, origin, destination):
        x1, y1 = origin
        x2, y2 = destination

        distance = math.sqrt(((x1 - x2)**2 + (y1 - y2)**2))

        return distance

    def stop_cancelled(self):
        cancelled_object = {
            "areaId": self.areaId,
            "vehicleId": self.vehicleId,
            "reason": "KAIGAS_IMPEDES_KODAR"
        }

        try:
            cancelled_post = requests.post(self.service_url + "/stop/" + str(self.stopId) + "/canceled", headers={"api-key": self.api_key}, data=cancelled_object, timeout=5)
        except Exception as e:
            rospy.logerr("Error canceling stop: %s", str(e))
            self.mm_status_overlay_publisher.publish("Error canceling stop: " + str(e))
            return

        if cancelled_post.status_code == 200:
            if cancelled_post.json()["ok"] == True:
                rospy.loginfo("Stop %s marked cancelled", str(self.stopId))
                self.mm_status_overlay_publisher.publish("Stop " + self.name + " marked as cancelled")
            else:
                rospy.logerr("Something went wrong: %s", cancelled_post.text)
                self.mm_status_overlay_publisher.publish("Something went wrong: " + cancelled_post.text)
        else:
            rospy.logerr("Bad StopCanceled request: %d %s", cancelled_post.status_code, cancelled_post.text)
            self.mm_status_overlay_publisher.publish("Bad StopCanceled request: " + cancelled_post.text)


    def stop_completed(self):
        completed_object = {
            "areaId": self.areaId,
            "vehicleId": self.vehicleId
        }
        
        try:
            completed_post = requests.post(self.service_url + "/stop/" + str(self.stopId) + "/completed", headers={"api-key": self.api_key}, data=completed_object, timeout=5)
        except Exception as e:
            rospy.logerr("Error completing stop: %s", str(e))
            self.mm_status_overlay_publisher.publish("Error completing stop: " + str(e))
            return

        if completed_post.status_code == 200:
            if completed_post.json()["ok"] == True:
                rospy.loginfo("Stop %s marked completed", str(self.stopId))
                self.mm_status_overlay_publisher.publish("Stop " + self.name + " marked as completed")
            else:
                rospy.logerr("Something went wrong: %s", completed_post.text)
                self.mm_status_overlay_publisher.publish("Something went wrong: " + completed_post.text)
        else:
            rospy.logerr("Bad StopComplete request: %d %s", completed_post.status_code, completed_post.text)
            self.mm_status_overlay_publisher.publish("Bad StopComplete request: " + completed_post.text)


    def update_status(self):
        (lat, lon) = self.to_gps(self.car_pose_x, self.car_pose_y)
        status_object = { 
            "vehicleId": self.vehicleId,
            "areaId": self.areaId,
            "lat": lat,
            "lon": lon,
            "passengerCount": 2,
            "speed": round(self.speed),
            "stopId": self.stopId
        }

        try:
            status_post = requests.post(self.service_url + "/status", headers={"api-key": self.api_key}, data=status_object, timeout=1)
        except Exception as e:
            rospy.logerr("Error during status update: %s", str(e))
            return

        if status_post.status_code == 200:
            rospy.loginfo("Status updated successfully")
        else:
            rospy.logerr("Bad UpdateStatus request: %d %s", status_post.status_code, status_post.text)

    def get_next_stop(self):
        stop_object = {
            "areaId": self.areaId,
            "vehicleId": self.vehicleId
        }

        try:
            stop_post = requests.post(self.service_url + "/stop", headers={"api-key": self.api_key}, data=stop_object, timeout=1)
        except Exception as e:
            rospy.logerr("Error getting next stop: %s", str(e))
            return

        if stop_post.status_code == 200:
            if stop_post.text == "{}":
                rospy.loginfo("No stop available yet")
                self.mm_stop_overlay_publisher.publish("No next stop")
                self.stopId = None
            elif self.stopId == stop_post.json()['stopId']:
                calculated_dist = self.distance((self.car_pose_y, self.car_pose_x), self.stop_coords)
                if calculated_dist <= self.complete_distance and self.speed <= self.complete_speed:
                    self.stop_completed()
                else:
                    self.mm_stop_overlay_publisher.publish("Next stop: " + self.name)
            else:
                rospy.loginfo("Got next stop")
                stop_response = stop_post.json()
                self.stopId = int(stop_response['stopId'])
                (lat, lon) = (stop_response['lat'], stop_response['lon'])
                (ros_lat, ros_lon) = self.to_ros(lat, lon)
                self.position, self.orientation, self.name = self.calculate((ros_lon, ros_lat))
                self.stop_coords = (self.position[1], self.position[0])
                self.mm_stop_overlay_publisher.publish("Next stop: " + self.name)
                rospy.loginfo("Next stop: %s", str(self.stopId))
                self.publish_2d_nav_goal(self.position[0], self.position[1], self.position[2], self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3])
                self.op_last_goal_publisher.publish(Bool(True))
        else:
            self.mm_status_overlay_publisher.publish("Error fetching next stop: " + stop_post.text)
            rospy.logerr("Bad GetNextStop request: %d %s", stop_post.status_code, stop_post.text)

    def modern_mobility_API_calls(self, event=None):
        self.update_status()
        self.get_next_stop()
    
    def run(self):
        rospy.Timer(rospy.Duration(5), self.modern_mobility_API_calls)
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('modern_mobility_node')
    mm_node = ModernMobilityNode()
    mm_node.run()

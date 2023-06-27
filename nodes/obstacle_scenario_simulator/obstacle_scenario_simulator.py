#!/usr/bin/python
# coding=utf-8
from __future__ import division
from autoware_msgs.msg import DetectedObjectArray, DetectedObject
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Vector3, Quaternion, Point32
import pytweening
import pandas as pd
import numpy as np
import rospy
import json
import math
import os


class MotionProfile:
    def __init__(self, replan, looper, velocity, stop_time, initial_delay, tween_ease_function):
        self.replan = replan
        self.looper = looper
        self.velocity = velocity
        self.stop_time = stop_time
        self.initial_delay = initial_delay
        self.tween_ease_function = tween_ease_function
        
class WaypointSimulator:
    def __init__(self, waypoints, motion_profile):
        self._waypoints = waypoints
        self._motion_profile = motion_profile
        self._ease_function = getattr(pytweening, self._motion_profile.tween_ease_function)

        self.__curr_waypoint_index = 1
        self.__curr_pose = [
            self._waypoints[self.__curr_waypoint_index][0],
            self._waypoints[self.__curr_waypoint_index][1],
            self._waypoints[self.__curr_waypoint_index][2],
            self._waypoints[self.__curr_waypoint_index][3]
        ]

        self.__fixed_time_delta = 0.02
        self.__accumulated_dt = 0.0
        self.__next = False
        self.__simulation_ended = False

    @property
    def curr_pose(self):
        return self.__curr_pose

    def simulation_status(self):
        """ Shows status of this simulation
        
        Returns
        -------
        bool
            Boolean telling whether the simulation has concluded or not.
        """

        if self.__simulation_ended:
            print("Simulation Ended")
        else:
            print("Simulation Running")
        return self.__simulation_ended

    def move(self):
        """ 
        Moves simulation with one time step    
        """

        if self.__simulation_ended:
            return

        if self.__next:
            self.__next = False
            self.__accumulated_dt = 0.0
            if self.__curr_waypoint_index < len(self._waypoints) - 1:
                self.__curr_waypoint_index += 1
            elif self._motion_profile.looper:
                self.__curr_waypoint_index = 1
            else:
                self.__simulation_ended = True
        
        time_val = self._waypoints[self.__curr_waypoint_index][4]

        src = (self._waypoints[self.__curr_waypoint_index - 1][0], self._waypoints[self.__curr_waypoint_index -1][1], self._waypoints[self.__curr_waypoint_index-1][2])
        dst = (self._waypoints[self.__curr_waypoint_index][0], self._waypoints[self.__curr_waypoint_index][1], self._waypoints[self.__curr_waypoint_index][2])

        if (self.__accumulated_dt <= (time_val)):
            ease_val = self._ease_function(self.__accumulated_dt / time_val)
            x, y, z = self.lerp(ease_val, [src, dst])
            self.__curr_pose = [x, y, z, self._waypoints[self.__curr_waypoint_index][3]]
            self.__accumulated_dt += self.__fixed_time_delta
        else:
            self.__next = True


    def lerp(self, dt, points):
        """ Linear interpolation between source and destination.

        Parameters
        ----------
        dt : float
            ease value between 0 and 1 i.e when 0 then interpolated value will be 
            source point and when its 1 then interpolated value will be destination point
        points : tuple
            A tuple (src, dst) where both src and dst are of shape (1, 3)
        
        Returns
        -------
        tuple
            A tuple of shape (1, 3) containing interpolated 3D position
        """

        dx = points[1][0] - points[0][0]
        dy = points[1][1] - points[0][1]
        dz = points[1][2] - points[0][2]

        return dt*dx + points[0][0], dt*dy + points[0][1], dt*dz + points[0][2]
        

class ScenarioSimulator:
    def __init__(self):
        
        # ----------------------------------------
        # General ros parameters of simulator node
        # ----------------------------------------
        
        self.path = rospy.get_param('~path', "/home/mahir/testing/custom_sim_scenarios/delta_roundabout/obstacle_stopping_bridge/congestion_scenario.json")

        self._default_motion_profile = MotionProfile(
            rospy.get_param("~replan", default=True),
            rospy.get_param('~looper', default=True),
            rospy.get_param("~velocity", default=11.0),
            rospy.get_param("~stop_time", default=10),
            rospy.get_param("~initial_delay", default=0),
            rospy.get_param("~tween_ease_function", default="linear")
        )

        scenario_config = None
        with open(self.path, 'r') as f:
            scenario_config = json.load(f)

        assert (scenario_config is not None), "Error: No Scenario configuration found"

        # ----------------
        # Node attributes
        # ----------------
        self._use_map_offsetting = rospy.get_param("~use_map_offsetting", default=False)
        self._map_offset_config = scenario_config["map_offset_configurations"] if  "map_offset_configurations" in scenario_config else None
        self._waypoint_simulators = self._initialize_simulators(scenario_config)
        self.rate = rospy.Rate(50)

        # -----------
        # Publishers
        # -----------
        self._simulated_objects_pub = rospy.Publisher(
            "simulated_objects", DetectedObjectArray, queue_size=1)

    def _initialize_simulators(self, scenario_config):
        """ Initializes simulators for each simulated obstacle in scenario_config

        Parameters
        ----------
        scenario_config : dict
            A dictionary of obstacle configurations extracted from scenario json file.
        
        Returns
        -------
        list
            A list of WaypointSimulator
        """
        
        type_dict = {'x': float, 'y': float, 'z': float, 'yaw': float, 'velocity': float, 'stop_flag': bool}
        csv_fields = {'x','y','z','yaw','velocity', 'stop_flag'}

        simulators = []

        for obs_idx, obstacle_config in enumerate(scenario_config['obstacles']):
            
            assert ("file" in obstacle_config), "Error: No file path for obstacle " + str(obs_idx)

            csv_path = os.path.join(os.path.dirname(self.path), obstacle_config["file"])

            waypoints = pd.read_csv(
                csv_path, delim_whitespace=False, dtype=type_dict, usecols=csv_fields).values
            
            motion_profile = self._extract_motion_profile(obstacle_config)
            replanned_waypoints = self._replan_motion(waypoints, motion_profile)

            simulators.append(WaypointSimulator(replanned_waypoints, motion_profile))

        return simulators


    def _extract_motion_profile(self, obstacle_config):
        """ Extracts motion profile w.r.t obstacle's configuration

        Parameters
        ----------
        obstacle_config : dict
            A dictionary of obstacle's configuration extracted from scenario json file
        
        Returns
        -------
        MotionProfile
            Motion profile of a simulated obstacle
        """

        return MotionProfile(
            obstacle_config['replan'] if 'replan' in obstacle_config else self._default_motion_profile.replan,
            obstacle_config['looper'] if 'looper' in obstacle_config else self._default_motion_profile.looper,
            obstacle_config['velocity'] if 'velocity' in obstacle_config else self._default_motion_profile.velocity,
            obstacle_config['stop_time'] if 'stop_time' in obstacle_config else self._default_motion_profile.stop_time,
            obstacle_config['initial_delay'] if 'initial_delay' in obstacle_config else self._default_motion_profile.initial_delay,
            obstacle_config['tween_ease_function'] if 'tween_ease_function' in obstacle_config else self._default_motion_profile.tween_ease_function
        )

    def _rotate_around_origin(self, origin, waypoint, yaw):
        """ Rotate a waypoint around origin with specified angle

        Parameters
        ----------
        origin : ndarray
            A numpy array of shape (2,) showing origin point
        waypoint : ndarray
            A numpy array of shape (2,) showing waypoint to be rotated
        yaw : float
            Rotation angle in Radians
        
        Returns
        -------
        tuple
            A tuple containing rotated waypoints around origin
        """
        
        x = origin[0] + math.cos(yaw) * (waypoint[0] - origin[0]) - math.sin(yaw) * (waypoint[1] - origin[1])
        y = origin[1] + math.sin(yaw) * (waypoint[0] - origin[0]) + math.cos(yaw) * (waypoint[1] - origin[1])

        return x, y
    

    def _replan_motion(self, waypoints, motion_profile):
        """ Replans obstacle's motion accordingly to motion profile.

        Parameters
        ----------
        waypoints : ndarray
            A numpy array of shape (n, 5) of waypoints.
        motion_profile : MotionProfile
            Motion profile for obstacle
        
        Returns
        -------
        ndarray
            An ndarray of waypoints of shape (n, 5)
        """
        
        # Extract stopping points from waypoints list
        stopping_points = np.where(waypoints[:,5].astype('bool'))[0]

        time_between_stops = []
        prev_stopping_point = 0
        for stop_point in stopping_points:
            # Extract time interval w.r.t velocity between stopping points
            time_between_stops.append((stop_point - prev_stopping_point) / motion_profile.velocity)
            prev_stopping_point = stop_point

        prev_dist = 0
        waypoints_updated = waypoints.copy()
        ease_function = getattr(pytweening, motion_profile.tween_ease_function)

        for idx, stop_dist in enumerate(stopping_points):

            dt = 0
            last_lerp = 0

            for dist_idx in range(prev_dist, stop_dist):
                
                # Ease value between current time and time to reach next stopping point
                # Note: ease_val will always be between 0 and 1
                ease_val = ease_function(dt / (time_between_stops[idx]))
                # lerp between 0 and time to next stopping point using ease value
                lerp_val = ease_val * time_between_stops[idx]

                # If ease_val is zero, then we are at a stopping point so we use stop time for this waypoint.
                if (ease_val) == 0:
                    waypoints_updated[dist_idx][4] = motion_profile.stop_time
                else:
                    # Set time value for current waypoint
                    waypoints_updated[dist_idx][4] = lerp_val - last_lerp

                # If map offsetting is enabled then offset waypoints according to configurations from json file
                if self._use_map_offsetting and self._map_offset_config is not None:
                    
                    origin_xy = np.array(self._map_offset_config['origin_xy'])
                    offset_xy = np.array(self._map_offset_config['offset_xy'])
                    yaw_rotation = self._map_offset_config['yaw_rotation']

                    translated_origin_xy = origin_xy + offset_xy

                    x = waypoints_updated[dist_idx][0] + offset_xy[0]
                    y = waypoints_updated[dist_idx][1] + offset_xy[1]                 

                    x, y = self._rotate_around_origin(translated_origin_xy, [x, y], yaw_rotation)
                    
                    waypoints_updated[dist_idx][0] = x
                    waypoints_updated[dist_idx][1] = y

                # Set current time delta i.e time = (distance/velocity).
                # Here, distance is set to 1 meter because the distance
                # between each waypoint is 1 meter.
                dt += (1.0 / motion_profile.velocity)

                # Update last_lerp value
                last_lerp = lerp_val
            
            prev_dist = stop_dist

        # If initial delay is required we set time value for initial waypoint
        if motion_profile.initial_delay > 0:
            waypoints_updated[1, 4] = motion_profile.initial_delay

        waypoints_updated[waypoints_updated.shape[0] - 1][4] = (1 / motion_profile.velocity)

        return waypoints_updated

    def _simulate_obstacles(self):
        """ 
        Runs simulation loop for all waypoint simulators.
        """

        while not rospy.is_shutdown():
            
            detected_objects_msg = DetectedObjectArray()
            detected_objects_msg.header.frame_id = "map"
            detected_objects_msg.header.stamp = rospy.Time.now()
            for simulator in self._waypoint_simulators:
                simulator.move()
                detected_objects_msg.objects.append(self._get_detected_object_msg(simulator.curr_pose))

            self._simulated_objects_pub.publish(detected_objects_msg)
            self.rate.sleep()

    def _get_detected_object_msg(self, pose):
        """ Populates a DetectedObject message w.r.t given pose

        Parameters
        ----------
        pose : list
            A list containing [x, y, z, yaw]
        
        Returns
        -------
        DetectedObject
            A DetectedObject type message
        """

        dimension = np.array([6, 1, 1])

        if self._use_map_offsetting and self._map_offset_config is not None:             
            x_dim, y_dim = self._rotate_around_origin([0, 0], [dimension[0], dimension[1]], self._map_offset_config['yaw_rotation'])
            dimension = np.array([x_dim, y_dim, dimension[2]])

        msg = DetectedObject()
        msg.pose.position = Vector3(pose[0], pose[1], pose[2])

        if self._use_map_offsetting and self._map_offset_config is not None: 
            rotation = quaternion_from_euler(0, 0, pose[3] + self._map_offset_config['yaw_rotation'])
        else:
            rotation = quaternion_from_euler(0, 0, pose[3])
        
        msg.pose.orientation = Quaternion(rotation[0], rotation[1], rotation[2], rotation[3])
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.dimensions = Vector3(dimension[0], dimension[1], dimension[2])
        msg.pose_reliable = True

        halved_dim = dimension/2
        rotation_matrix = np.array([[np.cos(pose[3]), -np.sin(pose[3]), 0],
                                    [np.sin(pose[3]), np.cos(pose[3]), 0], 
                                    [0, 0, 1]])

        # Rotate polygon points around origin
        rotated_vec_0 = rotation_matrix.dot(
            np.array([halved_dim[0], halved_dim[1], halved_dim[2]]))
        rotated_vec_1 = rotation_matrix.dot(
            np.array([-halved_dim[0], halved_dim[1], halved_dim[2]]))
        rotated_vec_2 = rotation_matrix.dot(
            np.array([-halved_dim[0], -halved_dim[1], halved_dim[2]]))
        rotated_vec_3 = rotation_matrix.dot(
            np.array([halved_dim[0], -halved_dim[1], halved_dim[2]]))
        rotated_vec_4 = rotation_matrix.dot(
            np.array([halved_dim[0], halved_dim[1], halved_dim[2]]))

        # Translate rotated polygon points to detected object's position and populate polygon points array.
        hull_points = [
            Point32(
                x=pose[0] + rotated_vec_0[0],
                y=pose[1] + rotated_vec_0[1],
                z=pose[2]
            ),
            Point32(
                x=pose[0] + rotated_vec_1[0],
                y=pose[1] + rotated_vec_1[1],
                z=pose[2]
            ),
            Point32(
                x=pose[0] + rotated_vec_2[0],
                y=pose[1] + rotated_vec_2[1],
                z=pose[2]
            ),
            Point32(
                x=pose[0] + rotated_vec_3[0],
                y=pose[1] + rotated_vec_3[1],
                z=pose[2]
            ),
            Point32(
                x=pose[0] + rotated_vec_4[0],
                y=pose[1] + rotated_vec_4[1],
                z=pose[2]
            ),
            Point32(
                x=pose[0] + 1,
                y=pose[1] + 1,
                z=pose[2]
            ),
            Point32(
                x=pose[0] - 1,
                y=pose[1] + 1,
                z=pose[2]
            ),
            Point32(
                x=pose[0] - 1,
                y=pose[1] - 1,
                z=pose[2]
            ),
            Point32(
                x=pose[0] + 1,
                y=pose[1] - 1,
                z=pose[2]
            )
        ]
        msg.convex_hull.header = msg.header
        msg.convex_hull.polygon.points = hull_points
        
        return msg

    def run(self):
        self._simulate_obstacles()
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('scnenario_simulator',
                    anonymous=True, log_level=rospy.INFO)
    node = ScenarioSimulator()
    node.run()

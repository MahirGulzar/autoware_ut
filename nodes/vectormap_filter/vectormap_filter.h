/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef COMMON_PLATFORM_VECTORMAP_FILTER
#define COMMON_PLATFORM_VECTORMAP_FILTER


#include <vector>

#include <ros/ros.h>
#include <ros/package.h>

#include <tf/transform_listener.h>

#include <vector_map/vector_map.h>

#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

#include "op_planner/RoadNetwork.h"
#include "op_utility/DataRW.h"

namespace VectorMapFilter
{

class VectorMapFilter
{
private:

  std::string filter_frame_;
  tf::TransformListener tf_listener_;
  tf::StampedTransform local2global_;
  tf::StampedTransform tracking_frame2lane_frame_;
  tf::StampedTransform lane_frame2tracking_frame_;

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_nh_;
  ros::Subscriber sub_detected_array_;
  ros::Publisher pub_object_array_;

  std_msgs::Header input_header_;

  // adding map filtering to immukf
  PlannerHNS::MAP_SOURCE_TYPE m_MapType;
  PlannerHNS::RoadNetwork m_Map;
  PlannerHNS::WayPoint m_CurrentPos;

  std::vector<PlannerHNS::Lane*> m_ClosestLanesList;
  ros::Subscriber sub_current_pose_;

  double filter_radius_;
  double centerline_filter_distance_;
  bool bEnable_lane_change_;
  bool bMap_;
  bool bNew_current_pos_;
  int filter_type_;

  void callback(const autoware_msgs::DetectedObjectArray& input);

  void transformPoseToGlobal(const autoware_msgs::DetectedObjectArray& input,
                             autoware_msgs::DetectedObjectArray& transformed_input);
  void transformPoseToLocal(autoware_msgs::DetectedObjectArray& detected_objects_output);

  geometry_msgs::Pose getTransformedPose(const geometry_msgs::Pose& in_pose,
                                                const tf::StampedTransform& tf_stamp);

  bool updateNecessaryTransform();


public:
  VectorMapFilter();
  void run();

    //Mapping Section
  UtilityHNS::MapRaw m_MapRaw;
  ros::Subscriber sub_bin_map;
  ros::Subscriber sub_lanes;
  ros::Subscriber sub_points;
  ros::Subscriber sub_dt_lanes;
  ros::Subscriber sub_intersect;
  ros::Subscriber sup_area;
  ros::Subscriber sub_lines;
  ros::Subscriber sub_stop_line;
  ros::Subscriber sub_signals;
  ros::Subscriber sub_vectors;
  ros::Subscriber sub_curbs;
  ros::Subscriber sub_edges;
  ros::Subscriber sub_way_areas;
  ros::Subscriber sub_cross_walk;
  ros::Subscriber sub_nodes;

  void callbackGetVMLanes(const vector_map_msgs::LaneArray& msg);
  void callbackGetVMPoints(const vector_map_msgs::PointArray& msg);
  void callbackGetVMdtLanes(const vector_map_msgs::DTLaneArray& msg);
  void callbackGetVMIntersections(const vector_map_msgs::CrossRoadArray& msg);
  void callbackGetVMAreas(const vector_map_msgs::AreaArray& msg);
  void callbackGetVMLines(const vector_map_msgs::LineArray& msg);
  void callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg);
  void callbackGetVMSignal(const vector_map_msgs::SignalArray& msg);
  void callbackGetVMVectors(const vector_map_msgs::VectorArray& msg);
  void callbackGetVMCurbs(const vector_map_msgs::CurbArray& msg);
  void callbackGetVMRoadEdges(const vector_map_msgs::RoadEdgeArray& msg);
  void callbackGetVMWayAreas(const vector_map_msgs::WayAreaArray& msg);
  void callbackGetVMCrossWalks(const vector_map_msgs::CrossWalkArray& msg);
  void callbackGetVMNodes(const vector_map_msgs::NodeArray& msg);

  void loadMap();
  void callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg);
  void filterObjects(const autoware_msgs::DetectedObjectArray& input_objects, autoware_msgs::DetectedObjectArray& filtered_objects);
  bool filterByMap(const autoware_msgs::DetectedObject& obj, const PlannerHNS::WayPoint& currState, PlannerHNS::RoadNetwork& map);
};

}  // namespace VectorMapFilter
#endif /* COMMON_PLATFORM_VECTORMAP_FILTER */

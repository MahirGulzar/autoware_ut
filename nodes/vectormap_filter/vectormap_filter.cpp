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


#include "vectormap_filter.h"
#include "op_planner/VectorMapLoader.h"
#include "op_planner/PlanningHelpers.h"

namespace VectorMapFilter
{

VectorMapFilter::VectorMapFilter()
  : private_nh_("~"),
  m_MapType(PlannerHNS::MAP_AUTOWARE),
  bMap_(false),
  bNew_current_pos_(false),
  filter_type_(2),
  filter_radius_(150),
  centerline_filter_distance_(1.5)
{
  private_nh_.param<bool>("enable_lane_change", bEnable_lane_change_, false);

  // params for filtering
  private_nh_.param<std::string>("filter_frame", filter_frame_, "map");
  private_nh_.param<int>("map_filter_type", filter_type_, 2);
  private_nh_.param<double>("filter_radius", filter_radius_, 150);
  private_nh_.param<double>("centerline_filter_distance", centerline_filter_distance_, 1.5);

  if(filter_type_ != 0)
  {
    sub_lanes = node_handle_.subscribe("/vector_map_info/lane", 1, &VectorMapFilter::callbackGetVMLanes,  this);
    sub_points = node_handle_.subscribe("/vector_map_info/point", 1, &VectorMapFilter::callbackGetVMPoints,  this);
    sub_dt_lanes = node_handle_.subscribe("/vector_map_info/dtlane", 1, &VectorMapFilter::callbackGetVMdtLanes,  this);
    sub_intersect = node_handle_.subscribe("/vector_map_info/cross_road", 1, &VectorMapFilter::callbackGetVMIntersections,  this);
    sup_area = node_handle_.subscribe("/vector_map_info/area", 1, &VectorMapFilter::callbackGetVMAreas,  this);
    sub_lines = node_handle_.subscribe("/vector_map_info/line", 1, &VectorMapFilter::callbackGetVMLines,  this);
    sub_stop_line = node_handle_.subscribe("/vector_map_info/stop_line", 1, &VectorMapFilter::callbackGetVMStopLines,  this);
    sub_signals = node_handle_.subscribe("/vector_map_info/signal", 1, &VectorMapFilter::callbackGetVMSignal,  this);
    sub_vectors = node_handle_.subscribe("/vector_map_info/vector", 1, &VectorMapFilter::callbackGetVMVectors,  this);
    sub_curbs = node_handle_.subscribe("/vector_map_info/curb", 1, &VectorMapFilter::callbackGetVMCurbs,  this);
    sub_edges = node_handle_.subscribe("/vector_map_info/road_edge", 1, &VectorMapFilter::callbackGetVMRoadEdges,  this);
    sub_way_areas = node_handle_.subscribe("/vector_map_info/way_area", 1, &VectorMapFilter::callbackGetVMWayAreas,  this);
    sub_cross_walk = node_handle_.subscribe("/vector_map_info/cross_walk", 1, &VectorMapFilter::callbackGetVMCrossWalks,  this);
    sub_nodes = node_handle_.subscribe("/vector_map_info/node", 1, &VectorMapFilter::callbackGetVMNodes,  this);
  }

}

void VectorMapFilter::run()
{
  pub_object_array_ = node_handle_.advertise<autoware_msgs::DetectedObjectArray>("filtered_objects", 1);
  sub_detected_array_ = node_handle_.subscribe("objects", 1, &VectorMapFilter::callback, this);
  sub_current_pose_ = node_handle_.subscribe("/current_pose",   1, &VectorMapFilter::callbackGetCurrentPose,  this);

}

void VectorMapFilter::callback(const autoware_msgs::DetectedObjectArray& input)
{
  input_header_ = input.header;


  bool success = updateNecessaryTransform();
  if (!success)
  {
    ROS_INFO("Could not find coordinate transformation");
    return;
  }

  autoware_msgs::DetectedObjectArray transformed_input;
  autoware_msgs::DetectedObjectArray detected_objects_output;
  autoware_msgs::DetectedObjectArray filtered_objects;
  transformPoseToGlobal(input, transformed_input);
  filterObjects(transformed_input, filtered_objects);
  transformPoseToLocal(filtered_objects);

  pub_object_array_.publish(filtered_objects);

}


bool VectorMapFilter::updateNecessaryTransform()
{
  bool success = true;
  try
  {
    tf_listener_.waitForTransform(input_header_.frame_id, filter_frame_, ros::Time(0), ros::Duration(1.0));
    tf_listener_.lookupTransform(filter_frame_, input_header_.frame_id, ros::Time(0), local2global_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    success = false;
  }
  return success;
}

void VectorMapFilter::transformPoseToGlobal(const autoware_msgs::DetectedObjectArray& input,
                                      autoware_msgs::DetectedObjectArray& transformed_input)
{
  transformed_input.header = input_header_;
  for (auto const &object: input.objects)
  {
    geometry_msgs::Pose out_pose = getTransformedPose(object.pose, local2global_);

    autoware_msgs::DetectedObject dd;
    dd.header = input.header;
    dd = object;
    dd.pose = out_pose;

    transformed_input.objects.push_back(dd);
  }
}

void VectorMapFilter::transformPoseToLocal(autoware_msgs::DetectedObjectArray& detected_objects_output)
{
  detected_objects_output.header = input_header_;

  tf::Transform inv_local2global = local2global_.inverse();
  tf::StampedTransform global2local;
  global2local.setData(inv_local2global);
  for (auto& object : detected_objects_output.objects)
  {
    geometry_msgs::Pose out_pose = getTransformedPose(object.pose, global2local);
    object.header = input_header_;
    object.pose = out_pose;
  }
}

geometry_msgs::Pose VectorMapFilter::getTransformedPose(const geometry_msgs::Pose& in_pose,
                                                  const tf::StampedTransform& tf_stamp)
{
  tf::Transform transform;
  geometry_msgs::PoseStamped out_pose;
  transform.setOrigin(tf::Vector3(in_pose.position.x, in_pose.position.y, in_pose.position.z));
  transform.setRotation(
      tf::Quaternion(in_pose.orientation.x, in_pose.orientation.y, in_pose.orientation.z, in_pose.orientation.w));
  geometry_msgs::PoseStamped pose_out;
  tf::poseTFToMsg(tf_stamp * transform, out_pose.pose);
  return out_pose.pose;
}


//Mapping Section
void VectorMapFilter::loadMap()
{

  if (m_MapType == PlannerHNS::MAP_AUTOWARE && !bMap_)
    {
      if(m_MapRaw.AreMessagesReceived())
      {
        PlannerHNS::VectorMapLoader vec_loader(1, bEnable_lane_change_, true, true, true);
        m_Map.Clear();
        vec_loader.LoadFromData(m_MapRaw, m_Map);
        PlannerHNS::MappingHelpers::ConvertVelocityToMeterPerSecond(m_Map);
        bMap_ = true;
      }
    }
}

void VectorMapFilter::callbackGetVMLanes(const vector_map_msgs::LaneArray& msg)
{
  std::cout << "Received Lanes imm_ukf_pda" << std::endl;
  if(m_MapRaw.pLanes == nullptr)
    m_MapRaw.pLanes = new UtilityHNS::AisanLanesFileReader(msg);
  loadMap();
}

void VectorMapFilter::callbackGetVMPoints(const vector_map_msgs::PointArray& msg)
{
  std::cout << "Received Points imm_ukf_pda" << std::endl;
  if(m_MapRaw.pPoints  == nullptr)
    m_MapRaw.pPoints = new UtilityHNS::AisanPointsFileReader(msg);
  loadMap();
}

void VectorMapFilter::callbackGetVMdtLanes(const vector_map_msgs::DTLaneArray& msg)
{
  std::cout << "Received dtLanes imm_ukf_pda" << std::endl;
  if(m_MapRaw.pCenterLines == nullptr)
    m_MapRaw.pCenterLines = new UtilityHNS::AisanCenterLinesFileReader(msg);
  loadMap();
}

void VectorMapFilter::callbackGetVMIntersections(const vector_map_msgs::CrossRoadArray& msg)
{
  std::cout << "Received CrossRoads imm_ukf_pda" << std::endl;
  if(m_MapRaw.pIntersections == nullptr)
    m_MapRaw.pIntersections = new UtilityHNS::AisanIntersectionFileReader(msg);
  loadMap();
}

void VectorMapFilter::callbackGetVMAreas(const vector_map_msgs::AreaArray& msg)
{
  std::cout << "Received Areas imm_ukf_pda" << std::endl;
  if(m_MapRaw.pAreas == nullptr)
    m_MapRaw.pAreas = new UtilityHNS::AisanAreasFileReader(msg);
  loadMap();
}

void VectorMapFilter::callbackGetVMLines(const vector_map_msgs::LineArray& msg)
{
  std::cout << "Received Lines imm_ukf_pda" << std::endl;
  if(m_MapRaw.pLines == nullptr)
    m_MapRaw.pLines = new UtilityHNS::AisanLinesFileReader(msg);
  loadMap();
}

void VectorMapFilter::callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg)
{
  std::cout << "Received StopLines imm_ukf_pda" << std::endl;
  if(m_MapRaw.pStopLines == nullptr)
    m_MapRaw.pStopLines = new UtilityHNS::AisanStopLineFileReader(msg);
  loadMap();
}

void VectorMapFilter::callbackGetVMSignal(const vector_map_msgs::SignalArray& msg)
{
  std::cout << "Received Signals imm_ukf_pda" << std::endl;
  if(m_MapRaw.pSignals  == nullptr)
    m_MapRaw.pSignals = new UtilityHNS::AisanSignalFileReader(msg);
  loadMap();
}

void VectorMapFilter::callbackGetVMVectors(const vector_map_msgs::VectorArray& msg)
{
  std::cout << "Received Vectors imm_ukf_pda" << std::endl;
  if(m_MapRaw.pVectors  == nullptr)
    m_MapRaw.pVectors = new UtilityHNS::AisanVectorFileReader(msg);
  loadMap();
}

void VectorMapFilter::callbackGetVMCurbs(const vector_map_msgs::CurbArray& msg)
{
  std::cout << "Received Curbs imm_ukf_pda" << std::endl;
  if(m_MapRaw.pCurbs == nullptr)
    m_MapRaw.pCurbs = new UtilityHNS::AisanCurbFileReader(msg);
  loadMap();
}

void VectorMapFilter::callbackGetVMRoadEdges(const vector_map_msgs::RoadEdgeArray& msg)
{
  std::cout << "Received Edges imm_ukf_pda" << std::endl;
  if(m_MapRaw.pRoadedges  == nullptr)
    m_MapRaw.pRoadedges = new UtilityHNS::AisanRoadEdgeFileReader(msg);
  loadMap();
}

void VectorMapFilter::callbackGetVMWayAreas(const vector_map_msgs::WayAreaArray& msg)
{
  std::cout << "Received Wayareas imm_ukf_pda" << std::endl;
  if(m_MapRaw.pWayAreas  == nullptr)
    m_MapRaw.pWayAreas = new UtilityHNS::AisanWayareaFileReader(msg);
  loadMap();
}

void VectorMapFilter::callbackGetVMCrossWalks(const vector_map_msgs::CrossWalkArray& msg)
{
  std::cout << "Received CrossWalks imm_ukf_pda" << std::endl;
  if(m_MapRaw.pCrossWalks == nullptr)
    m_MapRaw.pCrossWalks = new UtilityHNS::AisanCrossWalkFileReader(msg);
  loadMap();
}

void VectorMapFilter::callbackGetVMNodes(const vector_map_msgs::NodeArray& msg)
{
  std::cout << "Received Nodes imm_ukf_pda" << std::endl;
  if(m_MapRaw.pNodes == nullptr)
    m_MapRaw.pNodes = new UtilityHNS::AisanNodesFileReader(msg);
  loadMap();
}

void VectorMapFilter::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
  m_CurrentPos.pos = PlannerHNS::GPSPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, tf::getYaw(msg->pose.orientation));
  bNew_current_pos_ = true;
}

void VectorMapFilter::filterObjects(const autoware_msgs::DetectedObjectArray& input_objects, autoware_msgs::DetectedObjectArray& filtered_objects)
{

  if(bMap_)
  {
    m_ClosestLanesList = PlannerHNS::MappingHelpers::GetClosestLanesFast(m_CurrentPos, m_Map, filter_radius_);
  }

  for(unsigned int i=0; i < input_objects.objects.size(); i++)
  {
    autoware_msgs::DetectedObject autoware_obj;
    autoware_obj = input_objects.objects.at(i);

    if(!filterByMap(autoware_obj, m_CurrentPos, m_Map)) continue;

    filtered_objects.objects.push_back(autoware_obj);
  }

  filtered_objects.header = input_objects.header;
}


bool VectorMapFilter::filterByMap(const autoware_msgs::DetectedObject&  obj, const PlannerHNS::WayPoint& currState, PlannerHNS::RoadNetwork& map)
{
  if(!bMap_ || filter_type_ == 0) return true;

  if(filter_type_ == 1)
  {
    for(unsigned int ib = 0; ib < map.boundaries.size(); ib++)
    {
      double d_to_center = hypot(map.boundaries.at(ib).center.pos.y - currState.pos.y, map.boundaries.at(ib).center.pos.x - currState.pos.x);
      if(d_to_center < filter_radius_*2.0)
      {
        PlannerHNS::WayPoint object_center_position(obj.pose.position.x, obj.pose.position.y, obj.pose.position.z, 0);
        if(PlannerHNS::PlanningHelpers::PointInsidePolygon(map.boundaries.at(ib).points, object_center_position) == 1)
        {
          return true;
        }
      }
    }
  }
  else if(filter_type_ == 2)
  {
    for(unsigned int i =0 ; i < m_ClosestLanesList.size(); i++)
    {
      PlannerHNS::RelativeInfo info;
      PlannerHNS::WayPoint object_center_position(obj.pose.position.x, obj.pose.position.y, obj.pose.position.z, 0);
      PlannerHNS::PlanningHelpers::GetRelativeInfoLimited(m_ClosestLanesList.at(i)->points, object_center_position, info);
      if(info.iFront >= 0  &&  info.iFront < m_ClosestLanesList.at(i)->points.size())
      {

        PlannerHNS::WayPoint wp = m_ClosestLanesList.at(i)->points.at(info.iFront);

        double direct_d = hypot(wp.pos.y - object_center_position.pos.y, wp.pos.x - object_center_position.pos.x);

        if((info.bAfter || info.bBefore) && direct_d > centerline_filter_distance_*2.0)
        {
          continue;
        }
      }

      if(fabs(info.perp_distance) <= centerline_filter_distance_)
      {
        return true;
      }
    }
  }

  return false;
}

} //namespace VectorMapFilter

int main(int argc, char **argv) {
    ros::init(argc, argv, "vectormap_filter");
    VectorMapFilter::VectorMapFilter vectormap_filter;
    vectormap_filter.run();

    ros::spin();
    return 0;
}
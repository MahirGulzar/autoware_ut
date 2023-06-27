#include <libvectormap/vector_map.h>
#include <autoware_msgs/Signals.h>
#include <string> 
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

static VectorMap vmap;
static ros::Publisher marker_publisher;
static constexpr float LIGHT_SIZE = 2;
static constexpr uint32_t SUBSCRIBE_QUEUE_SIZE = 1000;
static visualization_msgs::Marker signal_marker;
static visualization_msgs::Marker text_marker;
static std_msgs::ColorRGBA color_green;
static std_msgs::ColorRGBA color_yellow;
static std_msgs::ColorRGBA color_red;
static std_msgs::ColorRGBA color_black;
static bool debug_link_ids = false;


static autoware_msgs::Signals ExtractLocations(autoware_msgs::Signals& signals)
{
  // create new container to gather tfls for visualization
  autoware_msgs::Signals tfl_viz;

  for (auto& signal : signals.Signals)
  {
    //signal is extractedPosition
    
    for (const auto& tfl_map : vmap.signals){
      const Signal tfl = tfl_map.second;

      // for each signal get trafficlight using linkId and stopline based signal
      if(tfl.linkid == signal.linkId){

        autoware_msgs::ExtractedPosition ep;
       
        int pid = vmap.vectors[tfl.vid].pid;
        Point3 signalcenter = vmap.getPoint(pid);

        ep.signalId = tfl.id;
        ep.linkId = tfl.linkid;
        ep.type = signal.type;
        ep.x = signalcenter.x();
        ep.y = signalcenter.y();
        ep.z = signalcenter.z();
        
        tfl_viz.Signals.push_back(ep);
      }
    }
  }
  return tfl_viz;
}

static void PublishMarkerArray(autoware_msgs::Signals& signals)
{
  visualization_msgs::MarkerArray signal_set;
  int id = 1000;
  for (auto& signal : signals.Signals)
  {
    if((signal.signalId - 2) % 10 != 0) // Only read IDs xx2 to represent single traffic light with single marker
      continue;
    signal_marker.pose.position.x = signal.x;
    signal_marker.pose.position.y = signal.y;
    signal_marker.pose.position.z = signal.z;
    signal_marker.id = signal.signalId;

    text_marker.pose = signal_marker.pose;
    text_marker.text = std::to_string(signal.linkId);
    text_marker.id = id;
    
    id++;

    if(signal.type == 2) // RED
    {
      signal_marker.color = color_red;
      text_marker.color = color_red;
    }
    else if(signal.type == 0) // GREEN
    {
      signal_marker.color = color_green;
      text_marker.color = color_green;
    }
    else // UNKNOWN
    {
      signal_marker.color = color_yellow;
      text_marker.color = color_yellow;
    }
    
    signal_set.markers.push_back(signal_marker);
    if (debug_link_ids)
      signal_set.markers.push_back(text_marker);
  }
  marker_publisher.publish(signal_set);
}

static void ROISignalCallback(const autoware_msgs::Signals::ConstPtr &extracted_pos){
  autoware_msgs::Signals signals_lamps = *extracted_pos;
  autoware_msgs::Signals tfl_viz = ExtractLocations(signals_lamps);
  PublishMarkerArray(tfl_viz);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "tl_visualizer", ros::init_options::NoSigintHandler);
  ros::NodeHandle rosnode;
  ros::NodeHandle private_nh("~");
  std::string roi_topic_name;
  private_nh.param<std::string>("roi_topic_name", roi_topic_name, "/roi_signal");
  private_nh.param<bool>("debug_link_ids", debug_link_ids, false);
  ros::Subscriber roi_signal_subscriber = rosnode.subscribe(roi_topic_name, 1, ROISignalCallback);
  marker_publisher = rosnode.advertise<visualization_msgs::MarkerArray>("tlr_result_merged", 1, true);

  double marker_duration = 2.0;
  signal_marker.ns = "tl_result";
  signal_marker.header.frame_id = "map";
  signal_marker.type = visualization_msgs::Marker::SPHERE;
  signal_marker.pose.orientation.x = 0.0;
  signal_marker.pose.orientation.y = 0.0;
  signal_marker.pose.orientation.z = 0.0;
  signal_marker.pose.orientation.w = 0.0;
  signal_marker.scale.x = LIGHT_SIZE;
  signal_marker.scale.y = LIGHT_SIZE;
  signal_marker.scale.z = LIGHT_SIZE;
  signal_marker.lifetime = ros::Duration(marker_duration);

  text_marker.ns = "tl_result";
  text_marker.header.frame_id = "map";
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.pose.orientation.x = 0.0;
  text_marker.pose.orientation.y = 0.0;
  text_marker.pose.orientation.z = 0.0;
  text_marker.pose.orientation.w = 0.0;
  text_marker.scale.x = 6;
  text_marker.scale.y = 6;
  text_marker.scale.z = 6;
  text_marker.lifetime = ros::Duration(marker_duration);

  // Define color constants
  color_black.r = 0.0f;
  color_black.g = 0.0f;
  color_black.b = 0.0f;
  color_black.a = 1.0f;

  color_red.r = 1.0f;
  color_red.g = 0.0f;
  color_red.b = 0.0f;
  color_red.a = 1.0f;

  color_yellow.r = 1.0f;
  color_yellow.g = 1.0f;
  color_yellow.b = 0.0f;
  color_yellow.a = 1.0f;

  color_green.r = 0.0f;
  color_green.g = 1.0f;
  color_green.b = 0.0f;
  color_green.a = 1.0f;
 
  /* load vector map */
  ros::Subscriber sub_point =
      rosnode.subscribe("vector_map_info/point", SUBSCRIBE_QUEUE_SIZE, &VectorMap::load_points, &vmap);
  ros::Subscriber sub_line =
      rosnode.subscribe("vector_map_info/line", SUBSCRIBE_QUEUE_SIZE, &VectorMap::load_lines, &vmap);
  ros::Subscriber sub_lane =
      rosnode.subscribe("vector_map_info/lane", SUBSCRIBE_QUEUE_SIZE, &VectorMap::load_lanes, &vmap);
  ros::Subscriber sub_vector =
      rosnode.subscribe("vector_map_info/vector", SUBSCRIBE_QUEUE_SIZE, &VectorMap::load_vectors, &vmap);
  ros::Subscriber sub_signal =
      rosnode.subscribe("vector_map_info/signal", SUBSCRIBE_QUEUE_SIZE, &VectorMap::load_signals, &vmap);

  /* wait until loading all vector map is completed */
  ros::Rate wait_rate(10);
  while (vmap.points.empty() || vmap.lines.empty() || vmap.lanes.empty() || vmap.vectors.empty()
    || vmap.signals.empty())
  {
    ros::spinOnce();
    ROS_INFO_THROTTLE(2, "[tl_visualizer] Waiting for Vector Map");
    wait_rate.sleep();
  }

  vmap.loaded = true;
  ROS_INFO("tl_visualizer: Vector Map loaded.");

  ros::Publisher signalPublisher = rosnode.advertise<autoware_msgs::Signals>(roi_topic_name, 100);

  ros::spin();
}

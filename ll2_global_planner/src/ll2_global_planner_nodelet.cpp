/*
* Unpublished Copyright (c) 2009-2020 AutonomouStuff, All Rights Reserved.
*
* This file is part of the ll2_global_planner which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include "ll2_global_planner/ll2_global_planner_nodelet.hpp"

#include <string>
#include <vector>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <autoware_msgs/LaneArray.h>

#include <lanelet2_core/primitives/GPSPoint.h>
#include <lanelet2_extension/projection/mgrs_projector.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

using namespace lanelet;

namespace ll2_global_planner
{

Ll2GlobalPlannerNl::Ll2GlobalPlannerNl() :
  tf_listener_(tf_buffer_)
{}

void Ll2GlobalPlannerNl::onInit()
{
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();
  loadParams();

  // Publishers
  waypoints_pub_ = nh_.advertise<autoware_msgs::LaneArray>("based/lane_waypoints_raw", 1, true);

  // Subscribers
  lanelet_sub_ = nh_.subscribe("lanelet_map_bin", 1, &Ll2GlobalPlannerNl::laneletMapCb, this);
  posegoal_sub_ = nh_.subscribe("move_base_simple/goal", 1, &Ll2GlobalPlannerNl::poseGoalCb, this);
  llh_sub_ = pnh_.subscribe("llh_goal", 1, &Ll2GlobalPlannerNl::llhGoalCb, this);
}

void Ll2GlobalPlannerNl::loadParams()
{
    ROS_INFO("Parameters Loaded");
}

void Ll2GlobalPlannerNl::laneletMapCb(const autoware_lanelet2_msgs::MapBin& map_msg)
{
  lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(map_msg, lanelet_map_);

  traffic_rules_ = traffic_rules::TrafficRulesFactory::instance().create(Locations::Germany, Participants::Vehicle);
  routing_graph_ = routing::RoutingGraph::build(*lanelet_map_, *traffic_rules_);

  initialized_ = true;
  ROS_INFO("Loaded Lanelet map");
}

void Ll2GlobalPlannerNl::poseGoalCb(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
  if (!initialized_)
  {
    ROS_WARN("Waiting for lanelet2 map");
    return;
  }

  BasicPoint2d goal_point(pose_msg->pose.position.x, pose_msg->pose.position.y);
  planRoute(goal_point);
}

void Ll2GlobalPlannerNl::llhGoalCb(const sensor_msgs::NavSatFix::ConstPtr& llh_msg)
{
  GPSPoint gps_point;
  gps_point.lat = llh_msg->latitude;
  gps_point.lon = llh_msg->longitude;
  gps_point.ele = llh_msg->altitude;
  projection::MGRSProjector projector;
  BasicPoint3d goal_point_3d = projector.forward(gps_point);
  BasicPoint2d goal_point(goal_point_3d.x(), goal_point_3d.y());
  planRoute(goal_point);
}

void Ll2GlobalPlannerNl::planRoute(const BasicPoint2d& goal_point)
{
  // Find the nearest lanelet to the goal point
  Lanelet goal_lanelet = getNearestLanelet(goal_point);

  // Get the current vehicle position
  geometry_msgs::TransformStamped tf_msg;
  try
  {
    tf_msg = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(2.0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_THROTTLE(2, "%s", ex.what());
    ROS_WARN_THROTTLE(2, "Waiting for map -> base_link transform");
    return;
  }

  BasicPoint2d starting_point(
    tf_msg.transform.translation.x, tf_msg.transform.translation.y);
  const Lanelet& starting_lanelet = getNearestLanelet(starting_point);

  // Plan a route from current vehicle position
  Optional<routing::LaneletPath> shortest_path_opt = routing_graph_->shortestPath(starting_lanelet, goal_lanelet);
  routing::LaneletPath shortest_path;

  if (!shortest_path_opt)
  {
    ROS_WARN("Could not find path in lanelet map!");
    return;
  }

  shortest_path = shortest_path_opt.value();
  LaneletSequence continuous_lane = shortest_path.getRemainingLane(shortest_path.begin());

  if (continuous_lane.size() != shortest_path.size())
  {
    ROS_WARN("This route contains a lane change which is currently unsupported");
    return;
  }

  ROS_INFO("Found a path containing %lu lanelets", shortest_path.size());

  autoware_msgs::Lane lane_msg;
  lane_msg.waypoints = generateAutowareWaypoints(continuous_lane, goal_point);
  lane_msg.header.stamp = ros::Time(0);
  lane_msg.header.frame_id = "map";
  lane_msg.is_blocked = false;

  autoware_msgs::LaneArray lane_array_msg;
  lane_array_msg.lanes.push_back(lane_msg);
  waypoints_pub_.publish(lane_array_msg);
}

std::vector<autoware_msgs::Waypoint> Ll2GlobalPlannerNl::generateAutowareWaypoints(
  const LaneletSequence& continuous_lane, const BasicPoint2d& goal_point)
{
  const float duplicate_dist_threshold = 0.1;
  std::vector<autoware_msgs::Waypoint> waypoints;

  // Loop over each lanelet
  for (auto& lanelet : continuous_lane.lanelets())
  {
    const std::string turn_direction = lanelet.attributeOr("turn_direction", "straight");
    const traffic_rules::SpeedLimitInformation speed_limit = traffic_rules_->speedLimit(lanelet);

    // centerline() will return a series of points spaced 1 meter apart thanks
    // to Autoware.AI's lanelet2_map_loader. The loader rewrites the original
    // centerline with a resampled centerline for easier use with autoware.
    const ConstLineString3d centerline = lanelet.centerline();
    const int wp_length = centerline.size() - 1;

    // Loop over each centerline point
    for (int i = 0; i <= wp_length; i++)
    {
      auto point = centerline[i];

      autoware_msgs::Waypoint new_wp;
      new_wp.pose.pose.position.x = point.x();
      new_wp.pose.pose.position.y = point.y();
      new_wp.pose.pose.position.z = point.z();
      new_wp.twist.twist.linear.x = speed_limit.speedLimit.value(); // m/s

      int steering_state = autoware_msgs::WaypointState::STR_STRAIGHT;
      if (turn_direction.compare("right") == 0)
      {
        steering_state = autoware_msgs::WaypointState::STR_RIGHT;
      }
      if (turn_direction.compare("left") == 0)
      {
        steering_state = autoware_msgs::WaypointState::STR_LEFT;
      }
      new_wp.wpstate.steering_state = steering_state;

      waypoints.push_back(new_wp);
    }
  }

  int wp_id = 0;
  bool first_point = true;
  float smallest_goal_dist = std::numeric_limits<float>::infinity();
  int smallest_goal_wp_id = 0;
  autoware_msgs::Waypoint* prev_waypoint = nullptr;
  std::vector<autoware_msgs::Waypoint> processed_waypoints;

  // Remove duplicates and set waypoint orientations
  for (autoware_msgs::Waypoint& waypoint : waypoints)
  {
    geometry_msgs::Pose& pose = waypoint.pose.pose;
    geometry_msgs::Pose& prev_pose = prev_waypoint->pose.pose;

    if (!first_point)
    {
      // Check for duplicate points
      float dist = std::hypot(pose.position.x - prev_pose.position.x, pose.position.y - prev_pose.position.y);
      if (dist < duplicate_dist_threshold)
      {
        continue;
      }

      // Set orientation of the previous point, aiming at current point
      float yaw = std::atan2(pose.position.y - prev_pose.position.y, pose.position.x - prev_pose.position.x);

      tf2::Quaternion orientation;
      orientation.setRPY(0, 0, yaw);
      tf2::convert(orientation, prev_pose.orientation);
    }
    else
    {
      first_point = false;
    }

    // Calculate distance to goal point
    float goal_dist = std::hypot(goal_point.x() - pose.position.x, goal_point.y() - pose.position.y);
    if (goal_dist < smallest_goal_dist)
    {
      smallest_goal_dist = goal_dist;
      smallest_goal_wp_id = wp_id;
    }

    waypoint.gid = wp_id;
    processed_waypoints.push_back(waypoint);
    prev_waypoint = &processed_waypoints.back();
    wp_id++;
  }

  // Set orientation of the last waypoint
  processed_waypoints.back().pose.pose.orientation = processed_waypoints.end()[-2].pose.pose.orientation;

  // Trim waypoints to stop at goal point
  processed_waypoints.resize(smallest_goal_wp_id);

  return processed_waypoints;
}

Lanelet Ll2GlobalPlannerNl::getNearestLanelet(const lanelet::BasicPoint2d& point)
{
  std::vector<std::pair<double, Lanelet>> closeLanelets = geometry::findNearest(lanelet_map_->laneletLayer, point, 1);

  if (closeLanelets.size() > 1)
  {
    ROS_WARN("Vehicle is positioned inside multiple lanelets, choosing the first lanelet as the starting point");
  }

  Lanelet nearest_lanelet = closeLanelets[0].second;

  return nearest_lanelet;
}

}  // namespace ll2_global_planner

PLUGINLIB_EXPORT_CLASS(ll2_global_planner::Ll2GlobalPlannerNl, nodelet::Nodelet);

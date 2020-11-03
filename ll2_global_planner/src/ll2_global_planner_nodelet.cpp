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

using namespace lanelet;

namespace ll2_global_planner
{

Ll2GlobalPlannerNl::Ll2GlobalPlannerNl() :
  tf_listener(this->tf_buffer)
{}

void Ll2GlobalPlannerNl::onInit()
{
  this->nh = getNodeHandle();
  this->pnh = getPrivateNodeHandle();
  this->loadParams();

  // Publishers
  this->waypoints_pub = nh.advertise<autoware_msgs::LaneArray>("based/lane_waypoints_raw", 1, true);

  // Subscribers
  this->lanelet_sub = nh.subscribe("lanelet_map_bin", 1, &Ll2GlobalPlannerNl::laneletMapCb, this);
  this->posegoal_sub = nh.subscribe("move_base_simple/goal", 1, &Ll2GlobalPlannerNl::poseGoalCb, this);
}

void Ll2GlobalPlannerNl::loadParams()
{
    ROS_INFO("Parameters Loaded");
}

void Ll2GlobalPlannerNl::laneletMapCb(const autoware_lanelet2_msgs::MapBin& map_msg)
{
}

void Ll2GlobalPlannerNl::poseGoalCb(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
}

}  // namespace ll2_global_planner

PLUGINLIB_EXPORT_CLASS(ll2_global_planner::Ll2GlobalPlannerNl, nodelet::Nodelet);

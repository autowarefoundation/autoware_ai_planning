/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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
#include <ros/ros.h>
#include <autoware_msgs/LaneArray.h>
#include "waypoint_replanner.h"

namespace waypoint_maker
{
class WaypointReplannerNode
{
public:
  WaypointReplannerNode();
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher lane_pub_;
  ros::Subscriber lane_sub_, config_sub_;
  bool replanning_mode_, realtime_tuning_mode_;
  bool is_first_publish_;
  WaypointReplanner replanner_;
  autoware_msgs::LaneArray lane_array_;
  void replan(autoware_msgs::LaneArray &lane_array);
  void publishLaneArray();
  void laneCallback(const autoware_msgs::LaneArray::ConstPtr& lane_array);
  void configCallback(const autoware_config_msgs::ConfigWaypointReplanner::ConstPtr& conf);
  autoware_config_msgs::ConfigWaypointReplanner startup_config;
  bool use_decision_maker_;
};

WaypointReplannerNode::WaypointReplannerNode() : pnh_("~"), is_first_publish_(true)
{
  WaypointReplannerConfig temp_config;

  double velocity_max_kph, velocity_min_kph;

  pnh_.param<bool>("replanning_mode", replanning_mode_, false);
  pnh_.param<bool>("realtime_tuning_mode", realtime_tuning_mode_, true);
  pnh_.param<double>("velocity_max", velocity_max_kph, 0.0);
  pnh_.param<double>("velocity_min", velocity_min_kph, 0.0);
  pnh_.param<double>("accel_limit", temp_config.accel_limit, 0.0);
  pnh_.param<double>("decel_limit", temp_config.decel_limit, 0.0);
  pnh_.param<double>("radius_thresh", temp_config.radius_thresh, 0.0);
  pnh_.param<double>("radius_min", temp_config.radius_min, 0.0);
  pnh_.param<bool>("resample_mode", temp_config.resample_mode, false);
  pnh_.param<double>("resample_interval", temp_config.resample_interval, 0.0);
  pnh_.param<bool>("replan_curve_mode", temp_config.replan_curve_mode, false);
  pnh_.param<bool>("replan_endpoint_mode", temp_config.replan_endpoint_mode, false);
  pnh_.param<bool>("overwrite_vmax_mode", temp_config.overwrite_vmax_mode, false);
  pnh_.param<double>("velocity_offset", temp_config.velocity_offset, 0.0);
  pnh_.param<double>("end_point_offset", temp_config.end_point_offset, 0.0);
  pnh_.param<double>("braking_distance", temp_config.braking_distance, 0.0);
  pnh_.param<bool>("use_decision_maker", use_decision_maker_, false);

  temp_config.velocity_max = kmph2mps(velocity_max_kph);
  temp_config.velocity_min = kmph2mps(velocity_min_kph);

  replanner_.updateConfig(temp_config);

  if (use_decision_maker_)
  {
    lane_pub_ = nh_.advertise<autoware_msgs::LaneArray>("/based/lane_waypoints_array", 10, true);
  }
  else
  {
    lane_pub_ = nh_.advertise<autoware_msgs::LaneArray>("/lane_waypoints_array", 10, true);
  }

  lane_sub_ = nh_.subscribe("/based/lane_waypoints_raw", 1, &WaypointReplannerNode::laneCallback, this);
  config_sub_ = nh_.subscribe("/config/waypoint_replanner", 1, &WaypointReplannerNode::configCallback, this);
}

void WaypointReplannerNode::replan(autoware_msgs::LaneArray& lane_array)
{
  for (auto &el : lane_array.lanes)
  {
    replanner_.replanLaneWaypointVel(el);
  }
}

void WaypointReplannerNode::publishLaneArray()
{
  autoware_msgs::LaneArray array(lane_array_);

  if (replanning_mode_)
  {
    replan(array);
  }

  lane_pub_.publish(array);
  is_first_publish_ = false;
}

void WaypointReplannerNode::laneCallback(const autoware_msgs::LaneArray::ConstPtr& lane_array)
{
  lane_array_ = *lane_array;
  publishLaneArray();
}

void WaypointReplannerNode::configCallback(const autoware_config_msgs::ConfigWaypointReplanner::ConstPtr& conf)
{
  replanning_mode_ = conf->replanning_mode;
  realtime_tuning_mode_ = conf->realtime_tuning_mode;
  use_decision_maker_ = conf->use_decision_maker;
  replanner_.initParameter(conf);
  if (!lane_array_.lanes.empty() && (is_first_publish_ || realtime_tuning_mode_))
  {
    publishLaneArray();
  }
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_replanner");
  waypoint_maker::WaypointReplannerNode wr;
  ros::spin();

  return 0;
}

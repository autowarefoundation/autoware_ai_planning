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

#ifndef ASTAR_AVOID_H
#define ASTAR_AVOID_H

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/LaneArray.h>

#include "libwaypoint_follower/libwaypoint_follower.h"
#include "astar_search/astar_search.h"

class AstarAvoid
{
public:
  typedef enum STATE
  {
    INITIALIZING = -1,
    RELAYING = 0,
    STOPPING = 1,
    PLANNING = 2,
    AVOIDING = 3
  } State;

  AstarAvoid();
  ~AstarAvoid() = default;
  void run();

private:
  // ros
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher safety_waypoints_pub_;
  ros::Subscriber costmap_sub_;
  ros::Subscriber current_pose_sub_;
  ros::Subscriber current_velocity_sub_;
  ros::Subscriber base_waypoints_sub_;
  ros::Subscriber closest_waypoint_sub_;
  ros::Subscriber obstacle_waypoint_sub_;
  ros::Subscriber state_sub_;
  ros::Rate *rate_;
  ros::Timer timer_;
  tf::TransformListener tf_listener_;

  // params
  int safety_waypoints_size_;   // output waypoint size [-]
  double update_rate_;          // publishing rate [Hz]

  bool enable_avoidance_;           // enable avoidance mode
  double avoid_waypoints_velocity_; // constant velocity on planned waypoints [km/h]
  double avoid_start_velocity_;     // self velocity for staring avoidance behavior [km/h]
  double replan_interval_;          // replan interval for avoidance planning [Hz]
  int search_waypoints_size_;       // range of waypoints for incremental search [-]
  int search_waypoints_delta_;      // skipped waypoints for incremental search [-]
  int closest_search_size_;         // search closest waypoint around your car [-]

  // classes
  AstarSearch astar_;
  State state_;

  // variables
  bool found_avoid_path_;

  // Index of the closest waypoint in the current_waypoints_ Lane.
  // Not the same as the waypoint gid. This value can change suddenly if the
  // current_waypoints_ switches between base_waypoints_ and avoid_waypoints_.
  int closest_waypoint_index_ = -1;

  // Index of the obstacle relative to closest_waypoint_index_.
  int obstacle_waypoint_index_ = -1;
  nav_msgs::OccupancyGrid costmap_;
  autoware_msgs::Lane base_waypoints_;
  autoware_msgs::Lane avoid_waypoints_;
  autoware_msgs::Lane current_waypoints_;
  geometry_msgs::PoseStamped current_pose_local_, current_pose_global_;
  geometry_msgs::PoseStamped goal_pose_local_, goal_pose_global_;
  geometry_msgs::TwistStamped current_velocity_;
  tf::Transform local2costmap_;  // local frame (e.g. velodyne) -> costmap origin

  bool costmap_initialized_ = false;
  bool current_pose_initialized_ = false;
  bool current_velocity_initialized_ = false;
  bool base_waypoints_initialized_ = false;
  bool closest_waypoint_initialized_ = false;

  // functions, callback
  void costmapCallback(const nav_msgs::OccupancyGrid& msg);
  void currentPoseCallback(const geometry_msgs::PoseStamped& msg);
  void currentVelocityCallback(const geometry_msgs::TwistStamped& msg);
  void baseWaypointsCallback(const autoware_msgs::Lane& msg);
  void closestWaypointCallback(const std_msgs::Int32& msg);
  void obstacleWaypointCallback(const std_msgs::Int32& msg);

  // functions
  bool checkInitialized();
  bool planAvoidWaypoints(int& end_of_avoid_index);
  void mergeAvoidWaypoints(const nav_msgs::Path& path, int& end_of_avoid_index);
  tf::Transform getTransform(const std::string& from, const std::string& to);

  // Find closest waypoint index within a search_size around the previous closest waypoint
  void updateClosestWaypoint(const autoware_msgs::Lane& waypoints, const geometry_msgs::Pose& pose, const int& search_size);
  // publish safety waypoints using a timer
  void publishWaypoints(const ros::TimerEvent& e);
};

#endif

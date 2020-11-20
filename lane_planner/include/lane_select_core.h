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

#ifndef LANE_SELECT_CORE_H
#define LANE_SELECT_CORE_H

// ROS includes
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// C++ includes
#include <iostream>
#include <numeric>
#include <tuple>

// User defined includes
#include "autoware_config_msgs/ConfigLaneSelect.h"
#include "autoware_msgs/LaneArray.h"
#include "autoware_msgs/State.h"
#include "autoware_msgs/VehicleLocation.h"
#include "hermite_curve.h"
#include "libwaypoint_follower/libwaypoint_follower.h"

namespace lane_planner
{
enum class ChangeFlag : int32_t
{
  straight,
  right,
  left,

  unknown = -1,
};

template <class T>
typename std::underlying_type<T>::type enumToInteger(T t)
{
  return static_cast<typename std::underlying_type<T>::type>(t);
}

class LaneSelectNode
{
  friend class LaneSelectTestClass;

public:
  LaneSelectNode();

  void run();

private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher pub1_, pub2_, pub3_, pub4_, pub5_;
  ros::Publisher vis_pub1_;
  ros::Timer timer_;

  // subscriber
  ros::Subscriber sub1_, sub5_, sub6_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub2_;
  message_filters::Subscriber<geometry_msgs::TwistStamped> sub3_;
  using PoseTwistSyncPolicy = message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped>;
  using PoseTwistSync = message_filters::Synchronizer<PoseTwistSyncPolicy>;
  std::shared_ptr<PoseTwistSync> pose_twist_sync_;

  // variables
  int32_t lane_array_id_;
  int32_t current_lane_idx_, prev_lane_idx_;
  int32_t right_lane_idx_;
  int32_t left_lane_idx_;
  bool is_new_lane_array_;

  using LaneTuple = std::tuple<autoware_msgs::Lane, int32_t, ChangeFlag>;
  // Hold lane array information subscribed from /traffic_waypoints_array
  // order: lane, closes_waypoint on the lane to ego-vehicle, lane change flag
  std::vector<LaneTuple> tuple_vec_;

  // Hold lane information used in CHANGE_LANE state.
  LaneTuple lane_for_change_;

  bool is_lane_array_subscribed_;
  bool is_current_pose_subscribed_;
  bool is_current_velocity_subscribed_;
  bool is_current_state_subscribed_;
  bool is_config_subscribed_;

  // parameter from runtime manager
  double distance_threshold_;
  double lane_change_interval_;
  double lane_change_target_ratio_;
  double lane_change_target_minimum_;
  double vlength_hermite_curve_;
  int search_closest_waypoint_minimum_dt_;

  // topics
  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::TwistStamped current_velocity_;
  std::string current_state_;
  double update_rate_;

  // callbacks
  void callbackFromLaneArray(const autoware_msgs::LaneArrayConstPtr& msg);
  void callbackFromState(const std_msgs::StringConstPtr& msg);
  void callbackFromDecisionMakerState(const std_msgs::StringConstPtr& msg);
  void callbackFromConfig(const autoware_config_msgs::ConfigLaneSelectConstPtr& msg);
  void callbackFromPoseTwistStamped(const geometry_msgs::PoseStampedConstPtr& pose_msg,
                                    const geometry_msgs::TwistStampedConstPtr& twist_msg);

  // initializer
  void initForROS();

  // visualizer
  void publishVisualizer();
  visualization_msgs::Marker createCurrentLaneMarker();
  visualization_msgs::Marker createRightLaneMarker();
  visualization_msgs::Marker createLeftLaneMarker();
  visualization_msgs::Marker createClosestWaypointsMarker();
  visualization_msgs::Marker createChangeLaneMarker();

  // functions
  void resetLaneIdx();
  void resetSubscriptionFlag();
  bool isAllTopicsSubscribed();
  void processing(const ros::TimerEvent& e);
  void publishLane(const autoware_msgs::Lane& lane);
  void publishClosestWaypoint(const int32_t clst_wp);
  void publishChangeFlag(const ChangeFlag flag);
  void publishVehicleLocation(const int32_t clst_wp, const int32_t larray_id);
  bool updateClosestWaypointNumberForEachLane();
  int32_t findMostClosestLane(const std::vector<uint32_t> idx_vec, const geometry_msgs::Point p);
  void findCurrentLane();
  void findNeighborLanes();
  void changeLane();
  void updateChangeFlag();
  void createLaneForChange();
  int32_t getClosestLaneChangeWaypointNumber(const std::vector<autoware_msgs::Waypoint>& wps, int32_t cl_wp);

  // spinOnce for test
  void spinOnce()
  {
    ros::spinOnce();
  }
};

int32_t getClosestWaypointNumber(const autoware_msgs::Lane& current_lane, const geometry_msgs::Pose& current_pose,
                                 const geometry_msgs::Twist& current_velocity, const int32_t previous_number,
                                 const double distance_threshold, const int search_closest_waypoint_minimum_dt);

double getTwoDimensionalDistance(const geometry_msgs::Point& target1, const geometry_msgs::Point& target2);

geometry_msgs::Point convertPointIntoRelativeCoordinate(const geometry_msgs::Point& input_point,
                                                        const geometry_msgs::Pose& pose);

geometry_msgs::Point convertPointIntoWorldCoordinate(const geometry_msgs::Point& input_point,
                                                     const geometry_msgs::Pose& pose);
double getRelativeAngle(const geometry_msgs::Pose& waypoint_pose, const geometry_msgs::Pose& current_pose);
bool getLinearEquation(geometry_msgs::Point start, geometry_msgs::Point end, double* a, double* b, double* c);
double getDistanceBetweenLineAndPoint(geometry_msgs::Point point, double sa, double b, double c);
}  // namespace lane_planner
#endif  // LANE_SELECT_CORE_H

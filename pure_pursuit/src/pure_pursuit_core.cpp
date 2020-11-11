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

#include <vector>
#include <string>

#include <pure_pursuit/pure_pursuit_core.h>
#include <pure_pursuit/pure_pursuit_viz.h>

namespace waypoint_follower
{
// Constructor
PurePursuitNode::PurePursuitNode()
  : private_nh_("~")
  , pp_()
  , update_rate_(30.0)
  , is_waypoint_set_(false)
  , is_pose_set_(false)
  , is_velocity_set_(false)
  , current_linear_velocity_(0)
  , command_linear_velocity_(0)
  , direction_(LaneDirection::Forward)
  , velocity_source_(-1)
  , const_lookahead_distance_(4.0)
  , const_velocity_(5.0)
  , lookahead_distance_ratio_(2.0)
  , minimum_lookahead_distance_(6.0)
{
  initForROS();
  health_checker_ptr_ = std::make_shared<autoware_health_checker::HealthChecker>(nh_, private_nh_);
  health_checker_ptr_->ENABLE();
  // initialize for PurePursuit
  pp_.setLinearInterpolationParameter(is_linear_interpolation_);
}

void PurePursuitNode::initForROS()
{
  // ros parameter settings
  std::string out_twist, out_ctrl_cmd;
  private_nh_.param("velocity_source", velocity_source_, 0);
  private_nh_.param("is_linear_interpolation", is_linear_interpolation_, true);
  private_nh_.param("add_virtual_end_waypoints", add_virtual_end_waypoints_, false);
  private_nh_.param("const_lookahead_distance", const_lookahead_distance_, 4.0);
  private_nh_.param("const_velocity", const_velocity_, 5.0);
  private_nh_.param("lookahead_ratio", lookahead_distance_ratio_, 2.0);
  private_nh_.param("minimum_lookahead_distance", minimum_lookahead_distance_, 6.0);
  private_nh_.param("update_rate", update_rate_, 30.0);
  private_nh_.param("out_twist_name", out_twist, std::string("twist_raw"));
  private_nh_.param("out_ctrl_cmd_name", out_ctrl_cmd, std::string("ctrl_raw"));
  private_nh_.param("output_interface", output_interface_, std::string("ctrl_cmd"));
  nh_.param("vehicle_info/wheel_base", wheel_base_, 2.7);

  // Output type, use old parameter name only if it is set
  if (private_nh_.hasParam("publishes_for_steering_robot"))
  {
    bool publishes_for_steering_robot;
    private_nh_.param(
      "publishes_for_steering_robot", publishes_for_steering_robot, false);
    if (publishes_for_steering_robot)
    {
      output_interface_ = "ctrl_cmd";
    }
    else
    {
      output_interface_ = "twist";
    }
  }
  else
  {
    private_nh_.param(
      "output_interface", output_interface_, std::string("all"));
  }

  if (output_interface_ != "twist" && output_interface_ != "ctrl_cmd" &&
      output_interface_ != "all")
  {
    ROS_ERROR("Control command interface type is not valid");
    ros::shutdown();
  }

  // setup subscriber
  sub1_ = nh_.subscribe("final_waypoints", 10, &PurePursuitNode::callbackFromWayPoints, this);
  sub2_ = nh_.subscribe("current_pose", 10, &PurePursuitNode::callbackFromCurrentPose, this);
  sub3_ = nh_.subscribe("config/waypoint_follower", 10, &PurePursuitNode::callbackFromConfig, this);
  sub4_ = nh_.subscribe("current_velocity", 10, &PurePursuitNode::callbackFromCurrentVelocity, this);

  // setup publishers
  pub1_ = nh_.advertise<geometry_msgs::TwistStamped>(out_twist, 10);
  pub2_ = nh_.advertise<autoware_msgs::ControlCommandStamped>(out_ctrl_cmd, 10);
  pub11_ = nh_.advertise<visualization_msgs::Marker>("next_waypoint_mark", 0);
  pub12_ = nh_.advertise<visualization_msgs::Marker>("next_target_mark", 0);
  pub13_ = nh_.advertise<visualization_msgs::Marker>("search_circle_mark", 0);
  // debug tool
  pub14_ = nh_.advertise<visualization_msgs::Marker>("line_point_mark", 0);
  pub15_ = nh_.advertise<visualization_msgs::Marker>("trajectory_circle_mark", 0);
  pub16_ = nh_.advertise<std_msgs::Float32>("angular_gravity", 0);
  pub17_ = nh_.advertise<std_msgs::Float32>("deviation_of_current_position", 0);
  pub18_ = nh_.advertise<visualization_msgs::Marker>("expanded_waypoints_mark", 0);
}

void PurePursuitNode::run()
{
  ros::Rate loop_rate(update_rate_);
  while (ros::ok())
  {
    ros::spinOnce();
    if (!is_pose_set_ || !is_waypoint_set_ || !is_velocity_set_)
    {
      if (!is_pose_set_)
      {
        ROS_WARN_THROTTLE(5, "Waiting for current_pose topic ...");
      }
      if (!is_waypoint_set_)
      {
        ROS_WARN_THROTTLE(5, "Waiting for final_waypoints topic ...");
      }
      if (!is_velocity_set_)
      {
        ROS_WARN_THROTTLE(5, "Waiting for current_velocity topic ...");
      }

      loop_rate.sleep();
      continue;
    }

    pp_.setLookaheadDistance(computeLookaheadDistance());
    pp_.setMinimumLookaheadDistance(minimum_lookahead_distance_);

    double kappa = 0;
    bool can_get_curvature = pp_.canGetCurvature(&kappa);

    publishControlCommands(can_get_curvature, kappa);
    health_checker_ptr_->NODE_ACTIVATE();
    health_checker_ptr_->CHECK_RATE("topic_rate_vehicle_cmd_slow", 8, 5, 1, "topic vehicle_cmd publish rate slow.");
    // for visualization with Rviz
    pub11_.publish(displayNextWaypoint(pp_.getPoseOfNextWaypoint()));
    pub13_.publish(displaySearchRadius(pp_.getCurrentPose().position, pp_.getLookaheadDistance()));
    pub12_.publish(displayNextTarget(pp_.getPoseOfNextTarget()));
    pub15_.publish(displayTrajectoryCircle(
        waypoint_follower::generateTrajectoryCircle(pp_.getPoseOfNextTarget(), pp_.getCurrentPose())));
    if (add_virtual_end_waypoints_)
    {
      pub18_.publish(displayExpandWaypoints(pp_.getCurrentWaypoints(), expand_size_));
    }
    std_msgs::Float32 angular_gravity_msg;
    angular_gravity_msg.data = computeAngularGravity(computeCommandVelocity(), kappa);
    pub16_.publish(angular_gravity_msg);

    publishDeviationCurrentPosition(pp_.getCurrentPose().position, pp_.getCurrentWaypoints());

    is_pose_set_ = false;
    is_velocity_set_ = false;

    loop_rate.sleep();
  }
}

void PurePursuitNode::publishControlCommands(const bool& can_get_curvature, const double& kappa) const
{
  if (output_interface_ == "twist")
  {
    publishTwistStamped(can_get_curvature, kappa);
  }
  else if (output_interface_ == "ctrl_cmd")
  {
    publishCtrlCmdStamped(can_get_curvature, kappa);
  }
  else if (output_interface_ == "all")
  {
    publishTwistStamped(can_get_curvature, kappa);
    publishCtrlCmdStamped(can_get_curvature, kappa);
  }
  else
  {
    ROS_WARN("[pure_pursuit] control command interface is not appropriate");
  }
}

void PurePursuitNode::publishTwistStamped(const bool& can_get_curvature, const double& kappa) const
{
  geometry_msgs::TwistStamped ts;
  ts.header.stamp = ros::Time::now();
  ts.twist.linear.x = can_get_curvature ? computeCommandVelocity() : 0;
  ts.twist.angular.z = can_get_curvature ? kappa * ts.twist.linear.x : 0;
  pub1_.publish(ts);
}

void PurePursuitNode::publishCtrlCmdStamped(const bool& can_get_curvature, const double& kappa) const
{
  autoware_msgs::ControlCommandStamped ccs;
  ccs.header.stamp = ros::Time::now();
  ccs.cmd.linear_velocity = can_get_curvature ? computeCommandVelocity() : 0;
  ccs.cmd.linear_acceleration = can_get_curvature ? computeCommandAccel() : 0;
  ccs.cmd.steering_angle = can_get_curvature ? convertCurvatureToSteeringAngle(wheel_base_, kappa) : 0;
  pub2_.publish(ccs);
}

double PurePursuitNode::computeLookaheadDistance() const
{
  if (velocity_source_ == enumToInteger(Mode::dialog))
  {
    return const_lookahead_distance_;
  }

  const double maximum_lookahead_distance = current_linear_velocity_ * 10;
  const double ld = current_linear_velocity_ * lookahead_distance_ratio_;

  return ld < minimum_lookahead_distance_ ? minimum_lookahead_distance_ :
                                            ld > maximum_lookahead_distance ? maximum_lookahead_distance : ld;
}

int PurePursuitNode::getSgn() const
{
  int sgn = 0;
  if (direction_ == LaneDirection::Forward)
  {
    sgn = 1;
  }
  else if (direction_ == LaneDirection::Backward)
  {
    sgn = -1;
  }
  return sgn;
}

double PurePursuitNode::computeCommandVelocity() const
{
  if (velocity_source_ == enumToInteger(Mode::dialog))
  {
    return getSgn() * kmph2mps(const_velocity_);
  }

  return command_linear_velocity_;
}

// Assume constant acceleration motion, v_f^2 - v_i^2 = 2 * a * delta_d
double PurePursuitNode::computeCommandAccel() const
{
  const geometry_msgs::Pose current_pose = pp_.getCurrentPose();
  const geometry_msgs::Pose target_pose = pp_.getCurrentWaypoints().at(1).pose.pose;

  const double delta_d =
      std::hypot(target_pose.position.x - current_pose.position.x, target_pose.position.y - current_pose.position.y);
  const double v_i = current_linear_velocity_;
  const double v_f = computeCommandVelocity();
  return (v_f * v_f - v_i * v_i) / (2 * delta_d);
}

double PurePursuitNode::computeAngularGravity(double velocity, double kappa) const
{
  const double gravity = 9.80665;
  return (velocity * velocity) / (1.0 / kappa * gravity);
}

void PurePursuitNode::callbackFromConfig(const autoware_config_msgs::ConfigWaypointFollowerConstPtr& config)
{
  velocity_source_ = config->param_flag;
  const_lookahead_distance_ = config->lookahead_distance;
  const_velocity_ = config->velocity;
  lookahead_distance_ratio_ = config->lookahead_ratio;
  minimum_lookahead_distance_ = config->minimum_lookahead_distance;
}

void PurePursuitNode::publishDeviationCurrentPosition(const geometry_msgs::Point& point,
                                                      const std::vector<autoware_msgs::Waypoint>& waypoints) const
{
  // Calculate the deviation of current position from the waypoint approximate line
  if (waypoints.size() < 3)
  {
    return;
  }

  const geometry_msgs::Point end = waypoints.at(2).pose.pose.position;
  const geometry_msgs::Point start = waypoints.at(1).pose.pose.position;

  const tf::Vector3 p_A(start.x, start.y, 0.0);
  const tf::Vector3 p_B(end.x, end.y, 0.0);
  const tf::Vector3 p_C(point.x, point.y, 0.0);

  // The distance form a point C to a line passing through A and B is given by
  // length(AB.crossProduct(AC))/length(AC)
  const tf::Vector3 AB = p_B - p_A;
  const tf::Vector3 AC = p_C - p_A;
  const float distance = (AB.cross(AC)).length() / AC.length();

  std_msgs::Float32 msg;
  msg.data = distance;
  pub17_.publish(msg);
}

void PurePursuitNode::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
  pp_.setCurrentPose(msg);
  is_pose_set_ = true;
}

void PurePursuitNode::callbackFromCurrentVelocity(const geometry_msgs::TwistStampedConstPtr& msg)
{
  current_linear_velocity_ = msg->twist.linear.x;
  pp_.setCurrentVelocity(current_linear_velocity_);
  is_velocity_set_ = true;
}

void PurePursuitNode::callbackFromWayPoints(const autoware_msgs::LaneConstPtr& msg)
{
  command_linear_velocity_ = (!msg->waypoints.empty()) ? msg->waypoints.at(0).twist.twist.linear.x : 0;
  if (add_virtual_end_waypoints_)
  {
    const LaneDirection solved_dir = getLaneDirection(*msg);
    direction_ = (solved_dir != LaneDirection::Error) ? solved_dir : direction_;
    autoware_msgs::Lane expanded_lane(*msg);
    expand_size_ = -expanded_lane.waypoints.size();
    connectVirtualLastWaypoints(&expanded_lane, direction_);
    expand_size_ += expanded_lane.waypoints.size();

    pp_.setCurrentWaypoints(expanded_lane.waypoints);
  }
  else
  {
    pp_.setCurrentWaypoints(msg->waypoints);
  }
  is_waypoint_set_ = true;
}

void PurePursuitNode::connectVirtualLastWaypoints(autoware_msgs::Lane* lane, LaneDirection direction)
{
  if (lane->waypoints.empty())
  {
    return;
  }
  static double interval = 1.0;
  const geometry_msgs::Pose& pn = lane->waypoints.back().pose.pose;
  autoware_msgs::Waypoint virtual_last_waypoint;
  virtual_last_waypoint.pose.pose.orientation = pn.orientation;
  virtual_last_waypoint.twist.twist.linear.x = 0.0;
  geometry_msgs::Point virtual_last_point_rlt;
  const int sgn = getSgn();
  for (double dist = minimum_lookahead_distance_; dist > 0.0; dist -= interval)
  {
    virtual_last_point_rlt.x += interval * sgn;
    virtual_last_waypoint.pose.pose.position = calcAbsoluteCoordinate(virtual_last_point_rlt, pn);
    lane->waypoints.emplace_back(virtual_last_waypoint);
  }
}

double convertCurvatureToSteeringAngle(const double& wheel_base, const double& kappa)
{
  return atan(wheel_base * kappa);
}

}  // namespace waypoint_follower

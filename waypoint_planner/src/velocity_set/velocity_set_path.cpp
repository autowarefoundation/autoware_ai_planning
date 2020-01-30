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

#include <waypoint_planner/velocity_set/velocity_set_path.h>

VelocitySetPath::VelocitySetPath()
  : set_path_(false),
    current_vel_(0)
{
  ros::NodeHandle private_nh_("~");
  private_nh_.param<double>("velocity_offset", velocity_offset_, 1.2);
  private_nh_.param<double>("decelerate_vel_min", decelerate_vel_min_, 1.3);
}

VelocitySetPath::~VelocitySetPath()
{
}

// check if waypoint number is valid
bool VelocitySetPath::checkWaypoint(int num, const char *name) const
{
  if (num < 0 || num >= getPrevWaypointsSize())
  {
    return false;
  }
  return true;
}

// set about '_temporal_waypoints_size' meter waypoints from closest waypoint
void VelocitySetPath::setTemporalWaypoints(int temporal_waypoints_size, int closest_waypoint, geometry_msgs::PoseStamped control_pose)
{
  if (closest_waypoint < 0)
    return;

  temporal_waypoints_.waypoints.clear();
  temporal_waypoints_.header = new_waypoints_.header;
  temporal_waypoints_.increment = new_waypoints_.increment;

  // push current pose
  autoware_msgs::Waypoint current_point;
  current_point.pose = control_pose;
  current_point.twist = new_waypoints_.waypoints[closest_waypoint].twist;
  current_point.dtlane = new_waypoints_.waypoints[closest_waypoint].dtlane;
  temporal_waypoints_.waypoints.push_back(current_point);

  for (int i = 0; i < temporal_waypoints_size; i++)
  {
    if (closest_waypoint + i >= getNewWaypointsSize())
      return;

    temporal_waypoints_.waypoints.push_back(new_waypoints_.waypoints[closest_waypoint + i]);
  }

  return;
}

double VelocitySetPath::calcChangedVelocity(const double& current_vel, const double& accel, const std::array<int, 2>& range) const
{
  static double current_velocity = current_vel;
  static double square_vel = current_vel * current_vel;
  if (current_velocity != current_vel)
  {
    current_velocity = current_vel;
    square_vel = current_vel * current_vel;
  }
  return std::sqrt(square_vel + 2.0 * accel * calcInterval(range.at(0), range.at(1)));
}

void VelocitySetPath::changeWaypointsForDeceleration(double deceleration, int closest_waypoint, int obstacle_waypoint)
{
  int extra = 4; // for safety

  // decelerate with constant deceleration
  for (int index = obstacle_waypoint + extra; index >= closest_waypoint; index--)
  {
    if (!checkWaypoint(index, __FUNCTION__))
      continue;

    // v = sqrt( (v0)^2 + 2ax )
    std::array<int, 2> range = {index, obstacle_waypoint};
    double changed_vel = calcChangedVelocity(decelerate_vel_min_, deceleration, range);

    double prev_vel = prev_waypoints_.waypoints[index].twist.twist.linear.x;
    const int sgn = (prev_vel < 0) ? -1 : 1;
    new_waypoints_.waypoints[index].twist.twist.linear.x = sgn * std::min(std::abs(prev_vel), changed_vel);
  }

}

void VelocitySetPath::avoidSuddenAcceleration(double deceleration, int closest_waypoint)
{
  for (int i = 0;; i++)
  {
    if (!checkWaypoint(closest_waypoint + i, __FUNCTION__))
      return;

    // accelerate with constant acceleration
    // v = root((v0)^2 + 2ax)
    std::array<int, 2> range = {closest_waypoint, closest_waypoint + i};
    double changed_vel = calcChangedVelocity(current_vel_, deceleration, range) + velocity_offset_;

    const double& target_vel = new_waypoints_.waypoints[closest_waypoint + i].twist.twist.linear.x;
    // Don't exceed original velocity
    if (changed_vel > std::abs(target_vel))
      return;

    const int sgn = (target_vel < 0) ? -1 : 1;
    new_waypoints_.waypoints[closest_waypoint + i].twist.twist.linear.x = sgn * changed_vel;
  }

  return;
}

void VelocitySetPath::avoidSuddenDeceleration(double velocity_change_limit, double deceleration, int closest_waypoint)
{
  if (closest_waypoint < 0)
    return;

  const double& closest_vel = new_waypoints_.waypoints[closest_waypoint].twist.twist.linear.x;

  // if accelerating, do not modify the speed profile.
  if ((current_vel_ >= 0.0 && current_vel_ <= closest_vel) || (current_vel_ < 0.0 && current_vel_ > closest_vel))
    return;

  // not avoid braking
  if (std::abs(current_vel_ - closest_vel) < velocity_change_limit)
    return;

  //std::cout << "avoid sudden braking!" << std::endl;
  for (int i = 0;; i++)
  {
    if (!checkWaypoint(closest_waypoint + i, __FUNCTION__))
      return;

    // sqrt(v^2 - 2ax)
    std::array<int, 2> range = {closest_waypoint, closest_waypoint + i};
    double changed_vel = calcChangedVelocity(std::abs(current_vel_) - velocity_change_limit, -deceleration, range);
    const double& target_vel = new_waypoints_.waypoints[closest_waypoint + i].twist.twist.linear.x;

    if (std::isnan(changed_vel))
    {
      break;
    }
    const int sgn = (target_vel < 0) ? -1 : 1;
    new_waypoints_.waypoints[closest_waypoint + i].twist.twist.linear.x = sgn * changed_vel;
  }

}

void VelocitySetPath::changeWaypointsForStopping(int stop_waypoint, int obstacle_waypoint, int closest_waypoint, double deceleration)
{
  if (closest_waypoint < 0)
    return;

  // decelerate with constant deceleration
  for (int index = stop_waypoint; index >= closest_waypoint; index--)
  {
    if (!checkWaypoint(index, __FUNCTION__))
      continue;

    // v = (v0)^2 + 2ax, and v0 = 0
    std::array<int, 2> range = {index, stop_waypoint};
    double changed_vel = calcChangedVelocity(0.0, deceleration, range);

    double prev_vel = prev_waypoints_.waypoints[index].twist.twist.linear.x;
    const int sgn = (prev_vel < 0) ? -1 : 1;
    new_waypoints_.waypoints[index].twist.twist.linear.x = sgn * std::min(std::abs(prev_vel), changed_vel);
  }

  // fill velocity with 0 for stopping
  for (int i = stop_waypoint; i <= obstacle_waypoint; i++)
  {
    new_waypoints_.waypoints[i].twist.twist.linear.x = 0;
  }

}

void VelocitySetPath::initializeNewWaypoints()
{
  new_waypoints_ = prev_waypoints_;
}

double VelocitySetPath::calcInterval(const int begin, const int end) const
{
  // check index
  if (begin < 0 || begin >= getPrevWaypointsSize() || end < 0 || end >= getPrevWaypointsSize())
  {
    ROS_WARN("Invalid index");
    return 0;
  }

  // Calculate the inteval of waypoints
  double dist_sum = 0;
  for (int i = begin; i < end; i++)
  {
    tf::Vector3 v1(prev_waypoints_.waypoints[i].pose.pose.position.x,
                   prev_waypoints_.waypoints[i].pose.pose.position.y, 0);

    tf::Vector3 v2(prev_waypoints_.waypoints[i + 1].pose.pose.position.x,
                   prev_waypoints_.waypoints[i + 1].pose.pose.position.y, 0);

    dist_sum += tf::tfDistance(v1, v2);
  }

  return dist_sum;
}

void VelocitySetPath::resetFlag()
{
  set_path_ = false;
}


void VelocitySetPath::waypointsCallback(const autoware_msgs::LaneConstPtr& msg)
{
  prev_waypoints_ = *msg;
  // temporary, edit waypoints velocity later
  new_waypoints_ = *msg;

  set_path_ = true;
}

void VelocitySetPath::currentVelocityCallback(const geometry_msgs::TwistStampedConstPtr& msg)
{
  current_vel_ = msg->twist.linear.x;
}

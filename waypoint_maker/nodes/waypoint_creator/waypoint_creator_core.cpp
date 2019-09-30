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

#include <iostream>
#include <amathutils_lib/amathutils.hpp>

#include "waypoint_creator/interpolate.h"
#include "waypoint_creator/waypoint_creator_core.hpp"

WaypointCreator::WaypointCreator() : nh_(""), pnh_("~"), tf2_listener_(tf2_buffer_)
{
  std::string out_lane_array_topic;
  pnh_.param("out_lane_array_topic", out_lane_array_topic, std::string("/lane_waypoints_array"));
  pnh_.param("remove_pose_thr", remove_pose_thr_, double(2.0));
  pnh_.param("interpolation_interval", interpolation_interval_, double(1.0));
  pnh_.param("interpolation_method", interpolation_method_, std::string("spline"));
  pnh_.param("waypoint_velocity", waypoint_velocity_, double(5.0));
  pnh_.param("waypoint_frame_id", waypoint_frame_id_, std::string("map"));

  std::string source_point, delete_pose;
  pnh_.param("source_point_topic", source_point, std::string("/clicked_point"));
  pnh_.param("delete_pose_topic", delete_pose, std::string("/move_base_simple/goal"));
  sub_source_point_ = nh_.subscribe(source_point, 1, &WaypointCreator::inputPointCallback, this);
  sub_delete_point_ = nh_.subscribe(delete_pose, 1, &WaypointCreator::deletePoseCallback, this);
  pub_waypoints_ = pnh_.advertise<autoware_msgs::LaneArray>(out_lane_array_topic, 1);
  pub_visualize_ = pnh_.advertise<visualization_msgs::MarkerArray>("debug/waypoints", 1);

  source_pose_v_.clear();
  interpolated_pose_v_.clear();
}

void WaypointCreator::inputPointCallback(const geometry_msgs::PointStamped& in_point)
{
  geometry_msgs::PointStamped in_point_converted;
  try
  {
    tf2_buffer_.transform(in_point, in_point_converted, waypoint_frame_id_, ros::Duration(1.0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("[WaypointCreator] transform failed %s\n", ex.what());  // Print exception which was caught
    return;
  }
  geometry_msgs::Pose p;
  p.position = in_point_converted.point;
  source_pose_v_.push_back(p);

  interpolatePosesWithConstantDistance(source_pose_v_, interpolation_interval_, interpolation_method_, interpolated_pose_v_);

  publishWaypoints();
  visualizeWaypoints();
}

void WaypointCreator::deletePoseCallback(const geometry_msgs::PoseStamped& in_pose)
{
  geometry_msgs::PoseStamped in_pose_converted;
  try
  {
    tf2_buffer_.transform(in_pose, in_pose_converted, waypoint_frame_id_, ros::Duration(1.0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("[WaypointCreator] transform failed %s\n", ex.what());  // Print exception which was caught
    return;
  }

  unsigned int closest = 0;
  double dist_min = 1.0E5;
  for (unsigned int i = 0; i < source_pose_v_.size(); ++i)
  {
    double dist = amathutils::find_distance(in_pose_converted.pose, source_pose_v_.at(i));
    if (dist < dist_min)
    {
      dist_min = dist;
      closest = i;
    }
  }
  if (dist_min < remove_pose_thr_)
  {
    deleteVisualizeWaypoints();
    ROS_INFO("[WaypointCreator] remove pose");
    source_pose_v_.erase(source_pose_v_.begin() + closest);
  }
  else
  {
    ROS_INFO("[WaypointCreator] delete pose called, but found no points around received pose");
  }

  interpolatePosesWithConstantDistance(source_pose_v_, interpolation_interval_, interpolation_method_, interpolated_pose_v_);

  publishWaypoints();
  visualizeWaypoints();
}

bool WaypointCreator::interpolatePosesWithConstantDistance(const std::vector<geometry_msgs::Pose>& in_poses,
                                                           const double& interpolation_interval,
                                                           const std::string& method,
                                                           std::vector<geometry_msgs::Pose>& out_poses) const
{
  if (in_poses.size() < 2)
  {
    out_poses = in_poses;
    return true;
  }

  if (method == "linear")
  {
    return interpolatePoses(in_poses, interpolation_interval, method, out_poses);
  }
  else if (method == "spline")
  {
    // apply twise for interval length
    std::vector<geometry_msgs::Pose> tmp;
    bool first_ret = interpolatePoses(in_poses, interpolation_interval, method, tmp);
    if (!first_ret)
    {
      return false;
    }
    else
    {
      return interpolatePoses(tmp, interpolation_interval, method, out_poses);
    }
  }
  else
  {
    ROS_WARN("[WaypointCreator] invalid interpolation method");
    return false;
  }
}

bool WaypointCreator::interpolatePoses(const std::vector<geometry_msgs::Pose>& in_poses,
                                       const double& interpolation_interval, const std::string& method,
                                       std::vector<geometry_msgs::Pose>& out_poses) const
{
  if (in_poses.size() == 0)
  {
    ROS_WARN("[WaypointCreator] input pose vector is empty in interpolation. return empty poses.");
    return false;
  }

  if (method != "linear" && method != "spline")
  {
    ROS_WARN("[WaypointCreator] unknown method for interpolation. return empty pose.");
    return false;
  }

  out_poses.clear();

  // convert vector<pose> to vector<double>
  std::vector<double> in_pos_x_v, in_pos_y_v, in_pos_z_v, in_yaw_v;
  for (auto& pose : in_poses)
  {
    in_pos_x_v.push_back(pose.position.x);
    in_pos_y_v.push_back(pose.position.y);
    in_pos_z_v.push_back(pose.position.z);
  }

  // calculate input vector distance
  std::vector<double> in_dist_v;
  in_dist_v.push_back(0.0);
  for (unsigned int i = 0; i < in_poses.size() - 1; ++i)
  {
    const double dx = in_poses.at(i + 1).position.x - in_poses.at(i).position.x;
    const double dy = in_poses.at(i + 1).position.y - in_poses.at(i).position.y;
    const double dz = in_poses.at(i + 1).position.z - in_poses.at(i).position.z;
    in_dist_v.push_back(in_dist_v.at(i) + std::hypot(std::hypot(dx, dy), dz));
  }

  // calculate desired distance vector
  std::vector<double> out_dist_v;
  for (double dist_sum = 0.0; dist_sum < in_dist_v.back(); dist_sum += interpolation_interval)
  {
    out_dist_v.push_back(dist_sum);
  }
  out_dist_v.push_back(in_dist_v.back());

  // apply spline interpolation
  std::vector<double> out_pos_x_v, out_pos_y_v, out_pos_z_v, out_yaw_v;
  if (method == "spline")
  {
    SplineInterpolate spline_interploate;
    if (!spline_interploate.interpolate(in_dist_v, in_pos_x_v, out_dist_v, out_pos_x_v) ||
        !spline_interploate.interpolate(in_dist_v, in_pos_y_v, out_dist_v, out_pos_y_v) ||
        !spline_interploate.interpolate(in_dist_v, in_pos_z_v, out_dist_v, out_pos_z_v))
    {
      ROS_ERROR("[WaypointCreator] spline interpolation failed!!!");
      return false;
    }
  }
  else if (method == "linear")
  {
    LinearInterpolate linear_interploate;
    if (!linear_interploate.interpolate(in_dist_v, in_pos_x_v, out_dist_v, out_pos_x_v) ||
        !linear_interploate.interpolate(in_dist_v, in_pos_y_v, out_dist_v, out_pos_y_v) ||
        !linear_interploate.interpolate(in_dist_v, in_pos_z_v, out_dist_v, out_pos_z_v))
    {
      ROS_ERROR("[WaypointCreator] linear interpolation failed!!!");
      return false;
    }
  }

  // yaw angle calculation
  for (int i = 0; i < (int)out_dist_v.size(); ++i)
  {
    int i_next = std::min(i + 1, (int)out_dist_v.size() - 1);
    int i_prev = std::max(i - 1, 0);
    const double eps = 1e-7;
    const double dx = out_pos_x_v.at(i_next) - out_pos_x_v.at(i_prev);
    const double dy = out_pos_y_v.at(i_next) - out_pos_y_v.at(i_prev);
    const double den = std::fabs(dx) > eps ? dx : (dx > 0.0 ? eps : -eps);
    const double yaw_tmp = std::atan2(dy, den);
    out_yaw_v.push_back(yaw_tmp);
  }

  // generate pose vector
  for (unsigned int i = 0; i < out_dist_v.size(); ++i)
  {
    geometry_msgs::Pose p;
    p.position.x = out_pos_x_v.at(i);
    p.position.y = out_pos_y_v.at(i);
    p.position.z = out_pos_z_v.at(i);
    p.orientation = amathutils::getQuaternionFromYaw(out_yaw_v.at(i));
    out_poses.push_back(p);
  }
  return true;
}

void WaypointCreator::publishWaypoints() const
{
  autoware_msgs::LaneArray lane_array;
  autoware_msgs::Lane lane;
  autoware_msgs::Waypoint wps;

  lane.header.frame_id = waypoint_frame_id_;
  lane.header.stamp = ros::Time::now();
  for (const auto& p : interpolated_pose_v_)
  {
    wps.pose.pose = p;
    wps.twist.twist.linear.x = waypoint_velocity_;
    lane.waypoints.push_back(wps);
  }
  lane_array.lanes.push_back(lane);

  pub_waypoints_.publish(lane_array);
}

void WaypointCreator::visualizeWaypoints() const
{
  geometry_msgs::Vector3 scale;
  scale.x = 1.0;
  scale.y = 0.3;
  scale.z = 0.3;
  visualization_msgs::MarkerArray marker_array;
  for (unsigned int i = 0; i < source_pose_v_.size(); ++i)
  {
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = 1.0;
    marker_array.markers.push_back(createPoseMarker(source_pose_v_.at(i), color, scale, "raw", i));
  }
  for (unsigned int i = 0; i < interpolated_pose_v_.size(); ++i)
  {
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.g = 1.0;
    marker_array.markers.push_back(createPoseMarker(interpolated_pose_v_.at(i), color, scale, "interpolated", i));
  }

  pub_visualize_.publish(marker_array);
}

visualization_msgs::Marker WaypointCreator::createPoseMarker(const geometry_msgs::Pose& pose,
                                                             const std_msgs::ColorRGBA& color,
                                                             const geometry_msgs::Vector3& scale, const std::string& ns,
                                                             const int32_t id) const
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = waypoint_frame_id_;
  marker.header.stamp = ros::Time();
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose;
  marker.scale = scale;
  marker.color = color;
  return marker;
}

void WaypointCreator::deleteVisualizeWaypoints()
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker delete_marker;
  delete_marker.action = visualization_msgs::Marker::DELETE;
  for (unsigned int i = 0; i < source_pose_v_.size(); ++i)
  {
    delete_marker.ns = "raw";
    delete_marker.id = i;
    marker_array.markers.push_back(delete_marker);
  }
  for (unsigned int i = 0; i < interpolated_pose_v_.size(); ++i)
  {
    delete_marker.ns = "interpolated";
    delete_marker.id = i;
    marker_array.markers.push_back(delete_marker);
  }
  pub_visualize_.publish(marker_array);
}


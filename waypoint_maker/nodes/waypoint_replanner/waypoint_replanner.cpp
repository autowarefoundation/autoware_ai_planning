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
#include "waypoint_replanner.h"

namespace waypoint_maker
{

WaypointReplanner::WaypointReplanner()
{
}

WaypointReplanner::~WaypointReplanner()
{
}

void WaypointReplanner::updateConfig(const WaypointReplannerConfig& config)
{
  config_ = config;
  config_.radius_inf = 10 * config_.radius_thresh;
  config_.velocity_param = calcVelParam(config_.velocity_max);
}

void WaypointReplanner::initParameter(const autoware_config_msgs::ConfigWaypointReplanner::ConstPtr& conf)
{
  WaypointReplannerConfig temp_config;

  temp_config.velocity_max = kmph2mps(conf->velocity_max);
  temp_config.velocity_min = kmph2mps(conf->velocity_min);
  temp_config.accel_limit = conf->accel_limit;
  temp_config.decel_limit = conf->decel_limit;
  temp_config.radius_thresh = conf->radius_thresh;
  temp_config.radius_min = conf->radius_min;
  temp_config.lookup_crv_width = 5;
  temp_config.resample_mode = conf->resample_mode;
  temp_config.resample_interval = conf->resample_interval;
  temp_config.replan_curve_mode = conf->replan_curve_mode;
  temp_config.replan_endpoint_mode = conf->replan_endpoint_mode;
  temp_config.overwrite_vmax_mode = conf->overwrite_vmax_mode;
  temp_config.velocity_offset = conf->velocity_offset;
  temp_config.end_point_offset = conf->end_point_offset;
  temp_config.braking_distance = conf->braking_distance;

  updateConfig(temp_config);
}

void WaypointReplanner::changeVelSign(autoware_msgs::Lane& lane, bool positive) const
{
  const int sgn = positive ? 1 : -1;
  for (auto& el : lane.waypoints)
  {
    el.twist.twist.linear.x = sgn * fabs(el.twist.twist.linear.x);
  }
}

void WaypointReplanner::replanLaneWaypointVel(autoware_msgs::Lane& lane)
{
  if (lane.waypoints.empty())
  {
    return;
  }
  const LaneDirection dir = getLaneDirection(lane);
  unsigned long last = lane.waypoints.size() - 1;
  changeVelSign(lane, true);
  limitVelocityByRange(0, last, 0, config_.velocity_max, lane);
  if (config_.resample_mode)
  {
    resampleLaneWaypoint(config_.resample_interval, lane, dir);
    last = lane.waypoints.size() - 1;
  }
  if (config_.replan_curve_mode)
  {
    std::vector<double> curve_radius;
    KeyVal curve_list;
    createRadiusList(lane, curve_radius);
    createCurveList(curve_radius, curve_list);
    if (config_.overwrite_vmax_mode)
    {// set velocity_max for all_point
      setVelocityByRange(0, last, 0, config_.velocity_max, lane);
    }
    // set velocity by curve
    for (const auto& el : curve_list)
    {
      const double& radius = el.second.second;
      double vmin = config_.velocity_max - config_.velocity_param * (config_.radius_thresh - radius);
      vmin = (vmin < config_.velocity_min) ? config_.velocity_min : vmin;
      limitVelocityByRange(el.first, el.second.first, config_.velocity_offset, vmin, lane);
    }
  }
  // set velocity on start & end of lane
  if (config_.replan_endpoint_mode)
  {
    const unsigned long zerospeed_start = last - config_.end_point_offset;
    const unsigned long lowspeed_start = zerospeed_start - config_.braking_distance;
    raiseVelocityByRange(0, last, 0, config_.velocity_min, lane);
    limitVelocityByRange(0, 0, 0, config_.velocity_min, lane);
    limitVelocityByRange(lowspeed_start, last, 0, config_.velocity_min, lane);
    setVelocityByRange(zerospeed_start, last, 0, 0.0, lane);
  }
  if (dir == LaneDirection::Backward)
  {
    changeVelSign(lane, false);
  }
}

void WaypointReplanner::resampleLaneWaypoint(const double resample_interval, autoware_msgs::Lane& lane, LaneDirection dir)
{
  if (lane.waypoints.size() < 2)
  {
    return;
  }
  autoware_msgs::Lane original_lane(lane);
  lane.waypoints.clear();
  lane.waypoints.emplace_back(original_lane.waypoints[0]);
  lane.waypoints.reserve(ceil(1.5 * calcPathLength(original_lane) / config_.resample_interval));

  for (unsigned long i = 1; i < original_lane.waypoints.size(); i++)
  {
    CbufGPoint curve_point = getCrvPointsOnResample(lane, original_lane, i);
    const std::vector<double> curve_param = calcCurveParam(curve_point);
    lane.waypoints.back().twist.twist = original_lane.waypoints[i - 1].twist.twist;
    lane.waypoints.back().wpstate = original_lane.waypoints[i - 1].wpstate;
    lane.waypoints.back().change_flag = original_lane.waypoints[i - 1].change_flag;
    // if going straight
    if (curve_param.empty())
    {
      resampleOnStraight(curve_point, lane, dir);
    }
    // else if turnning curve
    else
    {
      resampleOnCurve(curve_point[1], curve_param, lane, dir);
    }
  }
  lane.waypoints[0].pose.pose.orientation = lane.waypoints[1].pose.pose.orientation;
  lane.waypoints.back().twist.twist = original_lane.waypoints.back().twist.twist;
  lane.waypoints.back().wpstate = original_lane.waypoints.back().wpstate;
  lane.waypoints.back().change_flag = original_lane.waypoints.back().change_flag;
}

void WaypointReplanner::resampleOnStraight(const CbufGPoint& curve_point, autoware_msgs::Lane& lane, LaneDirection dir)
{
  if (curve_point.size() != 3)
  {
    return;
  }
  autoware_msgs::Waypoint wp(lane.waypoints.back());
  const geometry_msgs::Point& pt = wp.pose.pose.position;
  const int sgn = (dir == LaneDirection::Forward) ? 0 : 1;
  const double yaw = atan2(curve_point[2].y - curve_point[0].y, curve_point[2].x - curve_point[0].x) + sgn * M_PI;
  wp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

  const std::vector<double> nvec = { curve_point[1].x - pt.x, curve_point[1].y - pt.y, curve_point[1].z - pt.z };
  double dist = sqrt(nvec[0] * nvec[0] + nvec[1] * nvec[1]);
  std::vector<double> resample_vec = nvec;
  const double coeff = config_.resample_interval / dist;
  for (auto& el : resample_vec)
  {
    el *= coeff;
  }
  for (; dist > config_.resample_interval; dist -= config_.resample_interval)
  {
    wp.pose.pose.position.x += resample_vec[0];
    wp.pose.pose.position.y += resample_vec[1];
    wp.pose.pose.position.z += resample_vec[2];
    lane.waypoints.emplace_back(wp);
  }
}

void WaypointReplanner::resampleOnCurve(const geometry_msgs::Point& target_point,
                                        const std::vector<double>& curve_param, autoware_msgs::Lane& lane, LaneDirection dir)
{
  if (curve_param.size() != 3)
  {
    return;
  }
  autoware_msgs::Waypoint wp(lane.waypoints.back());
  const double& cx = curve_param[0];
  const double& cy = curve_param[1];
  const double& radius = curve_param[2];
  const double reverse_angle = (dir == LaneDirection::Backward) ? M_PI : 0.0;

  const geometry_msgs::Point& p0 = wp.pose.pose.position;
  const geometry_msgs::Point& p1 = target_point;
  double theta = fmod(atan2(p1.y - cy, p1.x - cx) - atan2(p0.y - cy, p0.x - cx), 2 * M_PI);
  int sgn = (theta > 0.0) ? (1) : (-1);
  if (fabs(theta) > M_PI)
  {
    theta -= 2 * sgn * M_PI;
  }
  sgn = (theta > 0.0) ? (1) : (-1);
  // interport
  double t = atan2(p0.y - cy, p0.x - cx);
  double dist = radius * fabs(theta);
  const double resample_dz = config_.resample_interval * (p1.z - p0.z) / dist;
  for (; dist > config_.resample_interval; dist -= config_.resample_interval)
  {
    if (lane.waypoints.size() == lane.waypoints.capacity())
    {
      break;
    }
    t += sgn * config_.resample_interval / radius;
    const double yaw = fmod(t + sgn * M_PI / 2.0, 2 * M_PI) + reverse_angle;
    wp.pose.pose.position.x = cx + radius * cos(t);
    wp.pose.pose.position.y = cy + radius * sin(t);
    wp.pose.pose.position.z += resample_dz;
    wp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    lane.waypoints.emplace_back(wp);
  }
}

// Three points used for curve detection (the target point is the center)
// [0] = previous point, [1] = target point, [2] = next point
const CbufGPoint WaypointReplanner::getCrvPointsOnResample(
    const autoware_msgs::Lane& lane, const autoware_msgs::Lane& original_lane, unsigned long original_index) const
{
  unsigned long id = original_index;
  CbufGPoint curve_point(3);
  const unsigned int n = (config_.lookup_crv_width - 1) / 2;
  const autoware_msgs::Waypoint cp[3] = {
    (lane.waypoints.size() < n) ? lane.waypoints.front() : lane.waypoints[lane.waypoints.size() - n],
    original_lane.waypoints[id],
    (id < original_lane.waypoints.size() - n) ? original_lane.waypoints[id + n] : original_lane.waypoints.back()
  };
  for (int i = 0; i < 3; i++)
  {
    curve_point.push_back(cp[i].pose.pose.position);
  }
  return curve_point;
}

const CbufGPoint WaypointReplanner::getCrvPoints(const autoware_msgs::Lane& lane, unsigned long index) const
{
  CbufGPoint curve_point(3);
  const unsigned int n = (config_.lookup_crv_width - 1) / 2;
  const unsigned long curve_index[3] = { (index < n) ? 0 : (index - n), index, (index >= lane.waypoints.size() - n) ?
                                                                                   (lane.waypoints.size() - 1) :
                                                                                   (index + n) };
  for (int i = 0; i < 3; i++)
  {
    curve_point.push_back(lane.waypoints[curve_index[i]].pose.pose.position);
  }
  return curve_point;
}

void WaypointReplanner::createRadiusList(const autoware_msgs::Lane& lane, std::vector<double>& curve_radius)
{
  if (lane.waypoints.empty())
  {
    return;
  }
  curve_radius.resize(lane.waypoints.size());
  curve_radius.at(0) = curve_radius.back() = config_.radius_inf;

  for (unsigned long i = 1; i < lane.waypoints.size() - 1; i++)
  {
    CbufGPoint curve_point(getCrvPoints(lane, i));
    const std::vector<double> curve_param(calcCurveParam(curve_point));

    // if going straight
    if (curve_param.empty())
    {
      curve_radius.at(i) = config_.radius_inf;
    }
    // else if turnning curve
    else
    {
      curve_radius.at(i) = (curve_param[2] > config_.radius_inf) ? config_.radius_inf : curve_param[2];
    }
  }
}

const double WaypointReplanner::calcVelParam(double vmax) const
{
  if (fabs(config_.radius_thresh - config_.radius_min) < 1e-8)
  {
    return DBL_MAX;  // error
  }
  return (vmax - config_.velocity_min) / (config_.radius_thresh - config_.radius_min);
}

void WaypointReplanner::createCurveList(const std::vector<double>& curve_radius, KeyVal& curve_list)
{
  unsigned long index = 0;
  bool on_curve = false;
  double radius_localmin = DBL_MAX;
  for (unsigned long i = 1; i < curve_radius.size(); i++)
  {
    if (!on_curve && curve_radius[i] <= config_.radius_thresh && curve_radius[i - 1] > config_.radius_thresh)
    {
      index = i;
      on_curve = true;
    }
    else if (on_curve && curve_radius[i - 1] <= config_.radius_thresh && curve_radius[i] > config_.radius_thresh)
    {
      on_curve = false;
      if (radius_localmin < config_.radius_min)
      {
        radius_localmin = config_.radius_min;
      }
      curve_list[index] = std::make_pair(i, radius_localmin);
      radius_localmin = DBL_MAX;
    }
    if (!on_curve)
    {
      continue;
    }
    if (radius_localmin > curve_radius[i])
    {
      radius_localmin = curve_radius[i];
    }
  }
}


void WaypointReplanner::setVelocityByRange(unsigned long start_idx, unsigned long end_idx, unsigned int offset,
                                             double vel, autoware_msgs::Lane& lane)
{
  if (lane.waypoints.empty())
  {
    return;
  }
  if (offset > 0)
  {
    start_idx = (start_idx > offset) ? (start_idx - offset) : 0;
    end_idx = (end_idx > offset) ? (end_idx - offset) : 0;
  }
  end_idx = (end_idx >= lane.waypoints.size()) ? lane.waypoints.size() - 1 : end_idx;
  for (unsigned long idx = start_idx; idx <= end_idx; idx++)
  {
    lane.waypoints[idx].twist.twist.linear.x = vel;
  }
}

void WaypointReplanner::raiseVelocityByRange(unsigned long start_idx, unsigned long end_idx, unsigned int offset,
                                           double vmin, autoware_msgs::Lane& lane)
{
  if (lane.waypoints.empty())
  {
    return;
  }
  if (offset > 0)
  {
    start_idx = (start_idx > offset) ? (start_idx - offset) : 0;
    end_idx = (end_idx > offset) ? (end_idx - offset) : 0;
  }
  end_idx = (end_idx >= lane.waypoints.size()) ? lane.waypoints.size() - 1 : end_idx;
  for (unsigned long idx = start_idx; idx <= end_idx; idx++)
  {
    if (lane.waypoints[idx].twist.twist.linear.x >= vmin)
    {
      continue;
    }
    lane.waypoints[idx].twist.twist.linear.x = vmin;
  }
}

void WaypointReplanner::limitVelocityByRange(unsigned long start_idx, unsigned long end_idx, unsigned int offset,
                                             double vmin, autoware_msgs::Lane& lane)
{
  if (lane.waypoints.empty())
  {
    return;
  }
  if (offset > 0)
  {
    start_idx = (start_idx > offset) ? (start_idx - offset) : 0;
    end_idx = (end_idx > offset) ? (end_idx - offset) : 0;
  }
  end_idx = (end_idx >= lane.waypoints.size()) ? lane.waypoints.size() - 1 : end_idx;
  for (unsigned long idx = start_idx; idx <= end_idx; idx++)
  {
    if (lane.waypoints[idx].twist.twist.linear.x < vmin)
    {
      continue;
    }
    lane.waypoints[idx].twist.twist.linear.x = vmin;
  }
  limitAccelDecel(start_idx, lane);
  limitAccelDecel(end_idx, lane);
}

void WaypointReplanner::limitAccelDecel(const unsigned long idx, autoware_msgs::Lane& lane)
{
  const double acc[2] = { config_.accel_limit, config_.decel_limit };
  const unsigned long end_idx[2] = { lane.waypoints.size() - idx, idx + 1 };
  const int sgn[2] = { 1, -1 };
  for (int j = 0; j < 2; j++)  // [j=0]: config_.accel_limitprocess, [j=1]: config_.decel_limitprocess
  {
    double v = lane.waypoints[idx].twist.twist.linear.x;
    unsigned long next = idx + sgn[j];
    for (unsigned long i = 1; i < end_idx[j]; i++, next += sgn[j])
    {
      const geometry_msgs::Point& p0 = lane.waypoints[next - sgn[j]].pose.pose.position;
      const geometry_msgs::Point& p1 = lane.waypoints[next].pose.pose.position;
      const double dist = std::hypot(p0.x - p1.x, p0.y - p1.y);
      v = sqrt(2 * acc[j] * dist + v * v);
      if (v > config_.velocity_max || v > lane.waypoints[next].twist.twist.linear.x)
      {
        break;
      }
      lane.waypoints[next].twist.twist.linear.x = v;
    }
  }
}

// get curve 3-Parameter [center_x, center_y, radius] with 3 point input. If error occured, return empty vector.
const std::vector<double> WaypointReplanner::calcCurveParam(CbufGPoint p) const
{
  for (int i = 0; i < 3; i++, p.push_back(p.front()))  // if exception occured, change points order
  {
    const double d = 2 * ((p[0].y - p[2].y) * (p[0].x - p[1].x) - (p[0].y - p[1].y) * (p[0].x - p[2].x));
    if (fabs(d) < 1e-8)
    {
      continue;
    }
    const std::vector<double> x2 = { p[0].x * p[0].x, p[1].x * p[1].x, p[2].x * p[2].x };
    const std::vector<double> y2 = { p[0].y * p[0].y, p[1].y * p[1].y, p[2].y * p[2].y };
    const double a = y2[0] - y2[1] + x2[0] - x2[1];
    const double b = y2[0] - y2[2] + x2[0] - x2[2];
    std::vector<double> param(3);
    const double cx = param[0] = ((p[0].y - p[2].y) * a - (p[0].y - p[1].y) * b) / d;
    const double cy = param[1] = ((p[0].x - p[2].x) * a - (p[0].x - p[1].x) * b) / -d;
    param[2] = sqrt((cx - p[0].x) * (cx - p[0].x) + (cy - p[0].y) * (cy - p[0].y));
    return param;
  }
  return std::vector<double>();  // error
}

const double WaypointReplanner::calcPathLength(const autoware_msgs::Lane& lane) const
{
  double distance = 0.0;
  for (unsigned long i = 1; i < lane.waypoints.size(); i++)
  {
    const geometry_msgs::Point& p0 = lane.waypoints[i - 1].pose.pose.position;
    const geometry_msgs::Point& p1 = lane.waypoints[i].pose.pose.position;
    tf::Vector3 tf0(p0.x, p0.y, 0.0);
    tf::Vector3 tf1(p1.x, p1.y, 0.0);
    distance += tf::tfDistance(tf0, tf1);
  }
  return distance;
}

};

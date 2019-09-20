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
#ifndef __WAYPOINT_REPLANNER_H__
#define __WAYPOINT_REPLANNER_H__

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <autoware_config_msgs/ConfigWaypointReplanner.h>
#include <fstream>
#include <unordered_map>
#include <algorithm>
#include <boost/circular_buffer.hpp>
#include <autoware_msgs/Lane.h>
#include <libwaypoint_follower/libwaypoint_follower.h>

namespace waypoint_maker
{
typedef std::unordered_map<unsigned long, std::pair<unsigned long, double>> KeyVal;
typedef boost::circular_buffer<geometry_msgs::Point> CbufGPoint;

struct WaypointReplannerConfig
{
  double velocity_max = 0.0;
  double velocity_min = 0.0;
  double velocity_param = 0.0;
  double accel_limit = 0.0;
  double decel_limit = 0.0;
  double radius_thresh = 0.0;
  double radius_min = 0.0;
  double radius_inf = 0.0;
  bool resample_mode = false;
  double resample_interval = 0.0;
  bool replan_curve_mode = false;
  bool replan_endpoint_mode = false;
  bool overwrite_vmax_mode = false;
  double velocity_offset = 0.0;
  double end_point_offset =  0.0;
  double braking_distance = 0.0;
  int lookup_crv_width = 5;
};

class WaypointReplanner
{
private:
  WaypointReplannerConfig config_;

public:
  WaypointReplanner();
  ~WaypointReplanner();
  void updateConfig(const WaypointReplannerConfig& config);
  void initParameter(const autoware_config_msgs::ConfigWaypointReplanner::ConstPtr& conf);
  void replanLaneWaypointVel(autoware_msgs::Lane& lane);

protected:
  void changeVelSign(autoware_msgs::Lane& lane, bool positive) const;
  void resampleLaneWaypoint(const double resample_interval, autoware_msgs::Lane& lane, LaneDirection dir);
  void resampleOnStraight(const CbufGPoint& curve_point, autoware_msgs::Lane& lane, LaneDirection dir);
  void resampleOnCurve(const geometry_msgs::Point& target_point,
    const std::vector<double>& param,autoware_msgs::Lane& lane, LaneDirection dir);

  const CbufGPoint getCrvPointsOnResample(const autoware_msgs::Lane& lane,
    const autoware_msgs::Lane& original_lane, unsigned long original_index) const;
  const CbufGPoint getCrvPoints(const autoware_msgs::Lane& lane, unsigned long index) const;

  void createRadiusList(const autoware_msgs::Lane& lane, std::vector<double>& curve_radius);
  const double calcVelParam(double vmax) const;
  void createCurveList(const std::vector<double>& curve_radius, KeyVal& curve_list);
  void createVmaxList(const autoware_msgs::Lane& lane, const KeyVal &curve_list,
    unsigned long offset, KeyVal &vmax_list);
  double searchVmaxByRange(unsigned long start_idx, unsigned long end_idx, unsigned int offset,
    const autoware_msgs::Lane &lane) const;
  void setVelocityByRange(unsigned long start_idx, unsigned long end_idx, unsigned int offset, double vel,
    autoware_msgs::Lane& lane);
  void raiseVelocityByRange(unsigned long start_idx, unsigned long end_idx, unsigned int offset,
    double vmin, autoware_msgs::Lane& lane);

  void limitVelocityByRange(unsigned long start_idx, unsigned long end_idx, unsigned int offset, double vmin,
    autoware_msgs::Lane& lane);
  void limitAccelDecel(const unsigned long idx, autoware_msgs::Lane& lane);

  const std::vector<double> calcCurveParam(CbufGPoint point) const;
  const double calcPathLength(const autoware_msgs::Lane& lane) const;
};
}
#endif

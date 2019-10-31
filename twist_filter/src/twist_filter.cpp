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

#include <iostream>
#include <boost/optional.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/ControlCommandStamped.h>
#include <autoware_config_msgs/ConfigTwistFilter.h>
#include <autoware_health_checker/health_checker/health_checker.h>

namespace
{
// const values
constexpr double MIN_LINEAR_X = 1e-3;
constexpr double MIN_LENGTH = 1e-3;
constexpr double MIN_DURATION = 1e-3;

auto lowpass_filter()
{
  double y = 0.0;
  return [y](const double& x, const double& gain) mutable -> double
  {
    y = gain * y + (1 - gain) * x;
    return y;
  };
}

struct StampedValue
{
  ros::Time time;
  double dt;
  double val;
  StampedValue() : time(0.0), dt(0.0), val(0.0) {}
  void reset()
  {
    time = ros::Time(0.0);
    val = 0.0;
  }
};

class TwistFilter
{
public:
  TwistFilter();
private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publishers
  ros::Publisher twist_pub_, ctrl_pub_;
  ros::Publisher twist_lacc_limit_debug_pub_, twist_ljerk_limit_debug_pub_;
  ros::Publisher ctrl_lacc_limit_debug_pub_, ctrl_ljerk_limit_debug_pub_;
  ros::Publisher twist_lacc_result_pub_, twist_ljerk_result_pub_;
  ros::Publisher ctrl_lacc_result_pub_, ctrl_ljerk_result_pub_;

  // subscribers
  ros::Subscriber twist_sub_, ctrl_sub_, config_sub_;

  // ros params
  double wheel_base_;
  double lateral_accel_limit_;
  double lateral_jerk_limit_;
  double lowpass_gain_linear_x_;
  double lowpass_gain_angular_z_;
  double lowpass_gain_steering_angle_;

  // dataset
  StampedValue az_prev_;
  StampedValue sa_prev_;

  // health_checker
  autoware_health_checker::HealthChecker health_checker_;

  boost::optional<double>
    calcLaccWithAngularZ(const double& lv, const double& az) const;
  boost::optional<double>
    calcLjerkWithAngularZ(const double& lv, const double& az) const;
  boost::optional<double>
    calcLaccWithSteeringAngle(const double& lv, const double& sa) const;
  boost::optional<double>
    calcLjerkWithSteeringAngle(const double& lv, const double& sa) const;
  void publishLateralResultsWithTwist(
    const geometry_msgs::TwistStamped& msg) const;
  void publishLateralResultsWithCtrl(
    const autoware_msgs::ControlCommandStamped& msg) const;
  void checkTwist(const geometry_msgs::TwistStamped& msg);
  void checkCtrl(const autoware_msgs::ControlCommandStamped& msg);
  geometry_msgs::TwistStamped
    lateralLimitTwist(const geometry_msgs::TwistStamped& msg);
  geometry_msgs::TwistStamped
    smoothTwist(const geometry_msgs::TwistStamped& msg);
  autoware_msgs::ControlCommandStamped
    lateralLimitCtrl(const autoware_msgs::ControlCommandStamped& msg);
  autoware_msgs::ControlCommandStamped
    smoothCtrl(const autoware_msgs::ControlCommandStamped& msg);
  void updatePrevTwist(const geometry_msgs::TwistStamped& msg);
  void updatePrevCtrl(const autoware_msgs::ControlCommandStamped& msg);
  void configCallback(
    const autoware_config_msgs::ConfigTwistFilterConstPtr& config);
  void TwistCmdCallback(const geometry_msgs::TwistStampedConstPtr& msg);
  void CtrlCmdCallback(const autoware_msgs::ControlCommandStampedConstPtr& msg);
};




boost::optional<double> TwistFilter::calcLaccWithAngularZ(
  const double& lv, const double& az) const
{
  return az * lv;
}

boost::optional<double> TwistFilter::calcLjerkWithAngularZ(
  const double& lv, const double& az) const
{
  if (std::fabs(az_prev_.dt) < MIN_DURATION)
  {
    return boost::none;
  }
  return (az - az_prev_.val) * lv / az_prev_.dt;
}

boost::optional<double> TwistFilter::calcLaccWithSteeringAngle(
  const double& lv, const double& sa) const
{
  if (std::fabs(wheel_base_) < MIN_LENGTH)
  {
    return boost::none;
  }
  return lv * lv * std::tan(sa) / wheel_base_;
}

boost::optional<double> TwistFilter::calcLjerkWithSteeringAngle(
  const double& lv, const double& sa) const
{
  if (std::fabs(sa_prev_.dt) < MIN_DURATION ||
    std::fabs(wheel_base_) < MIN_LENGTH)
  {
    return boost::none;
  }
  return lv * lv *
    ((std::tan(sa) - std::tan(sa_prev_.val)) / sa_prev_.dt) / wheel_base_;
}

void TwistFilter::publishLateralResultsWithTwist(
  const geometry_msgs::TwistStamped& msg) const
{
  const double lv = msg.twist.linear.x;
  const double az = msg.twist.angular.z;
  const auto lacc = calcLaccWithAngularZ(lv, az);
  const auto ljerk = calcLjerkWithAngularZ(lv, az);
  if (!lacc || !ljerk)
  {
    return;
  }
  std_msgs::Float32 lacc_msg, ljerk_msg;
  lacc_msg.data = lacc.get();
  ljerk_msg.data = ljerk.get();
  twist_lacc_result_pub_.publish(lacc_msg);
  twist_ljerk_result_pub_.publish(ljerk_msg);
}

void TwistFilter::publishLateralResultsWithCtrl(
  const autoware_msgs::ControlCommandStamped& msg) const
{
  const double lv = msg.cmd.linear_velocity;
  const double sa = msg.cmd.steering_angle;
  const auto lacc = calcLaccWithSteeringAngle(lv, sa);
  const auto ljerk = calcLjerkWithSteeringAngle(lv, sa);
  if (!lacc || !ljerk)
  {
    return;
  }
  std_msgs::Float32 lacc_msg, ljerk_msg;
  lacc_msg.data = lacc.get();
  ljerk_msg.data = ljerk.get();
  ctrl_lacc_result_pub_.publish(lacc_msg);
  ctrl_ljerk_result_pub_.publish(ljerk_msg);
}

void TwistFilter::checkTwist(const geometry_msgs::TwistStamped& msg)
{
  const double lv = msg.twist.linear.x;
  const double az = msg.twist.angular.z;
  const auto lacc = calcLaccWithAngularZ(lv, az);
  const auto ljerk = calcLjerkWithAngularZ(lv, az);
  if (lacc)
  {
    health_checker_.CHECK_MAX_VALUE("twist_lateral_accel_high",
      lacc.get(), lateral_accel_limit_, 2 * lateral_accel_limit_, DBL_MAX,
      "lateral_accel is too high in twist filtering");
  }
  if (ljerk)
  {
    health_checker_.CHECK_MAX_VALUE("twist_lateral_jerk_high",
      lacc.get(), lateral_jerk_limit_, 2 * lateral_jerk_limit_, DBL_MAX,
      "lateral_jerk is too high in twist filtering");
  }
}

void TwistFilter::checkCtrl(const autoware_msgs::ControlCommandStamped& msg)
{
  const double lv = msg.cmd.linear_velocity;
  const double sa = msg.cmd.steering_angle;
  const auto lacc = calcLaccWithSteeringAngle(lv, sa);
  const auto ljerk = calcLjerkWithSteeringAngle(lv, sa);
  if (lacc)
  {
    health_checker_.CHECK_MAX_VALUE("ctrl_lateral_accel_high",
      lacc.get(), lateral_accel_limit_, 3 * lateral_accel_limit_, DBL_MAX,
      "lateral_accel is too high in ctrl filtering");
  }
  if (ljerk)
  {
    health_checker_.CHECK_MAX_VALUE("ctrl_lateral_jerk_high",
      lacc.get(), lateral_jerk_limit_, 3 * lateral_jerk_limit_, DBL_MAX,
      "lateral_jerk is too high in ctrl filtering");
  }
}

geometry_msgs::TwistStamped
  TwistFilter::lateralLimitTwist(const geometry_msgs::TwistStamped& msg)
{
  static bool init = false;

  geometry_msgs::TwistStamped ts;
  ts = msg;

  ros::Time t = msg.header.stamp;
  az_prev_.dt = (t - az_prev_.time).toSec();
  const double lv = msg.twist.linear.x;
  double az = msg.twist.angular.z;

  // skip first msg, check linear_velocity
  const bool is_stopping = (std::fabs(lv) < MIN_LINEAR_X);
  if (!init || is_stopping)
  {
    init = true;
    return ts;
  }

  // lateral acceleration
  double lacc = calcLaccWithAngularZ(lv, az).get();
  // limit lateral acceleration
  if (std::fabs(lacc) > lateral_accel_limit_ && !is_stopping)
  {
    double sgn = lacc / std::fabs(lacc);
    double az_max = sgn * (lateral_accel_limit_) / lv;
    ROS_WARN_THROTTLE(1,
      "Limit angular velocity by lateral acceleration: %f -> %f", az, az_max);
    az = az_max;
  }

  // lateral jerk
  double ljerk = calcLjerkWithAngularZ(lv, az).get();
  // limit lateral jerk
  if (std::fabs(ljerk) > lateral_jerk_limit_ && !is_stopping)
  {
    double sgn = ljerk / std::fabs(ljerk);
    double az_max =
      az_prev_.val + (sgn * lateral_jerk_limit_ / lv) * az_prev_.dt;
    ROS_WARN_THROTTLE(1,
      "Limit angular velocity by lateral jerk: %f -> %f", az, az_max);
    az = az_max;
  }

  // update by lateral limitaion
  ts.twist.angular.z = az;

  // update lateral acceleration/jerk
  lacc = calcLaccWithAngularZ(lv, az).get();
  ljerk = calcLjerkWithAngularZ(lv, az).get();

  // for debug
  std_msgs::Float32 lacc_msg, ljerk_msg;
  lacc_msg.data = lacc;
  ljerk_msg.data = ljerk;
  twist_lacc_limit_debug_pub_.publish(lacc_msg);
  twist_ljerk_limit_debug_pub_.publish(ljerk_msg);

  return ts;
}

geometry_msgs::TwistStamped
  TwistFilter::smoothTwist(const geometry_msgs::TwistStamped& msg)
{
  static auto lp_lx = lowpass_filter();
  static auto lp_az = lowpass_filter();

  geometry_msgs::TwistStamped ts;
  ts = msg;

  // apply lowpass filter to linear_x / angular_z
  ts.twist.linear.x = lp_lx(ts.twist.linear.x, lowpass_gain_linear_x_);
  ts.twist.angular.z = lp_az(ts.twist.angular.z, lowpass_gain_angular_z_);

  return ts;
}

autoware_msgs::ControlCommandStamped
  TwistFilter::lateralLimitCtrl(const autoware_msgs::ControlCommandStamped& msg)
{
  static bool init = false;

  autoware_msgs::ControlCommandStamped ccs;
  ccs = msg;

  ros::Time t = msg.header.stamp;
  sa_prev_.dt = (t - sa_prev_.time).toSec();
  const double lv = msg.cmd.linear_velocity;
  double sa = msg.cmd.steering_angle;

  // skip first msg, check linear_velocity
  const bool is_stopping = (std::fabs(lv) < MIN_LINEAR_X);
  if (!init || is_stopping)
  {
    init = true;
    return ccs;
  }

  // lateral acceleration
  double lacc = calcLaccWithSteeringAngle(lv, sa).get();
  // limit lateral acceleration
  if (std::fabs(lacc) > lateral_accel_limit_ && !is_stopping)
  {
    double sgn = lacc / std::fabs(lacc);
    double sa_max =
      std::atan(sgn * lateral_accel_limit_ * wheel_base_ / (lv * lv));
    ROS_WARN_THROTTLE(1,
      "Limit steering angle by lateral acceleration: %f -> %f", sa, sa_max);
    sa = sa_max;
  }

  // lateral jerk
  double ljerk = calcLjerkWithSteeringAngle(lv, sa).get();
  // limit lateral jerk
  if (std::fabs(ljerk) > lateral_jerk_limit_ && !is_stopping)
  {
    double sgn = ljerk / std::fabs(ljerk);
    double sa_max = std::atan(std::tan(sa_prev_.val) +
      sgn * (lateral_jerk_limit_ * wheel_base_ / (lv * lv)) * sa_prev_.dt);
    ROS_WARN_THROTTLE(1,
      "Limit steering angle by lateral jerk: %f -> %f", sa, sa_max);
    sa = sa_max;
  }

  // update by lateral limitaion
  ccs.cmd.steering_angle = sa;

  // update lateral acceleration/jerk
  lacc = calcLaccWithSteeringAngle(lv, sa).get();
  ljerk = calcLjerkWithSteeringAngle(lv, sa).get();

  // for debug
  std_msgs::Float32 lacc_msg, ljerk_msg;
  lacc_msg.data = lacc;
  ljerk_msg.data = ljerk;
  ctrl_lacc_limit_debug_pub_.publish(lacc_msg);
  ctrl_ljerk_limit_debug_pub_.publish(ljerk_msg);

  return ccs;
}

autoware_msgs::ControlCommandStamped
  TwistFilter::smoothCtrl(const autoware_msgs::ControlCommandStamped& msg)
{
  static auto lp_lx = lowpass_filter();
  static auto lp_sa = lowpass_filter();

  autoware_msgs::ControlCommandStamped ccs;
  ccs = msg;

  // apply lowpass filter to linear_x / steering_angle
  ccs.cmd.linear_velocity =
    lp_lx(ccs.cmd.linear_velocity, lowpass_gain_linear_x_);
  ccs.cmd.steering_angle =
    lp_sa(ccs.cmd.steering_angle, lowpass_gain_steering_angle_);

  return ccs;
}

void TwistFilter::configCallback(
  const autoware_config_msgs::ConfigTwistFilterConstPtr& config)
{
  lateral_accel_limit_ = config->lateral_accel_limit;
  lateral_jerk_limit_ = config->lateral_jerk_limit;
  lowpass_gain_linear_x_ = config->lowpass_gain_linear_x;
  lowpass_gain_angular_z_ = config->lowpass_gain_angular_z;
  lowpass_gain_steering_angle_ = config->lowpass_gain_steering_angle;
}

void TwistFilter::updatePrevTwist(const geometry_msgs::TwistStamped& msg)
{
  az_prev_.time = msg.header.stamp;
  az_prev_.val = msg.twist.angular.z;
}

void TwistFilter::updatePrevCtrl(
  const autoware_msgs::ControlCommandStamped& msg)
{
  sa_prev_.time = msg.header.stamp;
  sa_prev_.val = msg.cmd.steering_angle;
}

void TwistFilter::TwistCmdCallback(
  const geometry_msgs::TwistStampedConstPtr& msg)
{
  health_checker_.NODE_ACTIVATE();
  checkTwist(*msg);
  geometry_msgs::TwistStamped ts;
  ts = lateralLimitTwist(*msg);
  ts = smoothTwist(ts);
  twist_pub_.publish(ts);
  publishLateralResultsWithTwist(ts);
  updatePrevTwist(ts);
}

void TwistFilter::CtrlCmdCallback(
  const autoware_msgs::ControlCommandStampedConstPtr& msg)
{
  health_checker_.NODE_ACTIVATE();
  checkCtrl(*msg);
  autoware_msgs::ControlCommandStamped ccs;
  ccs = lateralLimitCtrl(*msg);
  ccs = smoothCtrl(ccs);
  ctrl_pub_.publish(ccs);
  publishLateralResultsWithCtrl(ccs);
  updatePrevCtrl(ccs);
}

TwistFilter::TwistFilter()
  : nh_()
  , private_nh_("~")
  , health_checker_(nh_, private_nh_)
{
  nh_.param("vehicle_info/wheel_base", wheel_base_, 2.7);
  private_nh_.param("lateral_accel_limit", lateral_accel_limit_, 5.0);
  private_nh_.param("lateral_jerk_limit", lateral_jerk_limit_, 5.0);
  private_nh_.param("lowpass_gain_linear_x", lowpass_gain_linear_x_, 0.0);
  private_nh_.param("lowpass_gain_angular_z", lowpass_gain_angular_z_, 0.0);
  private_nh_.param(
    "lowpass_gain_steering_angle", lowpass_gain_steering_angle_, 0.0);

  twist_sub_ = nh_.subscribe(
    "twist_raw", 1, &TwistFilter::TwistCmdCallback, this);
  ctrl_sub_ = nh_.subscribe("ctrl_raw", 1, &TwistFilter::CtrlCmdCallback, this);
  config_sub_ = nh_.subscribe(
    "config/twist_filter", 10, &TwistFilter::configCallback, this);

  twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("twist_cmd", 5);
  ctrl_pub_ =
    nh_.advertise<autoware_msgs::ControlCommandStamped>("ctrl_cmd", 5);
  twist_lacc_limit_debug_pub_ = private_nh_.advertise<std_msgs::Float32>(
    "limitation_debug/twist/lateral_accel", 5);
  twist_ljerk_limit_debug_pub_ = private_nh_.advertise<std_msgs::Float32>(
    "limitation_debug/twist/lateral_jerk", 5);
  ctrl_lacc_limit_debug_pub_ = private_nh_.advertise<std_msgs::Float32>(
    "limitation_debug/ctrl/lateral_accel", 5);
  ctrl_ljerk_limit_debug_pub_ = private_nh_.advertise<std_msgs::Float32>(
    "limitation_debug/ctrl/lateral_jerk", 5);
  twist_lacc_result_pub_ = private_nh_.advertise<std_msgs::Float32>(
    "result/twist/lateral_accel", 5);
  twist_ljerk_result_pub_ = private_nh_.advertise<std_msgs::Float32>(
    "result/twist/lateral_jerk", 5);
  ctrl_lacc_result_pub_ = private_nh_.advertise<std_msgs::Float32>(
    "result/ctrl/lateral_accel", 5);
  ctrl_ljerk_result_pub_ = private_nh_.advertise<std_msgs::Float32>(
    "result/ctrl/lateral_jerk", 5);
  health_checker_.ENABLE();
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "twist_filter");
  TwistFilter twist_filter;
  ros::spin();
  return 0;
}

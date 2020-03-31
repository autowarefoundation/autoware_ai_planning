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

#include "twist_filter/twist_filter.h"
#include <cmath>
#include <limits>

namespace twist_filter
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

TwistFilter::TwistFilter(const Configuration& config) : config_(config)
{
}

boost::optional<double> TwistFilter::calcLaccWithAngularZ(const Twist& twist) const
{
  return twist.az * twist.lx;
}

boost::optional<double> TwistFilter::calcLjerkWithAngularZ(const Twist& twist, const Twist& twist_prev,
                                                           const double& dt) const
{
  if (std::fabs(dt) < MIN_DURATION)
  {
    return boost::none;
  }
  return (twist.az - twist_prev.az) * twist.lx / dt;
}

boost::optional<double> TwistFilter::calcLaccWithSteeringAngle(const Ctrl& ctrl) const
{
  if (std::fabs(config_.wheel_base) < MIN_LENGTH)
  {
    return boost::none;
  }
  return ctrl.lv * ctrl.lv * std::tan(ctrl.sa) / config_.wheel_base;
}

boost::optional<double> TwistFilter::calcLjerkWithSteeringAngle(const Ctrl& ctrl, const Ctrl& ctrl_prev,
                                                                const double& dt) const
{
  if (std::fabs(dt) < MIN_DURATION || std::fabs(config_.wheel_base) < MIN_LENGTH)
  {
    return boost::none;
  }
  return ctrl.lv * ctrl.lv * ((std::tan(ctrl.sa) - std::tan(ctrl_prev.sa)) / dt) / config_.wheel_base;
}

boost::optional<Twist> TwistFilter::lateralLimitTwist(const Twist& twist, const Twist& twist_prev,
                                                      const double& dt) const
{
  static bool init = false;

  Twist twist_out = twist;

  // skip first msg, check linear_velocity
  const bool is_stopping = (std::fabs(twist.lx) < MIN_LINEAR_X);
  if (!init || is_stopping)
  {
    init = true;
    return twist_out;
  }

  // lateral acceleration
  auto lacc_result = calcLaccWithAngularZ(twist_out);
  // limit lateral acceleration
  if (lacc_result)
  {
    if (std::fabs(lacc_result.get()) + std::numeric_limits<double>::epsilon() > config_.lateral_accel_limit &&
        !is_stopping)
    {
      double sgn = lacc_result.get() / std::fabs(lacc_result.get());
      double az_max = sgn * (config_.lateral_accel_limit) / twist_out.lx;
      twist_out.az = az_max;
    }
  }
  else
  {
    return boost::none;
  }

  // lateral jerk
  auto ljerk_result = calcLjerkWithAngularZ(twist_out, twist_prev, dt);
  // limit lateral jerk
  if (ljerk_result)
  {
    if (std::fabs(ljerk_result.get()) + std::numeric_limits<double>::epsilon() > config_.lateral_jerk_limit &&
        !is_stopping)
    {
      double sgn = ljerk_result.get() / std::fabs(ljerk_result.get());
      double az_max = twist_prev.az + (sgn * config_.lateral_jerk_limit / twist_out.lx) * dt;
      twist_out.az = az_max;
    }
  }
  else
  {
    return boost::none;
  }
  return twist_out;
}

Twist TwistFilter::smoothTwist(const Twist& twist) const
{
  static auto lp_lx = lowpass_filter();
  static auto lp_az = lowpass_filter();
  // apply lowpass filter to linear_x / angular_z
  Twist twist_out;
  twist_out.lx = lp_lx(twist.lx, config_.lowpass_gain_linear_x);
  twist_out.az = lp_az(twist.az, config_.lowpass_gain_angular_z);
  return twist_out;
}

boost::optional<Ctrl> TwistFilter::lateralLimitCtrl(const Ctrl& ctrl, const Ctrl& ctrl_prev, const double& dt) const
{
  static bool init = false;

  Ctrl ctrl_out = ctrl;

  // skip first msg, check linear_velocity
  const bool is_stopping = (std::fabs(ctrl.lv) < MIN_LINEAR_X);
  if (!init || is_stopping)
  {
    init = true;
    return ctrl_out;
  }

  // lateral acceleration
  auto lacc_result = calcLaccWithSteeringAngle(ctrl_out);
  // limit lateral acceleration
  if (lacc_result)
  {
    if (std::fabs(lacc_result.get()) + std::numeric_limits<double>::epsilon() > config_.lateral_accel_limit &&
        !is_stopping)
    {
      double sgn = lacc_result.get() / std::fabs(lacc_result.get());
      double sa_max = std::atan(sgn * config_.lateral_accel_limit * config_.wheel_base / (ctrl_out.lv * ctrl_out.lv));
      ctrl_out.sa = sa_max;
    }
  }
  else
  {
    return boost::none;
  }

  // lateral jerk
  auto ljerk_result = calcLjerkWithSteeringAngle(ctrl_out, ctrl_prev, dt);
  // limit lateral jerk
  if (ljerk_result)
  {
    if (std::fabs(ljerk_result.get()) + std::numeric_limits<double>::epsilon() > config_.lateral_jerk_limit &&
        !is_stopping)
    {
      double sgn = ljerk_result.get() / std::fabs(ljerk_result.get());
      double sa_max =
          std::atan(std::tan(ctrl_prev.sa) +
                    sgn * (config_.lateral_jerk_limit * config_.wheel_base / (ctrl_out.lv * ctrl_out.lv)) * dt);
      ctrl_out.sa = sa_max;
    }
  }
  else
  {
    return boost::none;
  }

  return ctrl_out;
}

Ctrl TwistFilter::smoothCtrl(const Ctrl& ctrl) const
{
  static auto lp_lx = lowpass_filter();
  static auto lp_sa = lowpass_filter();
  // apply lowpass filter to linear_x / steering_angle
  Ctrl ctrl_out;
  ctrl_out.lv = lp_lx(ctrl.lv, config_.lowpass_gain_linear_x);
  ctrl_out.sa = lp_sa(ctrl.sa, config_.lowpass_gain_steering_angle);
  return ctrl_out;
}

}  // namespace twist_filter

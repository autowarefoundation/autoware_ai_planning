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

#ifndef TWIST_FILTER_TWIST_FILTER_H
#define TWIST_FILTER_TWIST_FILTER_H

#include <iostream>
#include <boost/optional.hpp>

namespace twist_filter
{
typedef struct
{
  double wheel_base;
  double lateral_accel_limit;
  double lateral_jerk_limit;
  double lowpass_gain_linear_x;
  double lowpass_gain_angular_z;
  double lowpass_gain_steering_angle;
}
Configuration;

typedef struct
{
  double lx;
  double az;
}
Twist;

typedef struct
{
  double lv;
  double sa;
}
Ctrl;

class TwistFilter
{
public:
  explicit TwistFilter(const Configuration& config);

  boost::optional<double> calcLaccWithAngularZ(const Twist& twist) const;
  boost::optional<double> calcLjerkWithAngularZ(const Twist& twist, const Twist& twist_prev, const double& dt) const;
  boost::optional<double> calcLaccWithSteeringAngle(const Ctrl& ctrl) const;
  boost::optional<double> calcLjerkWithSteeringAngle(const Ctrl& ctrl, const Ctrl& ctrl_prev, const double& dt) const;

  boost::optional<Twist> lateralLimitTwist(const Twist& twist, const Twist& twist_prev, const double& dt) const;
  Twist smoothTwist(const Twist& twist) const;
  boost::optional<Ctrl> lateralLimitCtrl(const Ctrl& ctrl, const Ctrl& ctrl_prev, const double& dt) const;
  Ctrl smoothCtrl(const Ctrl& ctrl) const;

  void setConfiguration(const Configuration& config)
  {
    config_ = config;
  }

  const Configuration& getConfiguration() const
  {
    return config_;
  }

private:
  // params
  Configuration config_;
};

}  // namespace twist_filter

#endif  // TWIST_FILTER_TWIST_FILTER_H

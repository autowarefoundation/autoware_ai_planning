
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

#include "wf_simulator/vehicle_model/wfsim_model_constant_acceleration.hpp"

WFSimModelConstantAccelTwist::WFSimModelConstantAccelTwist(double vx_lim, double wz_lim, double vx_rate, double wz_rate)
  : WFSimModelInterface(5 /* dim x */, 2 /* dim u */)
  , vx_lim_(vx_lim)
  , wz_lim_(wz_lim)
  , vx_rate_(vx_rate)
  , wz_rate_(wz_rate){};

double WFSimModelConstantAccelTwist::getX()
{
  return state_(IDX::X);
};
double WFSimModelConstantAccelTwist::getY()
{
  return state_(IDX::Y);
};
double WFSimModelConstantAccelTwist::getYaw()
{
  return state_(IDX::YAW);
};
double WFSimModelConstantAccelTwist::getVx()
{
  return state_(IDX::VX);
};
double WFSimModelConstantAccelTwist::getWz()
{
  return state_(IDX::WZ);
};
double WFSimModelConstantAccelTwist::getSteer()
{
  return 0.0;
};
Eigen::VectorXd WFSimModelConstantAccelTwist::calcModel(const Eigen::VectorXd& state, const Eigen::VectorXd& input)
{
  const double vel = state(IDX::VX);
  const double angvel = state(IDX::WZ);
  const double yaw = state(IDX::YAW);
  const double vx_des = std::max(std::min(input(IDX_U::VX_DES), vx_lim_), -vx_lim_);
  const double wz_des = std::max(std::min(input(IDX_U::WZ_DES), wz_lim_), -wz_lim_);
  double vx_rate = 0.0;
  double wz_rate = 0.0;
  if (vx_des > vel)
  {
    vx_rate = vx_rate_;
  }
  else if (vx_des < vel)
  {
    vx_rate = -vx_rate_;
  }

  if (wz_des > angvel)
  {
    wz_rate = wz_rate_;
  }
  else if (wz_des < angvel)
  {
    wz_rate = -wz_rate_;
  }

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = vel * cos(yaw);
  d_state(IDX::Y) = vel * sin(yaw);
  d_state(IDX::YAW) = angvel;
  d_state(IDX::VX) = vx_rate;
  d_state(IDX::WZ) = wz_rate;

  return d_state;
};
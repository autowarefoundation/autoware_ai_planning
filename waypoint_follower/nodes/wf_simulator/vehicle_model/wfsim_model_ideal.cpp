
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

#include "wf_simulator/vehicle_model/wfsim_model_ideal.hpp"

WFSimModelIdealTwist::WFSimModelIdealTwist() : WFSimModelInterface(3 /* dim x */, 2 /* dim u */){};

double WFSimModelIdealTwist::getX()
{
  return state_(IDX::X);
};
double WFSimModelIdealTwist::getY()
{
  return state_(IDX::Y);
};
double WFSimModelIdealTwist::getYaw()
{
  return state_(IDX::YAW);
};
double WFSimModelIdealTwist::getVx()
{
  return input_(IDX_U::VX_DES);
};
double WFSimModelIdealTwist::getWz()
{
  return input_(IDX_U::WZ_DES);
};
double WFSimModelIdealTwist::getSteer()
{
  return 0.0;
};

Eigen::VectorXd WFSimModelIdealTwist::calcModel(const Eigen::VectorXd& state, const Eigen::VectorXd& input)
{
  const double yaw = state(IDX::YAW);
  const double vx = input(IDX_U::VX_DES);
  const double wz = input(IDX_U::WZ_DES);

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = vx * cos(yaw);
  d_state(IDX::Y) = vx * sin(yaw);
  d_state(IDX::YAW) = wz;

  return d_state;
};

WFSimModelIdealSteer::WFSimModelIdealSteer(double wheelbase)
  : WFSimModelInterface(3 /* dim x */, 2 /* dim u */), wheelbase_(wheelbase){};

double WFSimModelIdealSteer::getX()
{
  return state_(IDX::X);
};
double WFSimModelIdealSteer::getY()
{
  return state_(IDX::Y);
};
double WFSimModelIdealSteer::getYaw()
{
  return state_(IDX::YAW);
};
double WFSimModelIdealSteer::getVx()
{
  return input_(IDX_U::VX_DES);
};
double WFSimModelIdealSteer::getWz()
{
  return input_(IDX_U::VX_DES) * std::tan(input_(IDX_U::STEER_DES)) / wheelbase_;
};
double WFSimModelIdealSteer::getSteer()
{
  return input_(IDX_U::STEER_DES);
};

Eigen::VectorXd WFSimModelIdealSteer::calcModel(const Eigen::VectorXd& state, const Eigen::VectorXd& input)
{
  const double yaw = state(IDX::YAW);
  const double vx = input(IDX_U::VX_DES);
  const double steer = input(IDX_U::STEER_DES);

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = vx * cos(yaw);
  d_state(IDX::Y) = vx * sin(yaw);
  d_state(IDX::YAW) = vx * std::tan(steer) / wheelbase_;

  return d_state;
};
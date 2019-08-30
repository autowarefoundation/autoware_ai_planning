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

#include "wf_simulator/vehicle_model/wfsim_model_interface.hpp"

WFSimModelInterface::WFSimModelInterface(int dim_x, int dim_u) : dim_x_(dim_x), dim_u_(dim_u)
{
  state_ = Eigen::VectorXd::Zero(dim_x_);
  input_ = Eigen::VectorXd::Zero(dim_u_);
};

void WFSimModelInterface::updateRungeKutta(const double& dt)
{
  Eigen::VectorXd k1 = calcModel(state_, input_);
  Eigen::VectorXd k2 = calcModel(state_ + k1 * 0.5 * dt, input_);
  Eigen::VectorXd k3 = calcModel(state_ + k2 * 0.5 * dt, input_);
  Eigen::VectorXd k4 = calcModel(state_ + k3 * dt, input_);

  state_ += 1.0 / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4) * dt;
}
void WFSimModelInterface::updateEuler(const double& dt)
{
  state_ += calcModel(state_, input_) * dt;
}
void WFSimModelInterface::getState(Eigen::VectorXd& state)
{
  state = state_;
};
void WFSimModelInterface::getInput(Eigen::VectorXd& input)
{
  input = input_;
};
void WFSimModelInterface::setState(const Eigen::VectorXd& state)
{
  state_ = state;
};
void WFSimModelInterface::setInput(const Eigen::VectorXd& input)
{
  input_ = input;
};

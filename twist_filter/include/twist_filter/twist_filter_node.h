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

#ifndef TWIST_FILTER_TWIST_FILTER_NODE_H
#define TWIST_FILTER_TWIST_FILTER_NODE_H

#include "twist_filter/twist_filter.h"
#include <memory>
#include <std_msgs/Float32.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_health_checker/health_checker/health_checker.h>
#include <autoware_msgs/ControlCommandStamped.h>
#include <autoware_config_msgs/ConfigTwistFilter.h>

namespace twist_filter_node
{
class TwistFilterNode
{
public:
  TwistFilterNode();
  ~TwistFilterNode() = default;

private:
  ros::NodeHandle nh_, pnh_;
  std::unique_ptr<twist_filter::TwistFilter> twist_filter_ptr_;
  autoware_health_checker::HealthChecker health_checker_;
  bool enable_smoothing_ = false;
  bool enable_debug_ = false;

  // publishers
  ros::Publisher twist_pub_, ctrl_pub_;
  ros::Publisher twist_lacc_limit_debug_pub_, twist_ljerk_limit_debug_pub_;
  ros::Publisher ctrl_lacc_limit_debug_pub_, ctrl_ljerk_limit_debug_pub_;
  ros::Publisher twist_lacc_result_pub_, twist_ljerk_result_pub_;
  ros::Publisher ctrl_lacc_result_pub_, ctrl_ljerk_result_pub_;

  // subscribers
  ros::Subscriber twist_sub_, ctrl_sub_, config_sub_;

  void configCallback(const autoware_config_msgs::ConfigTwistFilterConstPtr& config);
  void twistCmdCallback(const geometry_msgs::TwistStampedConstPtr& msg);
  void ctrlCmdCallback(const autoware_msgs::ControlCommandStampedConstPtr& msg);
  void checkTwist(const twist_filter::Twist twist, const twist_filter::Twist twist_prev, const double& dt);
  void checkCtrl(const twist_filter::Ctrl ctrl, const twist_filter::Ctrl ctrl_prev, const double& dt);
};

}  // namespace twist_filter_node

#endif  // TWIST_FILTER_TWIST_FILTER_NODE_H

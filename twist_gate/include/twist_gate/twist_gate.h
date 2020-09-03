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

#ifndef TWIST_GATE_TWIST_GATE_H
#define TWIST_GATE_TWIST_GATE_H

#include <string>
#include <iostream>
#include <map>
#include <thread>
#include <memory>

#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include "autoware_config_msgs/ConfigTwistFilter.h"
#include "autoware_msgs/ControlCommandStamped.h"
#include "autoware_msgs/VehicleCmd.h"
#include "autoware_msgs/Gear.h"

#include <autoware_health_checker/health_checker/health_checker.h>


class TwistGate
{
  using vehicle_cmd_msg_t = autoware_msgs::VehicleCmd;

  friend class TwistGateTestClass;

public:
  TwistGate(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

private:
  void twistCmdCallback(const geometry_msgs::TwistStamped::ConstPtr& input_msg);
  void ctrlCmdCallback(const autoware_msgs::ControlCommandStamped::ConstPtr& input_msg);
  void lampCmdCallback(const autoware_msgs::LampCmd::ConstPtr& input_msg);
  void stateCallback(const std_msgs::StringConstPtr& input_msg);
  void emergencyCmdCallback(const vehicle_cmd_msg_t::ConstPtr& input_msg);
  void updateEmergencyState();

  void updateStateAndPublish();
  void configCallback(const autoware_config_msgs::ConfigTwistFilter& msg);

  // spinOnce for test
  void spinOnce() { ros::spinOnce(); }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::shared_ptr<autoware_health_checker::HealthChecker> health_checker_ptr_;
  ros::Publisher control_command_pub_;
  ros::Publisher vehicle_cmd_pub_;
  ros::Subscriber config_sub_;
  std::map<std::string, ros::Subscriber> auto_cmd_sub_stdmap_;

  vehicle_cmd_msg_t output_msg_;
  std_msgs::Bool emergency_stop_msg_;
  ros::Time emergency_handling_time_;
  ros::Duration timeout_period_;

  bool is_state_drive_ = false;
  bool use_decision_maker_ = false;

  bool emergency_handling_active_ = false;
};

#endif  // TWIST_GATE_TWIST_GATE_H

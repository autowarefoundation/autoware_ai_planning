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
#include "autoware_msgs/RemoteCmd.h"
#include "autoware_msgs/VehicleCmd.h"
#include "autoware_msgs/Gear.h"

#include "tablet_socket_msgs/gear_cmd.h"
#include "tablet_socket_msgs/mode_cmd.h"

// headers in Autowae Health Checker
#include <autoware_health_checker/health_checker/health_checker.h>


class TwistGate
{
  using remote_msgs_t = autoware_msgs::RemoteCmd;
  using vehicle_cmd_msg_t = autoware_msgs::VehicleCmd;

  friend class TwistGateTestClass;

public:
  TwistGate(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  ~TwistGate();

private:
  void checkState();
  void watchdogTimer();
  void remoteCmdCallback(const remote_msgs_t::ConstPtr& input_msg);
  void autoCmdTwistCmdCallback(const geometry_msgs::TwistStamped::ConstPtr& input_msg);
  void modeCmdCallback(const tablet_socket_msgs::mode_cmd::ConstPtr& input_msg);
  void gearCmdCallback(const tablet_socket_msgs::gear_cmd::ConstPtr& input_msg);
  void accelCmdCallback(const autoware_msgs::AccelCmd::ConstPtr& input_msg);
  void steerCmdCallback(const autoware_msgs::SteerCmd::ConstPtr& input_msg);
  void brakeCmdCallback(const autoware_msgs::BrakeCmd::ConstPtr& input_msg);
  void lampCmdCallback(const autoware_msgs::LampCmd::ConstPtr& input_msg);
  void ctrlCmdCallback(const autoware_msgs::ControlCommandStamped::ConstPtr& input_msg);
  void stateCallback(const std_msgs::StringConstPtr& input_msg);
  void emergencyCmdCallback(const vehicle_cmd_msg_t::ConstPtr& input_msg);
  void timerCallback(const ros::TimerEvent& e);
  void configCallback(const autoware_config_msgs::ConfigTwistFilter& msg);

  void resetVehicleCmdMsg();

  // spinOnce for test
  void spinOnce() { ros::spinOnce(); }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::shared_ptr<autoware_health_checker::HealthChecker> health_checker_ptr_;
  ros::Publisher control_command_pub_;
  ros::Publisher vehicle_cmd_pub_;
  ros::Subscriber remote_cmd_sub_;
  ros::Subscriber config_sub_;
  std::map<std::string, ros::Subscriber> auto_cmd_sub_stdmap_;
  ros::Timer timer_;

  vehicle_cmd_msg_t twist_gate_msg_;
  std_msgs::Bool emergency_stop_msg_;
  ros::Time remote_cmd_time_, emergency_handling_time_;
  ros::Time state_time_;
  ros::Duration timeout_period_;
  double loop_rate_;

  std::thread watchdog_timer_thread_;
  bool is_alive;

  enum class CommandMode
  {
    AUTO = 1,
    REMOTE = 2
  }
  command_mode_,

  previous_command_mode_;
  std_msgs::String command_mode_topic_;

  bool is_state_drive_ = false;
  bool use_decision_maker_ = false;

  bool emergency_handling_active_ = false;
};

#endif  // TWIST_GATE_TWIST_GATE_H

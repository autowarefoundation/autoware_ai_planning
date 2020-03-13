/*
 *  Copyright (c) 2017, Tier IV, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "twist_gate/twist_gate.h"
#include <ros_observer/lib_ros_observer.h>

#include <chrono>
#include <string>
#include <autoware_system_msgs/DiagnosticStatus.h>

using AwDiagStatus = autoware_system_msgs::DiagnosticStatus;

TwistGate::TwistGate(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
  : nh_(nh)
  , private_nh_(private_nh)
  , timeout_period_(10.0)
  , command_mode_(CommandMode::AUTO)
  , previous_command_mode_(CommandMode::AUTO)
{
  private_nh_.param<double>("loop_rate", loop_rate_, 30.0);
  private_nh_.param<bool>("use_decision_maker", use_decision_maker_, false);

  health_checker_ptr_ = std::make_shared<autoware_health_checker::HealthChecker>(nh_, private_nh_);
  control_command_pub_ = nh_.advertise<std_msgs::String>("/ctrl_mode", 1);
  vehicle_cmd_pub_ = nh_.advertise<vehicle_cmd_msg_t>("/vehicle_cmd", 1, true);
  remote_cmd_sub_ = nh_.subscribe("/remote_cmd", 1, &TwistGate::remoteCmdCallback, this);
  config_sub_ = nh_.subscribe("config/twist_filter", 1, &TwistGate::configCallback, this);

  timer_ = nh_.createTimer(ros::Duration(1.0 / loop_rate_), &TwistGate::timerCallback, this);

  auto_cmd_sub_stdmap_["twist_cmd"] = nh_.subscribe("/twist_cmd", 1, &TwistGate::autoCmdTwistCmdCallback, this);
  auto_cmd_sub_stdmap_["mode_cmd"] = nh_.subscribe("/mode_cmd", 1, &TwistGate::modeCmdCallback, this);
  auto_cmd_sub_stdmap_["gear_cmd"] = nh_.subscribe("/gear_cmd", 1, &TwistGate::gearCmdCallback, this);
  auto_cmd_sub_stdmap_["accel_cmd"] = nh_.subscribe("/accel_cmd", 1, &TwistGate::accelCmdCallback, this);
  auto_cmd_sub_stdmap_["steer_cmd"] = nh_.subscribe("/steer_cmd", 1, &TwistGate::steerCmdCallback, this);
  auto_cmd_sub_stdmap_["brake_cmd"] = nh_.subscribe("/brake_cmd", 1, &TwistGate::brakeCmdCallback, this);
  auto_cmd_sub_stdmap_["lamp_cmd"] = nh_.subscribe("/lamp_cmd", 1, &TwistGate::lampCmdCallback, this);
  auto_cmd_sub_stdmap_["ctrl_cmd"] = nh_.subscribe("/ctrl_cmd", 1, &TwistGate::ctrlCmdCallback, this);
  auto_cmd_sub_stdmap_["state"] = nh_.subscribe("/decision_maker/state", 1, &TwistGate::stateCallback, this);
  auto_cmd_sub_stdmap_["emergency_velocity"] =
      nh_.subscribe("emergency_velocity", 1, &TwistGate::emergencyCmdCallback, this);

  twist_gate_msg_.header.seq = 0;
  emergency_stop_msg_.data = false;
  health_checker_ptr_->ENABLE();
  health_checker_ptr_->NODE_ACTIVATE();

  remote_cmd_time_ = ros::Time::now();
  emergency_handling_time_ = ros::Time::now();
  state_time_ = ros::Time::now();
  watchdog_timer_thread_ = std::thread(&TwistGate::watchdogTimer, this);
  is_alive = true;
}

TwistGate::~TwistGate()
{
  is_alive = false;
  watchdog_timer_thread_.join();
}

void TwistGate::resetVehicleCmdMsg()
{
  twist_gate_msg_.twist_cmd.twist.linear.x = 0;
  twist_gate_msg_.twist_cmd.twist.angular.z = 0;
  twist_gate_msg_.mode = 0;
  twist_gate_msg_.gear_cmd.gear = autoware_msgs::Gear::NONE;
  twist_gate_msg_.lamp_cmd.l = 0;
  twist_gate_msg_.lamp_cmd.r = 0;
  twist_gate_msg_.accel_cmd.accel = 0;
  twist_gate_msg_.brake_cmd.brake = 0;
  twist_gate_msg_.steer_cmd.steer = 0;
  twist_gate_msg_.ctrl_cmd.linear_velocity = -1;
  twist_gate_msg_.ctrl_cmd.steering_angle = 0;
  twist_gate_msg_.emergency = 0;
}

void TwistGate::checkState()
{
  const double state_msg_timeout = 0.5;
  double state_time_diff = ros::Time::now().toSec() - state_time_.toSec();

  if (use_decision_maker_ && (!is_state_drive_ || state_time_diff >= state_msg_timeout) )
  {
    twist_gate_msg_.twist_cmd.twist = geometry_msgs::Twist();
    twist_gate_msg_.ctrl_cmd = autoware_msgs::ControlCommand();
  }
}

void TwistGate::watchdogTimer()
{
  ShmVitalMonitor shm_vmon("TwistGate", 100);

  while (is_alive)
  {
    ros::Time now_time = ros::Time::now();

    // check command mode
    if (previous_command_mode_ != command_mode_)
    {
      if (command_mode_ == CommandMode::AUTO)
      {
        command_mode_topic_.data = "AUTO";
      }
      else if (command_mode_ == CommandMode::REMOTE)
      {
        command_mode_topic_.data = "REMOTE";
      }
      else
      {
        command_mode_topic_.data = "UNDEFINED";
      }

      control_command_pub_.publish(command_mode_topic_);
      previous_command_mode_ = command_mode_;
    }

    shm_vmon.run();

    // check lost Communication
    if (command_mode_ == CommandMode::REMOTE)
    {
      const double dt = (now_time - remote_cmd_time_).toSec() * 1000;
      health_checker_ptr_->CHECK_MAX_VALUE("remote_cmd_interval", dt, 700, 1000, 1500, "remote cmd interval is too "
                                                                                       "long.");
    }

    // check push emergency stop button
    const int level = (emergency_stop_msg_.data) ? AwDiagStatus::ERROR : AwDiagStatus::OK;
    health_checker_ptr_->CHECK_TRUE("emergency_stop_button", emergency_stop_msg_.data, level, "emergency stop button "
                                                                                              "is pushed.");

    // if no emergency message received for more than timeout_period_
    if ((now_time - emergency_handling_time_) > timeout_period_)
    {
      emergency_handling_active_ = false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void TwistGate::remoteCmdCallback(const remote_msgs_t::ConstPtr& input_msg)
{
  remote_cmd_time_ = ros::Time::now();
  command_mode_ = static_cast<CommandMode>(input_msg->control_mode);
  emergency_stop_msg_.data = static_cast<bool>(input_msg->vehicle_cmd.emergency);

  // Update Emergency Mode
  twist_gate_msg_.emergency = input_msg->vehicle_cmd.emergency;
  if (command_mode_ == CommandMode::REMOTE && emergency_stop_msg_.data == false)
  {
    twist_gate_msg_.header.frame_id = input_msg->vehicle_cmd.header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->vehicle_cmd.header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.twist_cmd.twist = input_msg->vehicle_cmd.twist_cmd.twist;
    twist_gate_msg_.ctrl_cmd = input_msg->vehicle_cmd.ctrl_cmd;
    twist_gate_msg_.accel_cmd = input_msg->vehicle_cmd.accel_cmd;
    twist_gate_msg_.brake_cmd = input_msg->vehicle_cmd.brake_cmd;
    twist_gate_msg_.steer_cmd = input_msg->vehicle_cmd.steer_cmd;
    twist_gate_msg_.gear_cmd.gear = input_msg->vehicle_cmd.gear_cmd.gear;
    twist_gate_msg_.lamp_cmd = input_msg->vehicle_cmd.lamp_cmd;
    twist_gate_msg_.mode = input_msg->vehicle_cmd.mode;
  }
}

void TwistGate::autoCmdTwistCmdCallback(const geometry_msgs::TwistStamped::ConstPtr& input_msg)
{
  health_checker_ptr_->CHECK_RATE("topic_rate_twist_cmd_slow", 8, 5, 1, "topic twist_cmd subscribe rate slow.");
  health_checker_ptr_->CHECK_MAX_VALUE("twist_cmd_linear_high", input_msg->twist.linear.x,
    DBL_MAX, DBL_MAX, DBL_MAX, "linear twist_cmd is too high");

  if (command_mode_ == CommandMode::AUTO && !emergency_handling_active_)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.twist_cmd.twist = input_msg->twist;

    checkState();
  }
}

void TwistGate::modeCmdCallback(const tablet_socket_msgs::mode_cmd::ConstPtr& input_msg)
{
  if (command_mode_ == CommandMode::AUTO && !emergency_handling_active_)
  {
    // TODO(unknown): check this if statement
    if (input_msg->mode == -1 || input_msg->mode == 0)
    {
      resetVehicleCmdMsg();
    }
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.mode = input_msg->mode;
  }
}

void TwistGate::gearCmdCallback(const tablet_socket_msgs::gear_cmd::ConstPtr& input_msg)
{
  if (command_mode_ == CommandMode::AUTO && !emergency_handling_active_)
  {
    twist_gate_msg_.gear_cmd.gear = input_msg->gear;
  }
}

void TwistGate::accelCmdCallback(const autoware_msgs::AccelCmd::ConstPtr& input_msg)
{
  if (command_mode_ == CommandMode::AUTO && !emergency_handling_active_)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.accel_cmd.accel = input_msg->accel;
  }
}

void TwistGate::steerCmdCallback(const autoware_msgs::SteerCmd::ConstPtr& input_msg)
{
  if (command_mode_ == CommandMode::AUTO && !emergency_handling_active_)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.steer_cmd.steer = input_msg->steer;
  }
}

void TwistGate::brakeCmdCallback(const autoware_msgs::BrakeCmd::ConstPtr& input_msg)
{
  if (command_mode_ == CommandMode::AUTO && !emergency_handling_active_)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.brake_cmd.brake = input_msg->brake;
  }
}

void TwistGate::lampCmdCallback(const autoware_msgs::LampCmd::ConstPtr& input_msg)
{
  if (command_mode_ == CommandMode::AUTO && !emergency_handling_active_)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.lamp_cmd.l = input_msg->l;
    twist_gate_msg_.lamp_cmd.r = input_msg->r;
  }
}

void TwistGate::ctrlCmdCallback(const autoware_msgs::ControlCommandStamped::ConstPtr& input_msg)
{
  if (command_mode_ == CommandMode::AUTO && !emergency_handling_active_)
  {
    twist_gate_msg_.header.frame_id = input_msg->header.frame_id;
    twist_gate_msg_.header.stamp = input_msg->header.stamp;
    twist_gate_msg_.header.seq++;
    twist_gate_msg_.ctrl_cmd = input_msg->cmd;

    checkState();
  }
}

void TwistGate::stateCallback(const std_msgs::StringConstPtr& input_msg)
{
  state_time_ = ros::Time::now();
  if (command_mode_ == CommandMode::AUTO && !emergency_handling_active_)
  {
    // Set Parking Gear
    if (input_msg->data.find("WaitOrder") != std::string::npos)
    {
      twist_gate_msg_.gear_cmd.gear = autoware_msgs::Gear::PARK;
    }
    // Set Drive Gear
    else
    {
      twist_gate_msg_.gear_cmd.gear = autoware_msgs::Gear::DRIVE;
    }

    // get drive state
    if (input_msg->data.find("Drive\n") != std::string::npos &&
        input_msg->data.find("VehicleReady\n") != std::string::npos)
    {
      is_state_drive_ = true;
    }
    else
    {
      is_state_drive_ = false;
    }

    // reset emergency flags
    if (input_msg->data.find("VehicleReady") != std::string::npos)
    {
      emergency_stop_msg_.data = false;
    }
  }
}

void TwistGate::emergencyCmdCallback(const vehicle_cmd_msg_t::ConstPtr& input_msg)
{
  emergency_handling_time_ = ros::Time::now();
  emergency_handling_active_ = true;
  twist_gate_msg_ = *input_msg;
}

void TwistGate::timerCallback(const ros::TimerEvent& e)
{
  vehicle_cmd_pub_.publish(twist_gate_msg_);
}

void TwistGate::configCallback(const autoware_config_msgs::ConfigTwistFilter& msg)
{
  use_decision_maker_ = msg.use_decision_maker;
}

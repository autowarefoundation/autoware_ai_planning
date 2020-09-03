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
{
  private_nh_.param<bool>("use_decision_maker", use_decision_maker_, false);

  health_checker_ptr_ = std::make_shared<autoware_health_checker::HealthChecker>(nh_, private_nh_);
  control_command_pub_ = nh_.advertise<std_msgs::String>("/ctrl_mode", 1);
  vehicle_cmd_pub_ = nh_.advertise<vehicle_cmd_msg_t>("/vehicle_cmd", 1, true);
  config_sub_ = nh_.subscribe("config/twist_filter", 1, &TwistGate::configCallback, this);

  auto_cmd_sub_stdmap_["twist_cmd"] = nh_.subscribe("/twist_cmd", 1, &TwistGate::twistCmdCallback, this);
  auto_cmd_sub_stdmap_["ctrl_cmd"] = nh_.subscribe("/ctrl_cmd", 1, &TwistGate::ctrlCmdCallback, this);
  auto_cmd_sub_stdmap_["lamp_cmd"] = nh_.subscribe("/lamp_cmd", 1, &TwistGate::lampCmdCallback, this);
  auto_cmd_sub_stdmap_["state"] = nh_.subscribe("/decision_maker/state", 1, &TwistGate::stateCallback, this);
  auto_cmd_sub_stdmap_["emergency_velocity"] =
      nh_.subscribe("emergency_velocity", 1, &TwistGate::emergencyCmdCallback, this);

  output_msg_.header.seq = 0;
  emergency_stop_msg_.data = false;
  health_checker_ptr_->ENABLE();
  health_checker_ptr_->NODE_ACTIVATE();

  emergency_handling_time_ = ros::Time::now();
}

void TwistGate::twistCmdCallback(const geometry_msgs::TwistStamped::ConstPtr& input_msg)
{
  health_checker_ptr_->CHECK_RATE("topic_rate_twist_cmd_slow", 8, 5, 1, "topic twist_cmd subscribe rate slow.");
  health_checker_ptr_->CHECK_MAX_VALUE("twist_cmd_linear_high", input_msg->twist.linear.x,
    DBL_MAX, DBL_MAX, DBL_MAX, "linear twist_cmd is too high");

  updateEmergencyState();
  if (!emergency_handling_active_)
  {
    output_msg_.header.frame_id = input_msg->header.frame_id;
    output_msg_.header.stamp = input_msg->header.stamp;
    output_msg_.header.seq++;
    output_msg_.twist_cmd.twist = input_msg->twist;

    updateStateAndPublish();
  }
}

void TwistGate::ctrlCmdCallback(const autoware_msgs::ControlCommandStamped::ConstPtr& input_msg)
{
  updateEmergencyState();
  if (!emergency_handling_active_)
  {
    output_msg_.header.frame_id = input_msg->header.frame_id;
    output_msg_.header.stamp = input_msg->header.stamp;
    output_msg_.header.seq++;
    output_msg_.ctrl_cmd = input_msg->cmd;

    updateStateAndPublish();
  }
}

void TwistGate::lampCmdCallback(const autoware_msgs::LampCmd::ConstPtr& input_msg)
{
  updateEmergencyState();
  if (!emergency_handling_active_)
  {
    output_msg_.lamp_cmd.l = input_msg->l;
    output_msg_.lamp_cmd.r = input_msg->r;
  }
}

void TwistGate::stateCallback(const std_msgs::StringConstPtr& input_msg)
{
  if (!emergency_handling_active_)
  {
    // Set Parking Gear
    if (input_msg->data.find("WaitOrder") != std::string::npos)
    {
      output_msg_.gear_cmd.gear = autoware_msgs::Gear::PARK;
    }
    // Set Drive Gear
    else
    {
      output_msg_.gear_cmd.gear = autoware_msgs::Gear::DRIVE;
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
  output_msg_ = *input_msg;

  updateStateAndPublish();
}

void TwistGate::updateEmergencyState()
{
  // Reset emergency handling
  if (emergency_handling_active_)
  {
    // If no emergency message received for more than timeout_period_
    if ((ros::Time::now() - emergency_handling_time_) > timeout_period_)
    {
      emergency_handling_active_ = false;
    }
  }
}

void TwistGate::updateStateAndPublish()
{
  // Clear commands if we aren't in drive state
  // ssc_interface will handle autonomy disengagements if the control commands timeout
  if (use_decision_maker_ && (!is_state_drive_))
  {
    output_msg_.twist_cmd.twist = geometry_msgs::Twist();
    output_msg_.ctrl_cmd = autoware_msgs::ControlCommand();
  }

  vehicle_cmd_pub_.publish(output_msg_);
}

void TwistGate::configCallback(const autoware_config_msgs::ConfigTwistFilter& msg)
{
  use_decision_maker_ = msg.use_decision_maker;
}

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

#include <ros/ros.h>
#include <gtest/gtest.h>
#include "twist_filter/twist_filter.h"
#include "twist_filter/twist_filter_node.h"

class TestSuite : public ::testing::Test
{
public:
  TestSuite() : nh_(), private_nh_("~")
  {
    // Raw velocity publisher
    twist_raw_publisher_ = nh_.advertise<geometry_msgs::TwistStamped>("twist_raw", 1, true);
    ctrl_raw_publisher_ = nh_.advertise<autoware_msgs::ControlCommandStamped>("ctrl_raw", 1, true);

    // Setup subscriber
    twist_cmd_subscriber_ = nh_.subscribe("twist_cmd", 1, &TestSuite::callbackTwistCmd, this);
    ctrl_cmd_subscriber_ = nh_.subscribe("ctrl_cmd", 1, &TestSuite::callbackCtrlCmd, this);
  }

  void setupParameters()
  {
    nh_.setParam("vehicle_info/wheel_base", 2.7);
    nh_.setParam("twist_filter/lateral_accel_limit", 5.0);
    nh_.setParam("twist_filter/lateral_jerk_limit", 5.0);
    nh_.setParam("twist_filter/lowpass_gain_linear_x", 0.0);
    nh_.setParam("twist_filter/lowpass_gain_angular_z", 0.0);
    nh_.setParam("twist_filter/lowpass_gain_steering_angle", 0.0);
  }

  void testTwistFiltering()
  {
    twist_filter_node::TwistFilterNode node;

    geometry_msgs::TwistStamped twist_raw;

    twist_raw.twist.linear.x = 1.0;
    twist_raw.twist.angular.z = 1.0;
    twist_raw_publisher_.publish(twist_raw);
    spinWhile(0.3);

    // Assert received value is not changing
    ASSERT_FLOAT_EQ(last_twist_cmd_.twist.linear.x, 1.0);
    ASSERT_FLOAT_EQ(last_twist_cmd_.twist.angular.z, 1.0);

    // Hit lateral accel limit
    twist_raw.twist.linear.x = 5.0;
    twist_raw.twist.angular.z = 2.0;
    twist_raw_publisher_.publish(twist_raw);
    spinWhile(0.3);

    // Assert received steering angle is less than original value
    ASSERT_FLOAT_EQ(last_twist_cmd_.twist.linear.x, 5.0);
    ASSERT_LT(last_twist_cmd_.twist.angular.z, 2.0);

    // Hit lateral jerk limit
    twist_raw.twist.linear.x = 5.0;
    twist_raw.twist.angular.z = 10.0;
    twist_raw_publisher_.publish(twist_raw);
    spinWhile(0.3);

    // Assert received steering angle is less than original value
    ASSERT_FLOAT_EQ(last_twist_cmd_.twist.linear.x, 5.0);
    ASSERT_LT(last_twist_cmd_.twist.angular.z, 10.0);
  }

  void testCtrlFiltering()
  {
    twist_filter_node::TwistFilterNode node;

    autoware_msgs::ControlCommandStamped ctrl_msg;

    ctrl_msg.cmd.linear_velocity = 1.0;
    ctrl_msg.cmd.steering_angle = 1.0;
    ctrl_raw_publisher_.publish(ctrl_msg);
    spinWhile(0.3);

    // Assert received value is not changing
    ASSERT_EQ(last_ctrl_cmd_.cmd.linear_velocity, 1.0);
    ASSERT_FLOAT_EQ(last_ctrl_cmd_.cmd.steering_angle, 1.0);

    // Hit lateral accel limit
    ctrl_msg.cmd.linear_velocity = 5.0;
    ctrl_msg.cmd.steering_angle = 2.0;
    ctrl_raw_publisher_.publish(ctrl_msg);
    spinWhile(0.3);

    // Assert received steering angle is less than original value
    ASSERT_FLOAT_EQ(last_ctrl_cmd_.cmd.linear_velocity, 5.0);
    ASSERT_LT(last_ctrl_cmd_.cmd.steering_angle, 2.0);

    // Hit lateral jerk limit
    ctrl_msg.cmd.linear_velocity = 5.0;
    ctrl_msg.cmd.steering_angle = 10.0;
    ctrl_raw_publisher_.publish(ctrl_msg);
    spinWhile(0.3);

    // Assert received steering angle is less than original value
    ASSERT_FLOAT_EQ(last_ctrl_cmd_.cmd.linear_velocity, 5.0);
    ASSERT_LT(last_ctrl_cmd_.cmd.steering_angle, 10.0);
  }

  void spinWhile(double time_sec)
  {
    for (int i = 0; i < std::round(time_sec / SPIN_DURATION); ++i)
    {
      ros::spinOnce();
      ros::Duration(SPIN_DURATION).sleep();
    }
  }

private:
  const double SPIN_DURATION = 0.05;

  ros::NodeHandle nh_, private_nh_;

  ros::Publisher twist_raw_publisher_;
  ros::Publisher ctrl_raw_publisher_;

  ros::Subscriber twist_cmd_subscriber_;
  ros::Subscriber ctrl_cmd_subscriber_;

  geometry_msgs::TwistStamped last_twist_cmd_;
  autoware_msgs::ControlCommandStamped last_ctrl_cmd_;

  void callbackTwistCmd(const geometry_msgs::TwistStamped::ConstPtr& msg)
  {
    last_twist_cmd_ = *msg;
  }

  void callbackCtrlCmd(const autoware_msgs::ControlCommandStamped::ConstPtr& msg)
  {
    last_ctrl_cmd_ = *msg;
  }
};

TEST_F(TestSuite, TestTwistFilter)
{
  setupParameters();
  testTwistFiltering();
  testCtrlFiltering();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestNode_TwistFilter");
  return RUN_ALL_TESTS();
}

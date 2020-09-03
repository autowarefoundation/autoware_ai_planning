/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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

#include <gtest/gtest.h>
#include <ros/ros.h>

#include "test_twist_gate.hpp"

class TwistGateTestSuite : public ::testing::Test
{
public:
  TwistGateTestSuite() {}
  ~TwistGateTestSuite() {}

  TwistGateTestClass test_obj_;

protected:
  virtual void SetUp()
  {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    test_obj_.tg = new TwistGate(nh, private_nh);
  };
  virtual void TearDown() { delete test_obj_.tg; }
};

TEST_F(TwistGateTestSuite, twistCmdCallback)
{
  double linear_x = 5.0;
  double angular_z = 1.5;
  test_obj_.publishTwistCmd(linear_x, angular_z);
  ros::WallDuration(0.1).sleep();
  test_obj_.tgSpinOnce();
  for (int i = 0; i < 3; i++)
  {
    ros::WallDuration(0.1).sleep();
    ros::spinOnce();
  }

  ASSERT_EQ(linear_x, test_obj_.cb_vehicle_cmd.twist_cmd.twist.linear.x);
  ASSERT_EQ(angular_z, test_obj_.cb_vehicle_cmd.twist_cmd.twist.angular.z);
}

TEST_F(TwistGateTestSuite, controlCmdCallback)
{
  double linear_vel = 5.0;
  double linear_acc = 1.5;
  double steer_angle = 1.57;
  test_obj_.publishControlCmd(linear_vel, linear_acc, steer_angle);
  ros::WallDuration(0.1).sleep();
  test_obj_.tgSpinOnce();
  for (int i = 0; i < 3; i++)
  {
    ros::WallDuration(0.1).sleep();
    ros::spinOnce();
  }


  ASSERT_EQ(linear_vel, test_obj_.cb_vehicle_cmd.ctrl_cmd.linear_velocity);
  ASSERT_EQ(linear_acc, test_obj_.cb_vehicle_cmd.ctrl_cmd.linear_acceleration);
  ASSERT_EQ(steer_angle, test_obj_.cb_vehicle_cmd.ctrl_cmd.steering_angle);
}

TEST_F(TwistGateTestSuite, lampCmdCallback)
{
  int lamp_l = 1;
  int lamp_r = 1;

  double linear_vel = 5.0;
  double linear_acc = 1.5;
  double steer_angle = 1.57;

  test_obj_.publishLampCmd(lamp_l, lamp_r);
  ros::WallDuration(0.1).sleep();
  test_obj_.tgSpinOnce();

  // Lamp commands are only updated when new control command is published
  test_obj_.publishControlCmd(linear_vel, linear_acc, steer_angle);
  ros::WallDuration(0.1).sleep();
  test_obj_.tgSpinOnce();
  for (int i = 0; i < 3; i++)
  {
    ros::WallDuration(0.1).sleep();
    ros::spinOnce();
  }

  ASSERT_EQ(lamp_l, test_obj_.cb_vehicle_cmd.lamp_cmd.l);
  ASSERT_EQ(lamp_r, test_obj_.cb_vehicle_cmd.lamp_cmd.r);
}

TEST_F(TwistGateTestSuite, emergencyVehicleCmdCallback)
{
  double linear_vel = 5.0;
  double linear_acc = 1.5;
  double steer_angle = 1.57;
  int emergency = 1;
  double emergency_vel = 0.5;

  // Trigger emergency
  test_obj_.publishControlCmd(linear_vel, linear_acc, steer_angle);
  test_obj_.publishEmergencyVehicleCmd(emergency, emergency_vel);
  ros::WallDuration(0.1).sleep();
  test_obj_.tgSpinOnce();

  for (int i = 0; i < 3; i++)
  {
    ros::WallDuration(0.1).sleep();
    ros::spinOnce();
  }

  // Make sure emergency mode was triggered
  ASSERT_EQ(emergency, test_obj_.cb_vehicle_cmd.emergency);

  test_obj_.publishControlCmd(linear_vel, linear_acc, steer_angle);
  ros::WallDuration(0.1).sleep();
  test_obj_.tgSpinOnce();

  for (int i = 0; i < 3; i++)
  {
    ros::WallDuration(0.1).sleep();
    ros::spinOnce();
  }

  // Make sure emergency mode still active and emergency cmds being used.
  ASSERT_EQ(emergency, test_obj_.cb_vehicle_cmd.emergency);
  ASSERT_EQ(emergency_vel, test_obj_.cb_vehicle_cmd.ctrl_cmd.linear_velocity);

  // Clear emergency mode
  std::this_thread::sleep_for(std::chrono::milliseconds(10000));

  test_obj_.publishControlCmd(linear_vel, linear_acc, steer_angle);
  ros::WallDuration(0.1).sleep();
  test_obj_.tgSpinOnce();

  for (int i = 0; i < 3; i++)
  {
    ros::WallDuration(0.1).sleep();
    ros::spinOnce();
  }

  // Make sure emergency mode was cleared
  ASSERT_EQ(linear_vel, test_obj_.cb_vehicle_cmd.ctrl_cmd.linear_velocity);
  ASSERT_EQ(linear_acc, test_obj_.cb_vehicle_cmd.ctrl_cmd.linear_acceleration);
  ASSERT_EQ(steer_angle, test_obj_.cb_vehicle_cmd.ctrl_cmd.steering_angle);
}

TEST_F(TwistGateTestSuite, stateCallback)
{
  test_obj_.publishDecisionMakerState("VehicleReady\nWaitOrder\nWaitEngage\n");
  test_obj_.tgSpinOnce();
  ros::WallDuration(0.1).sleep();
  ros::spinOnce();

  autoware_msgs::VehicleCmd tg_msg = test_obj_.getTwistGateMsg();
  ASSERT_EQ(autoware_msgs::Gear::PARK, tg_msg.gear_cmd.gear);
  ASSERT_EQ(false, test_obj_.getIsStateDriveFlag());

  test_obj_.publishDecisionMakerState("VehicleReady\nDriving\nDrive\n");
  test_obj_.tgSpinOnce();
  ros::WallDuration(0.1).sleep();
  ros::spinOnce();

  tg_msg = test_obj_.getTwistGateMsg();
  ASSERT_EQ(autoware_msgs::Gear::DRIVE, tg_msg.gear_cmd.gear);
  ASSERT_EQ(true, test_obj_.getIsStateDriveFlag());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TwistGateTestNode");
  return RUN_ALL_TESTS();
}

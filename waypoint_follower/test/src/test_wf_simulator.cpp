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

#include <ros/ros.h>
#include <cmath>
#include <gtest/gtest.h>
#include <iostream>
#include <amathutils_lib/amathutils.hpp>

#include "wf_simulator/wf_simulator_core.hpp"

class TestSuite : public ::testing::Test
{
public:
  TestSuite() : nh_(""), pnh_("~")
  {
    pub_vehicle_cmd_ = nh_.advertise<autoware_msgs::VehicleCmd>("/vehicle_cmd", 1);
    pub_lane_ = nh_.advertise<autoware_msgs::Lane>("/base_waypoints", 1);
    pub_closest_ = nh_.advertise<std_msgs::Int32>("/closest_waypoint", 1);
    sub_pose_ = nh_.subscribe("/current_pose", 1, &TestSuite::callbackPose, this);
    sub_twist_ = nh_.subscribe("/current_velocity", 1, &TestSuite::callbackTwist, this);
    spin_duration_ = 0.05;
    spin_loopnum_ = 10;
  }
  ~TestSuite()
  {
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_pose_, sub_twist_;
  ros::Publisher pub_vehicle_cmd_, pub_lane_, pub_closest_;
  geometry_msgs::TwistStamped curr_twist_;
  geometry_msgs::PoseStamped curr_pose_;
  double spin_duration_;
  int spin_loopnum_;

  void callbackPose(const geometry_msgs::PoseStamped& pose)
  {
    curr_pose_ = pose;
  }
  void callbackTwist(const geometry_msgs::TwistStamped& twist)
  {
    curr_twist_ = twist;
  }

  void resetPose()
  {
    geometry_msgs::PoseStamped pose;
    curr_pose_ = pose;
  }
  void resetTwist()
  {
    geometry_msgs::TwistStamped twist;
    curr_twist_ = twist;
  }

  void publishWaypoints()
  {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double wp_dt = 1.0;
    autoware_msgs::Lane lane;
    autoware_msgs::Waypoint wp;
    for (int i = 0; i < 50; ++i)
    {
      wp.pose.pose.position.x = x;
      wp.pose.pose.position.y = y;
      wp.pose.pose.orientation = amathutils::getQuaternionFromYaw(yaw);
      wp.twist.twist.linear.x = 1.0;
      wp.twist.twist.angular.z = 0.0;
      lane.waypoints.push_back(wp);
      x += 1.0 * std::cos(yaw) * wp_dt;
      y += 1.0 * std::sin(yaw) * wp_dt;
      yaw += 0.0 * wp_dt;
    }
    pub_lane_.publish(lane);

    std_msgs::Int32 closest;
    closest.data = 1;
    pub_closest_.publish(closest);

    ros::spinOnce();
    ros::Duration(spin_duration_).sleep();
    ros::spinOnce();
    ros::Duration(spin_duration_).sleep();
  }

  void publishVehicleCmd(double vx, double wz, double steer, double duration)
  {
    for (int i = 0; i < std::round(duration / spin_duration_); ++i)
    {
      ros::Time current_time = ros::Time::now();
      autoware_msgs::VehicleCmd cmd;
      cmd.twist_cmd.header.stamp = current_time;
      cmd.twist_cmd.twist.linear.x = vx;
      cmd.twist_cmd.twist.angular.z = wz;
      cmd.ctrl_cmd.linear_velocity = vx;
      cmd.ctrl_cmd.steering_angle = steer;
      pub_vehicle_cmd_.publish(cmd);
      ros::spinOnce();
      ros::Duration(spin_duration_).sleep();
    }
  }

  void testGoForwardStraight()
  {
    resetPose();
    resetTwist();
    WFSimulator wf_simulator;

    publishWaypoints();

    double duration = 3.0;
    double vx = 3.0;
    double wz = 0.0;
    double steer = 0.0;
    publishVehicleCmd(vx, wz, steer, duration);

    ASSERT_GT(curr_twist_.twist.linear.x, 0.5) << "going forward";
    ASSERT_GT(curr_pose_.pose.position.x, 1.0) << "going forward";
  }

  void testGoBackStraight()
  {
    resetPose();
    resetTwist();
    WFSimulator wf_simulator;

    publishWaypoints();

    double duration = 3.0;
    double vx = -3.0;
    double wz = 0.0;
    double steer = 0.0;
    publishVehicleCmd(vx, wz, steer, duration);

    ASSERT_LT(curr_twist_.twist.linear.x, -0.5) << "going back";
    ASSERT_LT(curr_pose_.pose.position.x, -1.0) << "going back";
  }

  void testGoRightForward()
  {
    resetPose();
    resetTwist();
    WFSimulator wf_simulator;

    publishWaypoints();

    double duration = 3.0;
    double vx = 3.0;
    double wz = 0.1;
    double steer = 0.1;
    publishVehicleCmd(vx, wz, steer, duration);

    ASSERT_GT(curr_twist_.twist.linear.x, 0.1) << "going forward";
    ASSERT_GT(curr_pose_.pose.position.y, 0.1) << "going right";
  }

  void testGoLeftForward()
  {
    resetPose();
    resetTwist();
    WFSimulator wf_simulator;

    publishWaypoints();

    double duration = 3.0;
    double vx = 3.0;
    double wz = -0.1;
    double steer = -0.1;
    publishVehicleCmd(vx, wz, steer, duration);

    ASSERT_GT(curr_twist_.twist.linear.x, 0.1) << "going forward";
    ASSERT_LT(curr_pose_.pose.position.y, -0.1) << "going left";
  }

  void spinWhile(double time_sec)
  {
    for (int i = 0; i < std::round(time_sec / spin_duration_); ++i)
    {
      ros::spinOnce();
      ros::Duration(spin_duration_).sleep();
    }
  }
};

TEST_F(TestSuite, TestWfSimulator)
{
  /* == TestGoStraight == */
  {
    pnh_.setParam("initialize_source", "ORIGIN");
    std::string vehicle_mode_type_array[] = { "IDEAL_TWIST", "IDEAL_STEER", "DELAY_TWIST", "DELAY_STEER",
                                              "CONST_ACCEL_TWIST" };
    for (const auto vehicle_model_type : vehicle_mode_type_array)
    {
      // ROS_ERROR("%s, %s", vehicle_model_type.c_str(), qp_solver_type.c_str());
      pnh_.setParam("vehicle_model_type", vehicle_model_type);
      testGoForwardStraight();
      testGoBackStraight();
      testGoRightForward();
      testGoLeftForward();
    }
  }

  /* == TestInitialPose PoseStamped == */
  {
    pnh_.setParam("initialize_source", "RVIZ");
    pnh_.setParam("vehicle_model_type", "IDEAL_TWIST");
    pnh_.setParam("add_measurement_noise", false);

    resetPose();
    resetTwist();
    WFSimulator wf_simulator;

    ros::Publisher pub_initialpose = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    geometry_msgs::PoseWithCovarianceStamped p;
    p.header.frame_id = "map";
    p.header.stamp = ros::Time::now();
    p.pose.pose.position.x = 3.5;
    p.pose.pose.orientation.w = 1.0;
    pub_initialpose.publish(p);
    spinWhile(1.0);
    ASSERT_LT(std::fabs(curr_pose_.pose.position.x - p.pose.pose.position.x), 0.1) << "initial pose x should be same";
  }

  /* == TestInitialPose PoseStampedWthCovariance == */
  {
    pnh_.setParam("initialize_source", "NDT");
    pnh_.setParam("vehicle_model_type", "IDEAL_TWIST");
    pnh_.setParam("add_measurement_noise", false);

    resetPose();
    resetTwist();
    WFSimulator wf_simulator;

    ros::Publisher pub_initialpose = nh_.advertise<geometry_msgs::PoseStamped>("/ndt_pose", 1);

    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    p.header.stamp = ros::Time::now();
    p.pose.position.x = 3.5;
    p.pose.orientation.w = 1.0;
    pub_initialpose.publish(p);
    spinWhile(1.0);
    ASSERT_LT(std::fabs(curr_pose_.pose.position.x - p.pose.position.x), 0.1) << "initial pose x should be same";
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestNode_WfSim");
  return RUN_ALL_TESTS();
}

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
#include <gtest/gtest.h>
#include <pure_pursuit/pure_pursuit_core.h>

namespace waypoint_follower
{
class PurePursuitNodeTestSuite : public ::testing::Test
{
protected:
  std::unique_ptr<PurePursuitNode> obj_;
  virtual void SetUp()
  {
    obj_ = std::unique_ptr<PurePursuitNode>(new PurePursuitNode());
    obj_->add_virtual_end_waypoints_ = true;
  }
  virtual void TearDown()
  {
    obj_.reset();
  }

public:
  PurePursuitNodeTestSuite() {}
  ~PurePursuitNodeTestSuite() {}
  LaneDirection getDirection()
  {
    return obj_->direction_;
  }
  void ppCallbackFromWayPoints(const autoware_msgs::LaneConstPtr& msg)
  {
    obj_->callbackFromWayPoints(msg);
  }
  void ppConnectVirtualLastWaypoints(
    autoware_msgs::Lane* expand_lane, LaneDirection direction)
  {
    obj_->connectVirtualLastWaypoints(expand_lane, direction);
  }
};

TEST_F(PurePursuitNodeTestSuite, inputPositivePath)
{
  autoware_msgs::Lane original_lane;
  original_lane.waypoints.resize(3, autoware_msgs::Waypoint());
  for (int i = 0; i < 3; i++)
  {
    original_lane.waypoints[i].pose.pose.position.x = i;
    original_lane.waypoints[i].pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(0.0);
  }
  const autoware_msgs::LaneConstPtr
    lp(boost::make_shared<autoware_msgs::Lane>(original_lane));
  ppCallbackFromWayPoints(lp);
  ASSERT_EQ(getDirection(), LaneDirection::Forward)
    << "direction is not matching to positive lane.";
}

TEST_F(PurePursuitNodeTestSuite, inputNegativePath)
{
  autoware_msgs::Lane original_lane;
  original_lane.waypoints.resize(3, autoware_msgs::Waypoint());
  for (int i = 0; i < 3; i++)
  {
    original_lane.waypoints[i].pose.pose.position.x = -i;
    original_lane.waypoints[i].pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(0.0);
  }
  const autoware_msgs::LaneConstPtr
    lp(boost::make_shared<autoware_msgs::Lane>(original_lane));
  ppCallbackFromWayPoints(lp);
  ASSERT_EQ(getDirection(), LaneDirection::Backward)
    << "direction is not matching to negative lane.";
}
// If original lane is empty, new lane is also empty.
TEST_F(PurePursuitNodeTestSuite, inputEmptyLane)
{
  autoware_msgs::Lane original_lane, new_lane;
  ppConnectVirtualLastWaypoints(&new_lane, LaneDirection::Forward);
  ASSERT_EQ(original_lane.waypoints.size(), new_lane.waypoints.size())
    << "Input empty lane, and output is not empty";
}

// If the original lane exceeds 2 points,
// the additional part will be updated at
// the interval of the first 2 points.
TEST_F(PurePursuitNodeTestSuite, inputNormalLane)
{
  autoware_msgs::Lane original_lane;
  original_lane.waypoints.resize(2, autoware_msgs::Waypoint());
  for (int i = 0; i < 2; i++)
  {
    original_lane.waypoints[i].pose.pose.position.x = i;
  }
  autoware_msgs::Lane new_lane(original_lane);
  ppConnectVirtualLastWaypoints(&new_lane, LaneDirection::Forward);

  ASSERT_LT(original_lane.waypoints.size(), new_lane.waypoints.size())
    << "Fail to expand waypoints";
}
}  // namespace waypoint_follower

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "PurePursuitTest");
  return RUN_ALL_TESTS();
}

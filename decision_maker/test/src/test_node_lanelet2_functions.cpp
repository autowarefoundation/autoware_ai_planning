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

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <lanelet2_extension/utility/message_conversion.h>
#include <amathutils_lib/amathutils.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <string>

#include "decision_maker_node.hpp"
#include "test_class.hpp"

using lanelet::Lanelet;
using lanelet::LineString3d;
using lanelet::LineStringOrPolygon3d;
using lanelet::Point3d;
using lanelet::Points3d;
using lanelet::utils::getId;

namespace decision_maker
{
class TestSuiteLanelet : public ::testing::Test
{
public:
  TestSuiteLanelet()
  {
  }
  ~TestSuiteLanelet()
  {
  }

  TestClass test_obj_;

  lanelet::LaneletMapPtr sample_map_ptr_;
  ros::Publisher map_bin_pub_;

  void createLaneletMap()
  {
    sample_map_ptr_ = std::make_shared<lanelet::LaneletMap>();

    // create sample lanelets
    Point3d p1, p2, p3, p4, p5, p6, p7, p8;

    p1 = Point3d(getId(), 0., -1., 0.);
    p2 = Point3d(getId(), 0., 1., 0.);
    LineString3d ls_left(getId(), { p1, p2 });  // NOLINT

    p3 = Point3d(getId(), 100., -1., 0.);
    p4 = Point3d(getId(), 100., 1., 0.);
    LineString3d ls_right(getId(), { p3, p4 });  // NOLINT

    p5 = Point3d(getId(), 19.5, -1., 0.);
    p6 = Point3d(getId(), 19.5, 1., 0.);
    LineString3d stop_line(getId(), { p5, p6 });  // NOLINT
    stop_line.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::StopLine;

    p7 = Point3d(getId(), 19.5, 1.0, 2.);
    p8 = Point3d(getId(), 19.5, 1.3, 2.);
    LineString3d stop_sign(getId(), { p7, p8 });  // NOLINT
    stop_sign.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::TrafficSign;
    stop_sign.attributes()[lanelet::AttributeName::Subtype] = "stop_sign";

    lanelet::TrafficSignsWithType tswt;
    tswt.trafficSigns = { stop_sign };
    tswt.type = "stop_sign";
    auto ts = lanelet::TrafficSign::make(getId(), lanelet::AttributeMap(), tswt, {}, { stop_line });  // NOLINT

    lanelet::Lanelet road_lanelet = Lanelet(getId(), ls_left, ls_right);
    road_lanelet.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
    road_lanelet.addRegulatoryElement(ts);

    sample_map_ptr_->add(road_lanelet);
  }

  void createLaneArray(autoware_msgs::LaneArray* lane_array_ptr)
  {
    lane_array_ptr->lanes.clear();
    autoware_msgs::Lane lane;
    for (int idx = 0; idx < 100; idx++)
    {
      static autoware_msgs::Waypoint wp;
      wp.gid = idx;
      wp.lid = idx;
      wp.pose.pose.position.x = 0.0 + static_cast<double>(idx);
      wp.pose.pose.position.y = 0.0;
      wp.pose.pose.position.z = 0.0;
      wp.twist.twist.linear.x = 5.0;
      wp.twist.twist.angular.z = 0.0;

      wp.wpstate.steering_state = autoware_msgs::WaypointState::STR_STRAIGHT;
      wp.wpstate.stop_state = autoware_msgs::WaypointState::NULLSTATE;
      wp.wpstate.event_state = autoware_msgs::WaypointState::TYPE_EVENT_NULL;

      lane.waypoints.push_back(wp);
    }
    lane_array_ptr->lanes.push_back(lane);
  }

  void publishLaneletMap()
  {
    autoware_lanelet2_msgs::MapBin map_bin_msg;
    lanelet::utils::conversion::toBinMsg(sample_map_ptr_, &map_bin_msg);
    map_bin_pub_.publish(map_bin_msg);
  }

  int getSteeringStateFromWaypoint(const autoware_msgs::LaneArray& lane_array)
  {
    for (const auto& lane : lane_array.lanes)
    {
      for (const auto& wp : lane.waypoints)
      {
        if (wp.wpstate.steering_state != autoware_msgs::WaypointState::STR_STRAIGHT)
        {
          return wp.wpstate.steering_state;
        }
      }
    }
    return autoware_msgs::WaypointState::STR_STRAIGHT;
  }

protected:
  virtual void SetUp()
  {
    int argc;
    char** argv;
    test_obj_.dmn = new DecisionMakerNode(argc, argv);

    createLaneletMap();

    ros::NodeHandle rosnode;
    map_bin_pub_ = rosnode.advertise<autoware_lanelet2_msgs::MapBin>("lanelet_map_bin", 1, true);
  };
  virtual void TearDown()
  {
    delete test_obj_.dmn;
  };
};

TEST_F(TestSuiteLanelet, initLaneletMap)
{
  publishLaneletMap();
  test_obj_.initLaneletMap();

  ASSERT_TRUE(test_obj_.isEventFlagTrue("lanelet2_map_loaded")) << "Failed to load lanelet map";
}

TEST_F(TestSuiteLanelet, setWaypointStateUsingLanelet2Map)
{
  autoware_msgs::LaneArray lane_array;
  createLaneArray(&lane_array);
  test_obj_.setLaneletMap(sample_map_ptr_);

  // check if stop state is set properly.
  test_obj_.setWaypointStateUsingLanelet2Map(&lane_array);
  ASSERT_EQ(autoware_msgs::WaypointState::TYPE_STOPLINE, lane_array.lanes.front().waypoints.at(20).wpstate.stop_state);

  // check if turn state is set properly
  test_obj_.setTurnDirectionToMap("straight");
  test_obj_.setWaypointStateUsingLanelet2Map(&lane_array);
  ASSERT_EQ(autoware_msgs::WaypointState::STR_STRAIGHT, getSteeringStateFromWaypoint(lane_array))
      << "Could not get the expected state"
      << "It should be " << autoware_msgs::WaypointState::STR_STRAIGHT;
  test_obj_.setTurnDirectionToMap("left");
  test_obj_.setWaypointStateUsingLanelet2Map(&lane_array);
  ASSERT_EQ(autoware_msgs::WaypointState::STR_LEFT, getSteeringStateFromWaypoint(lane_array))
      << "Could not get the expected state"
      << "It should be " << autoware_msgs::WaypointState::STR_LEFT;
  test_obj_.setTurnDirectionToMap("right");
  test_obj_.setWaypointStateUsingLanelet2Map(&lane_array);
  ASSERT_EQ(autoware_msgs::WaypointState::STR_RIGHT, getSteeringStateFromWaypoint(lane_array))
      << "Could not get the expected state"
      << "It should be " << autoware_msgs::WaypointState::STR_RIGHT;
}
}  // namespace decision_maker

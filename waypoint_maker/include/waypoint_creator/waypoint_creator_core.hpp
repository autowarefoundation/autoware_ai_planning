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

/**
 * @file waypoint_creator_core.hpp
 * @brief create smooth waypoints with interpolation from source poses
 * @author Takamasa Horibe
 * @date 2019.08.28
 */

#ifndef WAYPOINT_MARKER_WAYPOINT_CREATOR_CORE_H_
#define WAYPOINT_MARKER_WAYPOINT_CREATOR_CORE_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <autoware_msgs/LaneArray.h>

class WaypointCreator
{
public:
  /**
   * @brief constructor
   */
  WaypointCreator();

private:
  /* ros system */
  ros::NodeHandle nh_;                       //!< @brief ros node handle
  ros::NodeHandle pnh_;                      //!< @brief private ros node handle
  tf2_ros::Buffer tf2_buffer_;               //!< @brief tf2 buffer
  tf2_ros::TransformListener tf2_listener_;  //!< @brief tf2 listener
  ros::Subscriber sub_source_point_;         //!< @brief subscriber for source poses
  ros::Subscriber sub_delete_point_;         //!< @brief subscriber for delete poses
  ros::Publisher pub_waypoints_;             //!< @brief publihser for generated waypoints
  ros::Publisher pub_visualize_;             //!< @brief publisher for waypoints visualization

  /* pose vector */
  std::vector<geometry_msgs::Pose> source_pose_v_;        //!< @brief source poses vector
  std::vector<geometry_msgs::Pose> interpolated_pose_v_;  //!< @brief interpolated poses vector

  /* parameters */
  double waypoint_velocity_;       //!< @brief waypoint velocity
  double remove_pose_thr_;         //!< @brief when received delete pose, remove a closest source point which is in this
                                   //!< threshold range
  double interpolation_interval_;  //!< @brief length between waypoints used for interpolation
  std::string interpolation_method_;  //!< @brief interpolation method (option: linear or spline)
  std::string waypoint_frame_id_;     //!< @brief output waypoints frame_id

  /**
   * @brief callback function for source poses
   * @param [in] in_point received point stocked in source_pose_v_
   */
  void inputPointCallback(const geometry_msgs::PointStamped& in_point);

  /**
   * @brief callback function for delete poses
   * @param [in] in_pose received pose to remove source pose in source_pose_v_
   */
  void deletePoseCallback(const geometry_msgs::PoseStamped& in_pose);

  /**
   * @brief interpolate poses with constant interval length
   * @param [in] in_poses source pose vector for interpolation
   * @param [in] interpolation_interval interpolated output pose vector distance
   * @param [in] method interpolation method (option: "linear" or "spline")
   * @param [out] out_poses interpolated pose vector
   */
  bool interpolatePosesWithConstantDistance(const std::vector<geometry_msgs::Pose>& in_poses,
                                            const double& interpolation_interval, const std::string& method,
                                            std::vector<geometry_msgs::Pose>& out_poses) const;

  /**
   * @brief interpolate poses
   * @param [in] in_poses source pose vector for interpolation
   * @param [in] interpolation_interval sampling span for interpolation
   * @param [in] method interpolation method (option: "linear" or "spline")
   * @param [out] out_poses interpolated pose vector
   */
  bool interpolatePoses(const std::vector<geometry_msgs::Pose>& in_poses, const double& interpolation_interval,
                        const std::string& method, std::vector<geometry_msgs::Pose>& out_poses) const;

  /**
   * @brief publish generated waypoints
   */
  void publishWaypoints() const;

  /**
   * @brief publish visualization marker for waypoints
   */
  void visualizeWaypoints() const;

  /**
   * @brief create pose marker for visualization
   * @param [in] pose source pose vector for interpolation
   * @param [in] color output marker
   * @param [in] scale output marker scake
   * @param [in] ns output marker namespace
   * @param [in] id output marker id
   * @return created marker
   */
  visualization_msgs::Marker createPoseMarker(const geometry_msgs::Pose& pose, const std_msgs::ColorRGBA& color,
                                              const geometry_msgs::Vector3& scale, const std::string& ns,
                                              const int32_t id) const;

  /**
   * @brief publish visualize waypoints with delete action
   */
  void deleteVisualizeWaypoints();
};

#endif
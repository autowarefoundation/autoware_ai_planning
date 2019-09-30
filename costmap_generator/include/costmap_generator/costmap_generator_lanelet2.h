/*
 *  Copyright (c) 2018, Nagoya University
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
 ********************/

#ifndef COSTMAP_GENERATOR_LANELET2_H
#define COSTMAP_GENERATOR_LANELET2_H

// headers in ROS
#include <ros/ros.h>

#include <grid_map_msgs/GridMap.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_listener.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

// headers in Autoware
#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <costmap_generator/objects_to_costmap.h>
#include <costmap_generator/points_to_costmap.h>
#include <lanelet2_extension/utility/message_conversion.h>

// headers in STL
#include <memory>

class CostmapGeneratorLanelet2
{
public:
  CostmapGeneratorLanelet2();

  void init();
  void run();

private:
  friend class TestClass;
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  bool use_objects_box_;
  bool use_objects_convex_hull_;
  bool use_points_;
  bool use_wayarea_;

  bool has_subscribed_wayarea_;

  bool loaded_lanelet_map_ = false;
  lanelet::LaneletMapPtr lanelet_map_;
  bool use_all_road_lanelets_ = true;

  std::string lidar_frame_;
  std::string map_frame_;
  double grid_min_value_;
  double grid_max_value_;
  double grid_resolution_;
  double grid_length_x_;
  double grid_length_y_;
  double grid_position_x_;
  double grid_position_y_;

  double maximum_lidar_height_thres_;
  double minimum_lidar_height_thres_;

  double expand_polygon_size_;
  int size_of_expansion_kernel_;

  grid_map::GridMap costmap_;

  ros::Subscriber sub_lanelet_bin_map_;

  ros::Publisher pub_costmap_;
  ros::Publisher pub_occupancy_grid_;
  ros::Subscriber sub_waypoint_;
  ros::Subscriber sub_points_;
  ros::Subscriber sub_objects_;

  tf::TransformListener tf_listener_;

  std::vector<std::vector<geometry_msgs::Point>> area_points_;

  PointsToCostmap points2costmap_;
  ObjectsToCostmap objects2costmap_;

  const std::string OBJECTS_BOX_COSTMAP_LAYER_;
  const std::string OBJECTS_CONVEX_HULL_COSTMAP_LAYER_;
  const std::string SENSOR_POINTS_COSTMAP_LAYER_;
  const std::string VECTORMAP_COSTMAP_LAYER_;
  const std::string LANELET2_COSTMAP_LAYER_;
  const std::string COMBINED_COSTMAP_LAYER_;

  /// \brief callback for loading landlet2 map
  void laneletBinMapCallback(const autoware_lanelet2_msgs::MapBin& msg);

  /// \brief wait for lanelet2 map to load and build routing graph
  void initLaneletMap();

  /// \brief callback for DetectedObjectArray
  /// \param[in] in_objects input DetectedObjectArray usually from prediction or perception component
  void objectsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr& in_ojects);

  /// \brief callback for sensor_msgs::PointCloud2
  /// \param[in] in_sensor_points input sensot_msgs::PointCloud2. Assuming groud-fitered pointcloud by default
  void sensorPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& in_sensor_points);

  /// \brief initialize gridmap parameters based on rosparam
  void initGridmap();

  /// \brief publish ros msg: /semantics/costmap (grid_map::GridMap),
  ///                         /semantics/costmap_generator/occupancy_grid(nav_msgs::OccupancyGrid)
  /// \param[in] gridmap with calculated cost
  /// \param[in] input ros header
  void publishRosMsg(const grid_map::GridMap& gridmap, const std_msgs::Header& in_header);

  /// \brief set area_points from lanelet polygons
  /// \param [in] input lanelet_map
  /// \param [out] calculated area_points of lanelet polygons
  void loadRoadAreasFromLaneletMap(const lanelet::LaneletMapPtr lanelet_map,
                                   std::vector<std::vector<geometry_msgs::Point>>* area_points);

  /// \brief calculate cost from pointcloud data
  /// \param[in] in_sensor_points: subscribed pointcloud data
  grid_map::Matrix generateSensorPointsCostmap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_sensor_points);

  /// \brief calculate cost from DetectedObjectArray
  /// \param[in] in_objects: subscribed DetectedObjectArray
  grid_map::Matrix generateObjectsCostmap(const autoware_msgs::DetectedObjectArray::ConstPtr& in_objects,
                                          const bool use_objects_convex_hull);

  /// \brief calculate cost from lanelet2 map
  grid_map::Matrix generateLanelet2Costmap();

  /// \brief calculate cost for final output
  grid_map::Matrix generateCombinedCostmap();
};

#endif  // COSTMAP_GENERATOR_H

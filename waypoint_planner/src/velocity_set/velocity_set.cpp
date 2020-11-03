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
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <iostream>
#include <tf2_ros/transform_listener.h>

#include <waypoint_planner/velocity_set/libvelocity_set.h>
#include <waypoint_planner/velocity_set/velocity_set_info.h>
#include <waypoint_planner/velocity_set/velocity_set_path.h>

namespace
{
constexpr int32_t DECELERATION_SEARCH_DISTANCE = 30;
// The number of waypoints ahead of the current closest waypoint to search.
constexpr int32_t STOP_SEARCH_DISTANCE = 60;

void obstacleColorByKind(const EControl kind, std_msgs::ColorRGBA &color, const double alpha=0.5)
{
  if (kind == EControl::STOP)
  {
    color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = alpha;  // red
  }
  else if (kind == EControl::STOPLINE)
  {
    color.r = 0.0; color.g = 0.0; color.b = 1.0; color.a = alpha;  // blue
  }
  else if (kind == EControl::DECELERATE)
  {
    color.r = 1.0; color.g = 1.0; color.b = 0.0; color.a = alpha;  // yellow
  }
  else
  {
    color.r = 1.0; color.g = 1.0; color.b = 1.0; color.a = alpha;  // white
  }
}

// Display a detected obstacle
void displayObstacle(const EControl& kind, const ObstaclePoints& obstacle_points, const ros::Publisher& obstacle_pub)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  static geometry_msgs::Point prev_obstacle_point;
  if (kind == EControl::STOP || kind == EControl::STOPLINE || kind == EControl::DECELERATE)
  {
    marker.pose.position = obstacle_points.getObstaclePoint(kind);
    prev_obstacle_point = marker.pose.position;
  }
  else  // kind == OTHERS
  {
    marker.pose.position = prev_obstacle_point;
  }
  geometry_msgs::Quaternion quat;
  marker.pose.orientation = quat;

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 2.0;
  marker.lifetime = ros::Duration(0.1);
  marker.frame_locked = true;
  obstacleColorByKind(kind, marker.color, 0.7);

  obstacle_pub.publish(marker);
}

void displayDetectionRange(const autoware_msgs::Lane& lane, const CrossWalk& crosswalk, const int closest_waypoint,
                           const EControl& kind, const int obstacle_waypoint, const double stop_range,
                           const double deceleration_range, const ros::Publisher& detection_range_pub)
{
  // set up for marker array
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker crosswalk_marker;
  visualization_msgs::Marker waypoint_marker_stop;
  visualization_msgs::Marker waypoint_marker_decelerate;
  visualization_msgs::Marker stop_line_marker;
  crosswalk_marker.header.frame_id = "/map";
  crosswalk_marker.header.stamp = ros::Time();
  crosswalk_marker.id = 0;
  crosswalk_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  crosswalk_marker.action = visualization_msgs::Marker::ADD;
  waypoint_marker_stop = crosswalk_marker;
  waypoint_marker_decelerate = crosswalk_marker;
  stop_line_marker = crosswalk_marker;
  stop_line_marker.type = visualization_msgs::Marker::CUBE;

  // set each namespace
  crosswalk_marker.ns = "Crosswalk Detection";
  waypoint_marker_stop.ns = "Stop Detection";
  waypoint_marker_decelerate.ns = "Decelerate Detection";
  stop_line_marker.ns = "Stop Line";

  // set scale and color
  double scale = 2 * stop_range;
  waypoint_marker_stop.scale.x = scale;
  waypoint_marker_stop.scale.y = scale;
  waypoint_marker_stop.scale.z = scale;
  waypoint_marker_stop.color.a = 0.2;
  waypoint_marker_stop.color.r = 0.0;
  waypoint_marker_stop.color.g = 1.0;
  waypoint_marker_stop.color.b = 0.0;
  waypoint_marker_stop.frame_locked = true;

  scale = 2 * (stop_range + deceleration_range);
  waypoint_marker_decelerate.scale.x = scale;
  waypoint_marker_decelerate.scale.y = scale;
  waypoint_marker_decelerate.scale.z = scale;
  waypoint_marker_decelerate.color.a = 0.15;
  waypoint_marker_decelerate.color.r = 1.0;
  waypoint_marker_decelerate.color.g = 1.0;
  waypoint_marker_decelerate.color.b = 0.0;
  waypoint_marker_decelerate.frame_locked = true;

  // stop_line marker
  if (kind != EControl::KEEP)
  {
    if (obstacle_waypoint > -1)
    {
      stop_line_marker.pose.position = lane.waypoints[obstacle_waypoint].pose.pose.position;
      stop_line_marker.pose.orientation = lane.waypoints[obstacle_waypoint].pose.pose.orientation;
    }
    stop_line_marker.pose.position.z += 1.0;
    stop_line_marker.scale.x = 0.1;
    stop_line_marker.scale.y = 15.0;
    stop_line_marker.scale.z = 2.0;
    stop_line_marker.lifetime = ros::Duration(0.1);
    stop_line_marker.frame_locked = true;
    obstacleColorByKind(kind, stop_line_marker.color, 0.3);
    marker_array.markers.push_back(stop_line_marker);
  }

  // crosswalk marker
  int crosswalk_id = crosswalk.getDetectionCrossWalkID();
  if (crosswalk_id > 0)
  {
    scale = crosswalk.getDetectionPoints(crosswalk_id).width;
    crosswalk_marker.scale.x = scale;
    crosswalk_marker.scale.y = scale;
    crosswalk_marker.scale.z = scale;
    crosswalk_marker.color.a = 0.5;
    crosswalk_marker.color.r = 0.0;
    crosswalk_marker.color.g = 1.0;
    crosswalk_marker.color.b = 0.0;
    crosswalk_marker.frame_locked = true;

    if (!crosswalk.isMultipleDetection())
    {
      for (const auto& p : crosswalk.getDetectionPoints(crosswalk_id).points)
        crosswalk_marker.points.push_back(p);
    }
    else
    {
      for (const auto& c_id : crosswalk.getDetectionCrossWalkIDs())
      {
        for (const auto& p : crosswalk.getDetectionPoints(c_id).points)
        {
          scale = crosswalk.getDetectionPoints(c_id).width;
          crosswalk_marker.points.push_back(p);
        }
      }
    }
    marker_array.markers.push_back(crosswalk_marker);
  }

  // stop and deceleration markers
  for (int i = 0; i < STOP_SEARCH_DISTANCE; i++)
  {
    if (closest_waypoint < 0 || i + closest_waypoint > static_cast<int>(lane.waypoints.size()) - 1)
      break;

    geometry_msgs::Point point;
    point = lane.waypoints[closest_waypoint + i].pose.pose.position;

    waypoint_marker_stop.points.push_back(point);

    if (i > DECELERATION_SEARCH_DISTANCE)
      continue;
    waypoint_marker_decelerate.points.push_back(point);
  }
  if (!waypoint_marker_stop.points.empty())
  {
    marker_array.markers.push_back(waypoint_marker_stop);
  }
  if (!waypoint_marker_decelerate.points.empty())
  {
    marker_array.markers.push_back(waypoint_marker_decelerate);
  }

  detection_range_pub.publish(marker_array);
  marker_array.markers.clear();
}

// obstacle detection for crosswalk
EControl crossWalkDetection(const pcl::PointCloud<pcl::PointXYZ>& pcl_points, const CrossWalk& crosswalk,
                            const geometry_msgs::Pose localizer_pose, const int points_threshold,
                            ObstaclePoints* obstacle_points)
{
  int crosswalk_id = crosswalk.getDetectionCrossWalkID();
  double search_radius = crosswalk.getDetectionPoints(crosswalk_id).width / 2;

  // Search each calculated points in the crosswalk
  for (const auto& c_id : crosswalk.getDetectionCrossWalkIDs())
  {
    for (const auto& p : crosswalk.getDetectionPoints(c_id).points)
    {
      geometry_msgs::Point detection_point = calcRelativeCoordinate(p, localizer_pose);
      tf::Vector3 detection_vector = point2vector(detection_point);
      detection_vector.setZ(0.0);

      int stop_count = 0;  // the number of points in the detection area
      for (const auto& p : pcl_points)
      {
        tf::Vector3 point_vector(p.x, p.y, 0.0);
        double distance = tf::tfDistance(point_vector, detection_vector);
        if (distance < search_radius)
        {
          stop_count++;
          geometry_msgs::Point point_temp;
          point_temp.x = p.x;
          point_temp.y = p.y;
          point_temp.z = p.z;
          obstacle_points->setStopPoint(calcAbsoluteCoordinate(point_temp, localizer_pose));
        }
        if (stop_count > points_threshold)
          return EControl::STOP;
      }
    }

    obstacle_points->clearStopPoints();
    if (!crosswalk.isMultipleDetection())
      break;
  }
  return EControl::KEEP;  // find no obstacles
}

int detectStopObstacle(const pcl::PointCloud<pcl::PointXYZ>& pcl_points, const int closest_waypoint,
                       const autoware_msgs::Lane& lane, const CrossWalk& crosswalk, double stop_range,
                       double points_threshold, const geometry_msgs::Pose localizer_pose,
                       ObstaclePoints* obstacle_points, EObstacleType* obstacle_type,
                       const int wpidx_detection_result_by_other_nodes)
{
  int stop_obstacle_waypoint = -1;
  *obstacle_type = EObstacleType::NONE;
  // start search from the closest waypoint
  for (int i = closest_waypoint; i < closest_waypoint + STOP_SEARCH_DISTANCE && i < static_cast<int>(lane.waypoints.size()); i++)
  {

    // detection another nodes
    if (wpidx_detection_result_by_other_nodes >= 0 &&
        lane.waypoints.at(i).gid == wpidx_detection_result_by_other_nodes)
    {
      stop_obstacle_waypoint = i;
      *obstacle_type = EObstacleType::STOPLINE;
      obstacle_points->setStopPoint(lane.waypoints.at(i).pose.pose.position); // for vizuialization
      break;
    }

    // Detection for cross walk
    // If enable_crosswalk_detection is false, crosswalk.getDetectionWaypoint() returns -1.
    if (i == crosswalk.getDetectionWaypoint())
    {
      // found an obstacle in the cross walk
      if (crossWalkDetection(pcl_points, crosswalk, localizer_pose, points_threshold, obstacle_points) == EControl::STOP)
      {
        stop_obstacle_waypoint = i;
        *obstacle_type = EObstacleType::ON_CROSSWALK;
        break;
      }
    }

    // waypoint in lidar point cloud frame
    geometry_msgs::Point waypoint = calcRelativeCoordinate(lane.waypoints[i].pose.pose.position, localizer_pose);
    tf::Vector3 tf_waypoint = point2vector(waypoint);
    tf_waypoint.setZ(0);

    int stop_point_count = 0;
    for (const auto& p : pcl_points)
    {
      tf::Vector3 point_vector(p.x, p.y, 0);

      // 2D distance between waypoint and points (obstacle)
      double dt = tf::tfDistance(point_vector, tf_waypoint);
      if (dt < stop_range)
      {
        stop_point_count++;
        geometry_msgs::Point point_temp;
        point_temp.x = p.x;
        point_temp.y = p.y;
        point_temp.z = p.z;
        obstacle_points->setStopPoint(calcAbsoluteCoordinate(point_temp, localizer_pose));
      }
    }

    // there is an obstacle if the number of points exceeded the threshold
    if (stop_point_count > points_threshold)
    {
      stop_obstacle_waypoint = i;
      *obstacle_type = EObstacleType::ON_WAYPOINTS;
      break;
    }

    obstacle_points->clearStopPoints();
  }

  return stop_obstacle_waypoint;
}

int detectDecelerateObstacle(const pcl::PointCloud<pcl::PointXYZ>& pcl_points, const int closest_waypoint,
                             const autoware_msgs::Lane& lane, const double stop_range, const double deceleration_range,
                             const double points_threshold, const geometry_msgs::Pose localizer_pose,
                             ObstaclePoints* obstacle_points)
{
  int decelerate_obstacle_waypoint = -1;
  // start search from the closest waypoint
  for (int i = closest_waypoint; i < closest_waypoint + DECELERATION_SEARCH_DISTANCE && i < static_cast<int>(lane.waypoints.size()); i++)
  {
    // waypoint seen by localizer
    geometry_msgs::Point waypoint = calcRelativeCoordinate(lane.waypoints[i].pose.pose.position, localizer_pose);
    tf::Vector3 tf_waypoint = point2vector(waypoint);
    tf_waypoint.setZ(0);

    int decelerate_point_count = 0;
    for (const auto& p : pcl_points)
    {
      tf::Vector3 point_vector(p.x, p.y, 0);

      // 2D distance between waypoint and points (obstacle)
      double dt = tf::tfDistance(point_vector, tf_waypoint);
      if (dt > stop_range && dt < stop_range + deceleration_range)
      {
        decelerate_point_count++;
        geometry_msgs::Point point_temp;
        point_temp.x = p.x;
        point_temp.y = p.y;
        point_temp.z = p.z;
        obstacle_points->setDeceleratePoint(calcAbsoluteCoordinate(point_temp, localizer_pose));
      }
    }

    // there is an obstacle if the number of points exceeded the threshold
    if (decelerate_point_count > points_threshold)
    {
      decelerate_obstacle_waypoint = i;
      break;
    }

    obstacle_points->clearDeceleratePoints();

    // check next waypoint...
  }

  return decelerate_obstacle_waypoint;
}

// Detect an obstacle by using pointcloud
EControl pointsDetection(const pcl::PointCloud<pcl::PointXYZ>& pcl_points, const int closest_waypoint,
                         const autoware_msgs::Lane& lane, const CrossWalk& crosswalk, const VelocitySetInfo& vs_info,
                         int* obstacle_waypoint, ObstaclePoints* obstacle_points)
{
  // no input for detection || no closest waypoint
  if ((pcl_points.empty() && vs_info.getDetectionResultByOtherNodes() == -1) || closest_waypoint < 0)
    return EControl::KEEP;

  EObstacleType obstacle_type = EObstacleType::NONE;
  int stop_obstacle_waypoint =
      detectStopObstacle(pcl_points, closest_waypoint, lane, crosswalk, vs_info.getStopRange(),
                         vs_info.getPointsThreshold(), vs_info.getLocalizerPose(),
                         obstacle_points, &obstacle_type, vs_info.getDetectionResultByOtherNodes());

  // skip searching deceleration range
  if (vs_info.getDecelerationRange() < 0.01)
  {
    *obstacle_waypoint = stop_obstacle_waypoint;
    if (stop_obstacle_waypoint < 0)
      return EControl::KEEP;
    else if (obstacle_type == EObstacleType::ON_WAYPOINTS || obstacle_type == EObstacleType::ON_CROSSWALK)
      return EControl::STOP;
    else if (obstacle_type == EObstacleType::STOPLINE)
      return EControl::STOPLINE;
    else
      return EControl::OTHERS;
  }

  int decelerate_obstacle_waypoint =
      detectDecelerateObstacle(pcl_points, closest_waypoint, lane, vs_info.getStopRange(), vs_info.getDecelerationRange(),
                               vs_info.getPointsThreshold(), vs_info.getLocalizerPose(), obstacle_points);

  // stop obstacle was not found
  if (stop_obstacle_waypoint < 0)
  {
    *obstacle_waypoint = decelerate_obstacle_waypoint;
    return decelerate_obstacle_waypoint < 0 ? EControl::KEEP : EControl::DECELERATE;
  }

  // stop obstacle was found but decelerate obstacle was not found
  if (decelerate_obstacle_waypoint < 0)
  {
    *obstacle_waypoint = stop_obstacle_waypoint;
    return EControl::STOP;
  }

  // about 5.0 meter
  double waypoint_interval =
      getPlaneDistance(lane.waypoints[0].pose.pose.position, lane.waypoints[1].pose.pose.position);
  int stop_decelerate_threshold = 5 / waypoint_interval;

  // both were found
  if (stop_obstacle_waypoint - decelerate_obstacle_waypoint > stop_decelerate_threshold)
  {
    *obstacle_waypoint = decelerate_obstacle_waypoint;
    return EControl::DECELERATE;
  }
  else
  {
    *obstacle_waypoint = stop_obstacle_waypoint;
    return EControl::STOP;
  }
}

EControl obstacleDetection(int closest_waypoint, const autoware_msgs::Lane& lane, const CrossWalk& crosswalk,
                           const VelocitySetInfo vs_info, const ros::Publisher& detection_range_pub,
                           const ros::Publisher& obstacle_pub, int* obstacle_waypoint)
{
  ObstaclePoints obstacle_points;
  EControl detection_result = pointsDetection(vs_info.getPoints(), closest_waypoint, lane, crosswalk, vs_info,
                                              obstacle_waypoint, &obstacle_points);
  displayDetectionRange(lane, crosswalk, closest_waypoint, detection_result, *obstacle_waypoint, vs_info.getStopRange(),
                        vs_info.getDecelerationRange(), detection_range_pub);

  static int false_count = 0;
  static EControl prev_detection = EControl::KEEP;
  static int prev_obstacle_waypoint = -1;

  // stop or decelerate because we found obstacles
  if (detection_result == EControl::STOP || detection_result == EControl::STOPLINE || detection_result == EControl::DECELERATE)
  {
    displayObstacle(detection_result, obstacle_points, obstacle_pub);
    prev_detection = detection_result;
    false_count = 0;
    prev_obstacle_waypoint = *obstacle_waypoint;
    return detection_result;
  }

  // there are no obstacles, but wait a little for safety
  if (prev_detection == EControl::STOP || prev_detection == EControl::STOPLINE || prev_detection == EControl::DECELERATE)
  {
    false_count++;

    if (false_count < 5)
    {
      *obstacle_waypoint = prev_obstacle_waypoint;
      displayObstacle(EControl::OTHERS, obstacle_points, obstacle_pub);
      return prev_detection;
    }
  }

  // there are no obstacles, so we move forward
  *obstacle_waypoint = -1;
  false_count = 0;
  prev_detection = EControl::KEEP;
  return detection_result;
}

void changeWaypoints(const VelocitySetInfo& vs_info, const EControl& detection_result, int closest_waypoint,
                     int obstacle_waypoint, VelocitySetPath* vs_path)
{
  double deceleration = 0.0;
  double velocity_change_limit = vs_info.getVelocityChangeLimit();

  if (detection_result == EControl::STOP || detection_result == EControl::STOPLINE)
  {
    // STOP for obstacle/stopline
    // stop_waypoint is about stop_distance meter away from obstacles/stoplines
    int stop_distance = (detection_result == EControl::STOP)
      ? vs_info.getStopDistanceObstacle() : vs_info.getStopDistanceStopline();
    deceleration = (detection_result == EControl::STOP)
      ? vs_info.getDecelerationObstacle() : vs_info.getDecelerationStopline();
    int stop_waypoint =
        calcWaypointIndexReverse(vs_path->getPrevWaypoints(), obstacle_waypoint, stop_distance);
    // change waypoints to stop by the stop_waypoint
    vs_path->changeWaypointsForStopping(stop_waypoint, obstacle_waypoint, closest_waypoint, deceleration);
  }
  else
  { // ACCELERATE, KEEP, or DECELERATE for obstacles
    vs_path->initializeNewWaypoints();
    deceleration = vs_info.getDecelerationObstacle();
    if (detection_result == EControl::DECELERATE) {
      vs_path->changeWaypointsForDeceleration(deceleration, closest_waypoint, obstacle_waypoint);
    }
  }
  vs_path->avoidSuddenAcceleration(deceleration, closest_waypoint);
  vs_path->avoidSuddenDeceleration(velocity_change_limit, deceleration, closest_waypoint);
}

}  // end namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_set");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  double update_rate;
  bool enable_crosswalk_detection;
  bool enable_multiple_crosswalk_detection;
  std::string points_topic;

  private_nh.param<double>("update_rate", update_rate, 10.0);
  private_nh.param<bool>("use_crosswalk_detection", enable_crosswalk_detection, true);
  private_nh.param<bool>("enable_multiple_crosswalk_detection", enable_multiple_crosswalk_detection, true);
  private_nh.param<std::string>("points_topic", points_topic, "points_lanes");

  CrossWalk crosswalk;
  VelocitySetPath vs_path;
  VelocitySetInfo vs_info;

  // velocity set path subscriber
  ros::Subscriber waypoints_sub = nh.subscribe("safety_waypoints", 1, &VelocitySetPath::waypointsCallback, &vs_path);
  ros::Subscriber current_vel_sub =
    nh.subscribe("current_velocity", 1, &VelocitySetPath::currentVelocityCallback, &vs_path);

  // velocity set info subscriber
  ros::Subscriber config_sub = nh.subscribe("config/velocity_set", 1, &VelocitySetInfo::configCallback, &vs_info);
  // point clouds subscriber and only points within ROI are kept in callback function.
  ros::Subscriber points_sub = nh.subscribe(points_topic, 1, &VelocitySetInfo::pointsCallback, &vs_info);
  // localizer_pose represents the lidar's pose.
  // current_pose represents the ego-vehicle's pose at the center of rear axle.
  ros::Subscriber control_pose_sub = nh.subscribe("current_pose", 1, &VelocitySetInfo::controlPoseCallback, &vs_info);
  ros::Subscriber detectionresult_sub = nh.subscribe("/state/stopline_wpidx", 1, &VelocitySetInfo::detectionCallback, &vs_info);

  // vector map subscribers
  if (enable_crosswalk_detection)
  {
    crosswalk.setMultipleDetectionFlag(enable_multiple_crosswalk_detection);
    ros::Subscriber sub_dtlane =
        nh.subscribe("vector_map_info/cross_walk", 1, &CrossWalk::crossWalkCallback, &crosswalk);
    ros::Subscriber sub_area = nh.subscribe("vector_map_info/area", 1, &CrossWalk::areaCallback, &crosswalk);
    ros::Subscriber sub_line = nh.subscribe("vector_map_info/line", 1, &CrossWalk::lineCallback, &crosswalk);
    ros::Subscriber sub_point = nh.subscribe("vector_map_info/point", 1, &CrossWalk::pointCallback, &crosswalk);
  }

  // TF Listener
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // publisher
  ros::Publisher detection_range_markers_pub = nh.advertise<visualization_msgs::MarkerArray>("detection_range", 1);
  ros::Publisher obstacle_marker_pub = nh.advertise<visualization_msgs::Marker>("obstacle", 1);
  ros::Publisher obstacle_waypoint_pub = nh.advertise<std_msgs::Int32>("obstacle_waypoint", 1, true);
  ros::Publisher stopline_waypoint_pub = nh.advertise<std_msgs::Int32>("stopline_waypoint", 1, true);
  ros::Publisher final_waypoints_pub = nh.advertise<autoware_msgs::Lane>("final_waypoints", 1, true);

  ros::Rate loop_rate(update_rate);
  while (ros::ok())
  {
    ros::spinOnce();

    try
    {
      geometry_msgs::TransformStamped map_to_lidar_tf = tfBuffer.lookupTransform(
        "map", "velodyne", ros::Time::now(), ros::Duration(2.0));
      vs_info.setLocalizerPose(map_to_lidar_tf);
    }
    catch(tf2::TransformException &ex)
    {
        ROS_WARN("Failed to get map->lidar transform. skip computation: %s", ex.what());
        continue;
    }

    // Since the index 0 of safety_waypoints from astar_avoid node holds the closest waypoint, it is set to 0.
    int32_t current_closest_waypoint = 0;
    // Initialize it to -1 which indicates no closest_crosswalk_waypoint is found.
    int32_t closest_crosswalk_waypoint = -1;

    if (!vs_info.getSetPose() || !vs_path.getSetPath() || vs_path.getPrevWaypointsSize() == 0)
    {
      loop_rate.sleep();
      continue;
    }

    if (enable_crosswalk_detection)
    {
      if (crosswalk.loaded_all && !crosswalk.set_points)
      {
        crosswalk.setCrossWalkPoints();
      }
      // if crosswalk.loaded_all is false, the closest_crosswalk_waypoint is set to -1.
      closest_crosswalk_waypoint = crosswalk.findClosestCrosswalk(current_closest_waypoint, vs_path.getPrevWaypoints(), STOP_SEARCH_DISTANCE);
    }
    crosswalk.setDetectionWaypoint(closest_crosswalk_waypoint);

    int32_t traffic_waypoint_idx = -1;
    EControl detection_result = obstacleDetection(current_closest_waypoint, vs_path.getPrevWaypoints(), crosswalk, vs_info,
                                                  detection_range_markers_pub, obstacle_marker_pub, &traffic_waypoint_idx);

    // Update waypoints' velocity profile based on obtacle detection results.
    changeWaypoints(vs_info, detection_result, current_closest_waypoint, traffic_waypoint_idx, &vs_path);

    // Only retrieve a limited number of updated waypoints ahead of the ego-vehicle.
    vs_path.setTemporalWaypoints(vs_info.getTemporalWaypointsSize(), current_closest_waypoint, vs_info.getControlPose());

    // publish final waypoints
    final_waypoints_pub.publish(vs_path.getTemporalWaypoints());

    // publish obstacle and stopline waypoint index
    std_msgs::Int32 obstacle_waypoint_index;
    std_msgs::Int32 stopline_waypoint_index;
    if (detection_result == EControl::STOP)
    {
      obstacle_waypoint_index.data = traffic_waypoint_idx;
      stopline_waypoint_index.data = -1;
    }
    else if (detection_result == EControl::STOPLINE)
    {
      obstacle_waypoint_index.data = -1;
      stopline_waypoint_index.data = traffic_waypoint_idx;
    }
    else
    {
      obstacle_waypoint_index.data = -1;
      stopline_waypoint_index.data = -1;
    }
    obstacle_waypoint_pub.publish(obstacle_waypoint_index);
    stopline_waypoint_pub.publish(stopline_waypoint_index);

    vs_path.resetFlag();
    vs_info.clearPoints();

    loop_rate.sleep();
  }

  return 0;
}

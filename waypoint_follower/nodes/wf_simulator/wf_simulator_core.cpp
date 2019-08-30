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

#include "wf_simulator/wf_simulator_core.hpp"

// clang-format off
#define DEBUG_INFO(...) { ROS_INFO(__VA_ARGS__); }

// clang-format on
WFSimulator::WFSimulator() : nh_(""), pnh_("~"), is_initialized_(false)
{
  /* wf_simulator parameters */
  pnh_.param("loop_rate", loop_rate_, double(50.0));
  pnh_.param("lidar_height", lidar_height_, double(1.0));
  nh_.param("vehicle_info/wheel_base", wheelbase_, double(2.7));
  pnh_.param("simulation_frame_id", simulation_frame_id_, std::string("base_link"));
  pnh_.param("map_frame_id", map_frame_id_, std::string("map"));
  pnh_.param("lidar_frame_id", lidar_frame_id_, std::string("velodyne"));
  pnh_.param("add_measurement_noise", add_measurement_noise_, bool(true));

  /* set pub sub topic name */
  std::string sim_pose_name, sim_lidar_pose_name, sim_velocity_name, sim_vehicle_status_name;
  pnh_.param("sim_pose_name", sim_pose_name, std::string("current_pose"));
  pnh_.param("sim_lidar_pose_name", sim_lidar_pose_name, std::string("localizer_pose"));
  pnh_.param("sim_velocity_name", sim_velocity_name, std::string("current_velocity"));
  pnh_.param("sim_vehicle_status_name", sim_vehicle_status_name, std::string("vehicle_status"));
  pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(sim_pose_name, 1);
  pub_lidar_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(sim_lidar_pose_name, 1);
  pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped>(sim_velocity_name, 1);
  pub_vehicle_status_ = nh_.advertise<autoware_msgs::VehicleStatus>(sim_vehicle_status_name, 1);
  sub_vehicle_cmd_ = nh_.subscribe("vehicle_cmd", 1, &WFSimulator::callbackVehicleCmd, this);
  timer_simulation_ = nh_.createTimer(ros::Duration(1.0 / loop_rate_), &WFSimulator::timerCallbackSimulation, this);
  timer_tf_ = nh_.createTimer(ros::Duration(0.1), &WFSimulator::timerCallbackPublishTF, this);

  bool use_waypoints_for_z_position_source;
  pnh_.param("use_waypoints_for_z_position_source", use_waypoints_for_z_position_source, bool(false));
  if (use_waypoints_for_z_position_source)
  {
    sub_waypoints_ = nh_.subscribe("base_waypoints", 1, &WFSimulator::callbackWaypoints, this);
    sub_closest_waypoint_ = nh_.subscribe("closest_waypoint", 1, &WFSimulator::callbackClosestWaypoint, this);
  }

  /* set vehicle model parameters */
  double tread_length, angvel_lim, vel_lim, steer_lim, accel_rate, angvel_rate, steer_rate_lim, vel_time_delay,
      vel_time_constant, steer_time_delay, steer_time_constant, angvel_time_delay, angvel_time_constant;
  pnh_.param("tread_length", tread_length, double(1.0));
  pnh_.param("angvel_lim", angvel_lim, double(3.0));
  pnh_.param("vel_lim", vel_lim, double(10.0));
  pnh_.param("steer_lim", steer_lim, double(3.14 / 3.0));
  pnh_.param("accel_rate", accel_rate, double(1.0));
  pnh_.param("angvel_rate", angvel_rate, double(1.0));
  pnh_.param("steer_rate_lim", steer_rate_lim, double(0.3));
  pnh_.param("vel_time_delay", vel_time_delay, double(0.25));
  pnh_.param("vel_time_constant", vel_time_constant, double(0.6197));
  pnh_.param("steer_time_delay", steer_time_delay, double(0.24));
  pnh_.param("steer_time_constant", steer_time_constant, double(0.27));
  pnh_.param("angvel_time_delay", angvel_time_delay, double(0.2));
  pnh_.param("angvel_time_constant", angvel_time_constant, double(0.5));
  const double dt = 1.0 / loop_rate_;

  /* set vehicle model type */
  std::string vehicle_model_type_str;
  pnh_.param("vehicle_model_type", vehicle_model_type_str, std::string("IDEAL_TWIST"));
  ROS_INFO("vehicle_model_type = %s", vehicle_model_type_str.c_str());
  if (vehicle_model_type_str == "IDEAL_TWIST")
  {
    vehicle_model_type_ = VehicleModelType::IDEAL_TWIST;
    vehicle_model_ptr_ = std::make_shared<WFSimModelIdealTwist>();
  }
  else if (vehicle_model_type_str == "IDEAL_STEER")
  {
    vehicle_model_type_ = VehicleModelType::IDEAL_STEER;
    vehicle_model_ptr_ = std::make_shared<WFSimModelIdealSteer>(wheelbase_);
  }
  else if (vehicle_model_type_str == "DELAY_TWIST")
  {
    vehicle_model_type_ = VehicleModelType::DELAY_TWIST;
    vehicle_model_ptr_ =
        std::make_shared<WFSimModelTimeDelayTwist>(vel_lim, angvel_lim, accel_rate, angvel_rate, dt, vel_time_delay,
                                                   vel_time_constant, angvel_time_delay, angvel_time_constant);
  }
  else if (vehicle_model_type_str == "DELAY_STEER")
  {
    vehicle_model_type_ = VehicleModelType::DELAY_STEER;
    vehicle_model_ptr_ = std::make_shared<WFSimModelTimeDelaySteer>(vel_lim, steer_lim, accel_rate, steer_rate_lim,
                                                                    wheelbase_, dt, vel_time_delay, vel_time_constant,
                                                                    steer_time_delay, steer_time_constant);
  }
  else if (vehicle_model_type_str == "CONST_ACCEL_TWIST")
  {
    vehicle_model_type_ = VehicleModelType::CONST_ACCEL_TWIST;
    vehicle_model_ptr_ = std::make_shared<WFSimModelConstantAccelTwist>(vel_lim, angvel_lim, accel_rate, angvel_rate);
  }
  else
  {
    ROS_ERROR("Invalid vehicle_model_type. Initialization failed.");
  }

  /* set normal distribution noises */
  int random_seed;
  pnh_.param("random_seed", random_seed, -1);
  if (random_seed >= 0)
  {
    rand_engine_ptr_ = std::make_shared<std::mt19937>(random_seed);
  }
  else
  {
    std::random_device seed;
    rand_engine_ptr_ = std::make_shared<std::mt19937>(seed());
  }
  double pos_noise_stddev, vel_noise_stddev, rpy_noise_stddev, angvel_noise_stddev, steer_noise_stddev;
  pnh_.param("pos_noise_stddev", pos_noise_stddev, 1e-2);
  pnh_.param("vel_noise_stddev", vel_noise_stddev, 1e-2);
  pnh_.param("rpy_noise_stddev", rpy_noise_stddev, 1e-4);
  pnh_.param("angvel_noise_stddev", angvel_noise_stddev, 1e-3);
  pnh_.param("steer_noise_stddev", steer_noise_stddev, 1e-4);
  pos_norm_dist_ptr_ = std::make_shared<std::normal_distribution<>>(0.0, pos_noise_stddev);
  vel_norm_dist_ptr_ = std::make_shared<std::normal_distribution<>>(0.0, vel_noise_stddev);
  rpy_norm_dist_ptr_ = std::make_shared<std::normal_distribution<>>(0.0, rpy_noise_stddev);
  angvel_norm_dist_ptr_ = std::make_shared<std::normal_distribution<>>(0.0, angvel_noise_stddev);
  steer_norm_dist_ptr_ = std::make_shared<std::normal_distribution<>>(0.0, steer_noise_stddev);

  /* set initialize source */
  std::string initialize_source;
  pnh_.param("initialize_source", initialize_source, std::string("ORIGIN"));
  ROS_INFO_STREAM("initialize_source : " << initialize_source);
  if (initialize_source == "RVIZ")
  {
    sub_initialpose_ = nh_.subscribe("initialpose", 1, &WFSimulator::callbackInitialPoseWithCov, this);
  }
  else if (initialize_source == "NDT")
  {
    sub_initialpose_ = nh_.subscribe("ndt_pose", 1, &WFSimulator::callbackInitialPoseStamped, this);
  }
  else if (initialize_source == "GNSS")
  {
    sub_initialpose_ = nh_.subscribe("gnss_pose", 1, &WFSimulator::callbackInitialPoseStamped, this);
  }
  else if (initialize_source == "ORIGIN")
  {
    geometry_msgs::Pose p;
    p.orientation.w = 1.0;  // yaw = 0
    geometry_msgs::Twist t;
    setInitialState(p, t);  // initialize with 0 for all variables
  }
  else
  {
    ROS_WARN("initialize_source is undesired, setting error!!");
  }
  current_pose_.orientation.w = 1.0;

  closest_pos_z_ = 0.0;
}

void WFSimulator::callbackWaypoints(const autoware_msgs::LaneConstPtr& msg)
{
  current_waypoints_ptr_ = std::make_shared<autoware_msgs::Lane>(*msg);
}

void WFSimulator::callbackClosestWaypoint(const std_msgs::Int32ConstPtr& msg)
{
  current_closest_waypoint_ptr_ = std::make_shared<std_msgs::Int32>(*msg);
  if (current_waypoints_ptr_ != nullptr)
  {
    if (-1 < msg->data && msg->data < (int)current_waypoints_ptr_->waypoints.size())
    {
      closest_pos_z_ = current_waypoints_ptr_->waypoints.at(msg->data).pose.pose.position.z;
    }
  }
}

void WFSimulator::callbackInitialPoseWithCov(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  geometry_msgs::Twist initial_twist;  // initialized with zero for all components
  setInitialStateWithPoseTransform(*msg, initial_twist);
}

void WFSimulator::callbackInitialPoseStamped(const geometry_msgs::PoseStampedConstPtr& msg)
{
  geometry_msgs::Twist initial_twist;  // initialized with zero for all components
  setInitialStateWithPoseTransform(*msg, initial_twist);
}

void WFSimulator::timerCallbackPublishTF(const ros::TimerEvent& e)
{
  publishTF(current_pose_);
}

void WFSimulator::timerCallbackSimulation(const ros::TimerEvent& e)
{
  if (!is_initialized_)
  {
    ROS_INFO_DELAYED_THROTTLE(3.0, "[wf_simulator] waiting initial position...");
    return;
  }

  if (prev_update_time_ptr_ == nullptr)
  {
    prev_update_time_ptr_ = std::make_shared<ros::Time>(ros::Time::now());
  }

  /* calculate delta time */
  const double dt = (ros::Time::now() - *prev_update_time_ptr_).toSec();
  *prev_update_time_ptr_ = ros::Time::now();

  /* update vehicle dynamics */
  vehicle_model_ptr_->updateRungeKutta(dt);

  /* save current vehicle pose & twist */
  current_pose_.position.x = vehicle_model_ptr_->getX();
  current_pose_.position.y = vehicle_model_ptr_->getY();
  current_pose_.position.z = closest_pos_z_;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = vehicle_model_ptr_->getYaw();
  current_twist_.linear.x = vehicle_model_ptr_->getVx();
  current_twist_.angular.z = vehicle_model_ptr_->getWz();

  if (add_measurement_noise_)
  {
    current_pose_.position.x += (*pos_norm_dist_ptr_)(*rand_engine_ptr_);
    current_pose_.position.y += (*pos_norm_dist_ptr_)(*rand_engine_ptr_);
    current_pose_.position.z += (*pos_norm_dist_ptr_)(*rand_engine_ptr_);
    roll += (*rpy_norm_dist_ptr_)(*rand_engine_ptr_);
    pitch += (*rpy_norm_dist_ptr_)(*rand_engine_ptr_);
    yaw += (*rpy_norm_dist_ptr_)(*rand_engine_ptr_);
    current_twist_.linear.x += (*vel_norm_dist_ptr_)(*rand_engine_ptr_);
    current_twist_.angular.z += (*angvel_norm_dist_ptr_)(*rand_engine_ptr_);
  }

  current_pose_.orientation = getQuaternionFromRPY(roll, pitch, yaw);

  /* publish pose & twist */
  publishPoseTwist(current_pose_, current_twist_);

  /* publish vehicle_statue for steering vehicle */
  autoware_msgs::VehicleStatus vs;
  vs.header.stamp = ros::Time::now();
  vs.header.frame_id = simulation_frame_id_;
  static const double MPS2KMPH = 3600.0 / 1000.0;
  vs.speed = current_twist_.linear.x * MPS2KMPH;
  vs.angle = vehicle_model_ptr_->getSteer();
  if (add_measurement_noise_)
  {
    vs.speed += (*vel_norm_dist_ptr_)(*rand_engine_ptr_) * MPS2KMPH;
    vs.angle += (*steer_norm_dist_ptr_)(*rand_engine_ptr_) * MPS2KMPH;
  }
  pub_vehicle_status_.publish(vs);
}

void WFSimulator::callbackVehicleCmd(const autoware_msgs::VehicleCmdConstPtr& msg)
{
  current_vehicle_cmd_ptr_ = std::make_shared<autoware_msgs::VehicleCmd>(*msg);
  if (vehicle_model_type_ == VehicleModelType::IDEAL_TWIST || vehicle_model_type_ == VehicleModelType::DELAY_TWIST ||
      vehicle_model_type_ == VehicleModelType::CONST_ACCEL_TWIST)
  {
    Eigen::VectorXd input(2);
    input << msg->twist_cmd.twist.linear.x, msg->twist_cmd.twist.angular.z;
    vehicle_model_ptr_->setInput(input);
  }
  else if (vehicle_model_type_ == VehicleModelType::IDEAL_STEER || vehicle_model_type_ == VehicleModelType::DELAY_STEER)
  {
    Eigen::VectorXd input(2);
    input << msg->ctrl_cmd.linear_velocity, msg->ctrl_cmd.steering_angle;
    vehicle_model_ptr_->setInput(input);
  }
  else
  {
    ROS_WARN("[%s] : invalid vehicle_model_type_  error.", __func__);
  }
}

void WFSimulator::setInitialStateWithPoseTransform(const geometry_msgs::PoseStamped& pose_stamped,
                                                   const geometry_msgs::Twist& twist)
{
  tf::StampedTransform transform;
  getTransformFromTF(map_frame_id_, pose_stamped.header.frame_id, transform);
  geometry_msgs::Pose pose;
  pose.position.x = pose_stamped.pose.position.x + transform.getOrigin().x();
  pose.position.y = pose_stamped.pose.position.y + transform.getOrigin().y();
  pose.position.z = pose_stamped.pose.position.z + transform.getOrigin().z();
  pose.orientation = pose_stamped.pose.orientation;
  setInitialState(pose, twist);
}

void WFSimulator::setInitialStateWithPoseTransform(const geometry_msgs::PoseWithCovarianceStamped& pose,
                                                   const geometry_msgs::Twist& twist)
{
  geometry_msgs::PoseStamped ps;
  ps.header = pose.header;
  ps.pose = pose.pose.pose;
  setInitialStateWithPoseTransform(ps, twist);
}

void WFSimulator::setInitialState(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& twist)
{
  const double x = pose.position.x;
  const double y = pose.position.y;
  const double yaw = tf2::getYaw(pose.orientation);
  const double vx = twist.linear.x;
  const double wz = twist.angular.z;
  const double steer = 0.0;

  if (vehicle_model_type_ == VehicleModelType::IDEAL_TWIST || vehicle_model_type_ == VehicleModelType::IDEAL_STEER)
  {
    Eigen::VectorXd state(3);
    state << x, y, yaw;
    vehicle_model_ptr_->setState(state);
  }
  else if (vehicle_model_type_ == VehicleModelType::DELAY_TWIST ||
           vehicle_model_type_ == VehicleModelType::CONST_ACCEL_TWIST)
  {
    Eigen::VectorXd state(5);
    state << x, y, yaw, vx, wz;
    vehicle_model_ptr_->setState(state);
  }
  else if (vehicle_model_type_ == VehicleModelType::DELAY_STEER)
  {
    Eigen::VectorXd state(5);
    state << x, y, yaw, vx, steer;
    vehicle_model_ptr_->setState(state);
  }
  else
  {
    ROS_WARN("undesired vehicle model type! Initialization failed.");
    return;
  }

  is_initialized_ = true;
}

void WFSimulator::getTransformFromTF(const std::string parent_frame, const std::string child_frame,
                                     tf::StampedTransform& transform)
{
  while (1)
  {
    try
    {
      tf_listener_.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
      break;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }
}

void WFSimulator::publishPoseTwist(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& twist)
{
  ros::Time current_time = ros::Time::now();

  // simulatied pose
  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = map_frame_id_;
  ps.header.stamp = current_time;
  ps.pose = pose;
  pub_pose_.publish(ps);

  // lidar pose
  ps.pose.position.z += lidar_height_;
  pub_lidar_pose_.publish(ps);

  geometry_msgs::TwistStamped ts;
  ts.header.frame_id = simulation_frame_id_;
  ts.header.stamp = current_time;
  ts.twist = twist;
  pub_twist_.publish(ts);
}

void WFSimulator::publishTF(const geometry_msgs::Pose& pose)
{
  ros::Time current_time = ros::Time::now();

  // send odom transform
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = map_frame_id_;
  odom_trans.child_frame_id = simulation_frame_id_;
  odom_trans.transform.translation.x = pose.position.x;
  odom_trans.transform.translation.y = pose.position.y;
  odom_trans.transform.translation.z = pose.position.z;
  odom_trans.transform.rotation = pose.orientation;
  tf_broadcaster_.sendTransform(odom_trans);

  // send lidar transform
  geometry_msgs::TransformStamped lidar_trans;
  lidar_trans.header.stamp = odom_trans.header.stamp;
  lidar_trans.header.frame_id = simulation_frame_id_;
  lidar_trans.child_frame_id = lidar_frame_id_;
  lidar_trans.transform.translation.z += lidar_height_;
  lidar_trans.transform.rotation.w = 1.0;
  tf_broadcaster_.sendTransform(lidar_trans);
}

geometry_msgs::Quaternion WFSimulator::getQuaternionFromRPY(const double& roll, const double& pitch, const double& yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return tf2::toMsg(q);
}

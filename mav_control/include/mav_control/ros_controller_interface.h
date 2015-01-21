/*
 * Copyright (C) 2014 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Pascal Gohl, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Sammy Omari, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * This software is released to the Contestants of the european
 * robotics challenges (EuRoC) for the use in stage 1. (Re)-distribution, whether
 * in parts or entirely, is NOT PERMITTED.
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 */

#ifndef MAV_CONTROL_ROS_CONTROLLER_INTERFACE_H
#define MAV_CONTROL_ROS_CONTROLLER_INTERFACE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/CommandAttitudeThrust.h>
#include <mav_msgs/CommandMotorSpeed.h>
#include <mav_msgs/CommandTrajectory.h>
#include <mav_msgs/MotorSpeed.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Imu.h>

#include "mav_control/controller_factory.h"

namespace mav_control {

// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultMotorVelocityReferencePubTopic = "motor_velocity_reference";
static const std::string kDefaultCommandTrajectoryTopic = "command/trajectory";
static const std::string kDefaultCommandAttitudeThrustSubTopic = "command/attitude";
static const std::string kDefaultCommandRateThrustSubTopic = "command/rate";
static const std::string kDefaultCommandMotorSpeedSubTopic = "command/motors";
static const std::string kDefaultImuSubTopic = "imu";
static const std::string kDefaultPoseSubTopic = "sensor_pose";
static const std::string kDefaultOdometrySubTopic = "odometry";

class RosControllerInterface {
 public:
  RosControllerInterface();
  ~RosControllerInterface();

  void InitializeParams();
  void Publish();

 private:
  std::shared_ptr<ControllerBase> controller_;
  bool controller_created_;

  ros::NodeHandle* node_handle_;

  std::string namespace_;
  std::string imu_sub_topic_;
  std::string pose_sub_topic_;
  std::string motor_velocity_reference_pub_topic_;
  std::string odometry_sub_topic_;

  // command topics
  std::string command_attitude_thrust_sub_topic_;
  std::string command_rate_thrust_sub_topic_;
  std::string command_motor_speed_sub_topic_;
  std::string command_trajectory_sub_topic_;

  ros::Publisher motor_velocity_reference_pub_;

  // subscribers
  ros::Subscriber cmd_attitude_sub_;
  ros::Subscriber cmd_motor_sub_;
  ros::Subscriber cmd_trajectory_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber odometry_sub_;

  void CommandAttitudeCallback(
      const mav_msgs::CommandAttitudeThrustConstPtr& input_reference_msg);
  void CommandTrajectoryCallback(
      const mav_msgs::CommandTrajectoryConstPtr& trajectory_reference_msg);
  void CommandMotorCallback(
      const mav_msgs::CommandMotorSpeedConstPtr& input_reference_msg);
  void OdometryCallback(const nav_msgs::OdometryConstPtr odometry_msg);
  void ImuCallback(const sensor_msgs::ImuConstPtr& imu);
  void PoseCallback(const geometry_msgs::PoseStampedConstPtr& pose);
};
}

#endif // MAV_CONTROL_ROS_CONTROLLER_INTERFACE_H

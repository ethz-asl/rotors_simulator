//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>, Michael Burri <burri210@gmail.com>
// All rights reserved.
//
// TODO(ff): Enter some license
//==============================================================================
#ifndef MAV_CONTROL_ROS_CONTROLLER_INTERFACE_H
#define MAV_CONTROL_ROS_CONTROLLER_INTERFACE_H

#include <mav_control/controller_factory.h>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/bind.hpp>

#include <stdio.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/CommandAttitudeThrust.h>
#include <mav_msgs/CommandMotorSpeed.h>
#include <mav_msgs/MotorSpeed.h>
#include <mav_msgs/CommandTrajectory.h>

#include <nav_msgs/Odometry.h>

namespace mav_control {


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
  std::string imu_topic_;
  std::string pose_topic_;
  std::string motor_velocity_topic_;
  std::string odometry_topic_;

  // command topics
  std::string command_topic_attitude_;
  std::string command_topic_rate_;
  std::string command_topic_motor_;
  std::string command_topic_trajectory_;

  ros::Publisher motor_cmd_pub_;

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

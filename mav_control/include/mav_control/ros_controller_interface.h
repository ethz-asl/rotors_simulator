//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>
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
#include <mav_msgs/ControlAttitudeThrust.h>
#include <mav_msgs/ControlMotorSpeed.h>
#include <mav_msgs/MotorSpeed.h>
#include <mav_msgs/ControlTrajectory.h>

#include <sensor_fusion_comm/DoubleArrayStamped.h>

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

  std::string namespace_;
  std::string command_topic_;
  std::string imu_topic_;
  std::string pose_topic_;
  std::string motor_velocity_topic_;

  ros::NodeHandle* node_handle_;
  ros::Publisher motor_cmd_pub_;
  ros::Subscriber cmd_attitude_sub_;
  ros::Subscriber cmd_motor_sub_;
  ros::Subscriber cmd_trajectory_sub_;

  ros::Subscriber imu_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber ekf_sub_;

  // Pointer to the model
//      physics::ModelPtr model_;
  // Pointer to the update event connection
//      event::ConnectionPtr updateConnection_;

  sensor_msgs::Imu imu_;

  mav_msgs::MotorSpeed turning_velocities_msg_;

//      boost::thread callback_queue_thread_;
//      void QueueThread();
  void CommandAttitudeCallback(
      const mav_msgs::ControlAttitudeThrustConstPtr& input_reference_msg);
  void CommandTrajectoryCallback(
      const mav_msgs::ControlTrajectoryConstPtr& trajectory_reference_msg);
  void CommandMotorCallback(
      const mav_msgs::ControlMotorSpeedConstPtr& input_reference_msg);
  void ExtEkfCallback(const sensor_fusion_comm::DoubleArrayStampedConstPtr ekf_state);
  void ImuCallback(const sensor_msgs::ImuConstPtr& imu);
  void PoseCallback(const geometry_msgs::PoseStampedConstPtr& pose);
};
}

#endif // MAV_CONTROL_ROS_CONTROLLER_INTERFACE_H

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


#ifndef MAV_GAZEBO_PLUGINS_CONTROLLER_INTERFACE_H
#define MAV_GAZEBO_PLUGINS_CONTROLLER_INTERFACE_H

#include <mav_control/controller_factory.h>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <stdio.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/CommandAttitudeThrust.h>
#include <mav_msgs/CommandRateThrust.h>
#include <mav_msgs/CommandMotorSpeed.h>
#include <mav_msgs/MotorSpeed.h>

namespace gazebo {
// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultMotorVelocityCommandPubTopic = "/motor_vel_ref";
static const std::string kDefaultCommandAttitudeThrustSubTopic = "/command/attitude";
static const std::string kDefaultCommandRateThrustSubTopic = "/command/rate";
static const std::string kDefaultCommandMotorSpeedSubTopic = "/command/motors";
static const std::string kDefaultImuSubTopic = "/imu";



class GazeboControllerInterface : public ModelPlugin {
 public:
  GazeboControllerInterface()
      : ModelPlugin(),
        namespace_(kDefaultNamespace),
        motor_velocity_command_pub_topic_(kDefaultMotorVelocityCommandPubTopic),
        command_attitude_thrust_sub_topic_(kDefaultCommandAttitudeThrustSubTopic),
        command_rate_thrust_sub_topic_(kDefaultCommandRateThrustSubTopic),
        command_motor_speed_sub_topic_(kDefaultCommandMotorSpeedSubTopic),
        imu_sub_topic_(kDefaultImuSubTopic),
        node_handle_(NULL),
        controller_created_(false) {}
  ~GazeboControllerInterface();

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:
  std::shared_ptr<ControllerBase> controller_;
  bool controller_created_;

  std::string namespace_;
  std::string motor_velocity_command_pub_topic_;
  std::string command_attitude_thrust_sub_topic_;
  std::string command_rate_thrust_sub_topic_;
  std::string command_motor_speed_sub_topic_;
  std::string imu_sub_topic_;

  ros::NodeHandle* node_handle_;
  ros::Publisher motor_velocity_command_pub_;
  ros::Subscriber cmd_attitude_sub_;
  ros::Subscriber cmd_rate_sub_;
  ros::Subscriber cmd_motor_sub_;
  ros::Subscriber imu_sub_;

  /// \brief Pointer to the model
  physics::ModelPtr model_;
  /// \brief Pointer to the world.
  physics::WorldPtr world_;
  /// \brief Pointer to the update event connection
  event::ConnectionPtr updateConnection_;

  sensor_msgs::Imu imu_;

  boost::thread callback_queue_thread_;
  void QueueThread();
  void CommandAttitudeCallback(const mav_msgs::CommandAttitudeThrustPtr& input_reference_msg);
  void CommandRateCallback(const mav_msgs::CommandRateThrustPtr& input_reference_msg);
  void CommandMotorCallback(const mav_msgs::CommandMotorSpeedPtr& input_reference_msg);
  void ImuCallback(const sensor_msgs::ImuPtr& imu);
  void PoseCallback(const geometry_msgs::PoseStampedPtr& pose);
};
}

#endif // MAV_GAZEBO_PLUGINS_CONTROLLER_INTERFACE_H

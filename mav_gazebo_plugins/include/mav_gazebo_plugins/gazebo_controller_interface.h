/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef MAV_GAZEBO_PLUGINS_CONTROLLER_INTERFACE_H
#define MAV_GAZEBO_PLUGINS_CONTROLLER_INTERFACE_H

#include <mav_control/controller_factory.h>

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/CommandAttitudeThrust.h>
#include <mav_msgs/CommandRateThrust.h>
#include <mav_msgs/CommandMotorSpeed.h>
#include <mav_msgs/MotorSpeed.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>

#include "mav_gazebo_plugins/common.h"

namespace gazebo {
// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultMotorVelocityReferencePubTopic = "motor_velocity_reference";
static const std::string kDefaultCommandAttitudeThrustSubTopic = "command/attitude";
static const std::string kDefaultCommandRateThrustSubTopic = "command/rate";
static const std::string kDefaultCommandMotorSpeedSubTopic = "command/motors";
static const std::string kDefaultImuSubTopic = "imu";



class GazeboControllerInterface : public ModelPlugin {
 public:
  GazeboControllerInterface()
      : ModelPlugin(),
        namespace_(kDefaultNamespace),
        motor_velocity_reference_pub_topic_(kDefaultMotorVelocityReferencePubTopic),
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
  std::string motor_velocity_reference_pub_topic_;
  std::string command_attitude_thrust_sub_topic_;
  std::string command_rate_thrust_sub_topic_;
  std::string command_motor_speed_sub_topic_;
  std::string imu_sub_topic_;

  ros::NodeHandle* node_handle_;
  ros::Publisher motor_velocity_reference_pub_;
  ros::Subscriber cmd_attitude_sub_;
  ros::Subscriber cmd_rate_sub_;
  ros::Subscriber cmd_motor_sub_;
  ros::Subscriber imu_sub_;

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  /// \brief Pointer to the update event connection.
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

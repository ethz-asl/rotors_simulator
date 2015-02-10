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


#ifndef ROTORS_GAZEBO_PLUGINS_CONTROLLER_INTERFACE_H
#define ROTORS_GAZEBO_PLUGINS_CONTROLLER_INTERFACE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <mav_msgs/CommandMotorSpeed.h>
#include <mav_msgs/MotorSpeed.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>

#include "rotors_gazebo_plugins/common.h"

namespace gazebo {
// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultMotorVelocityReferencePubTopic = "motor_velocity_reference";
static const std::string kDefaultCommandMotorSpeedSubTopic = "command/motors";

class GazeboControllerInterface : public ModelPlugin {
 public:
  GazeboControllerInterface()
      : ModelPlugin(),
        namespace_(kDefaultNamespace),
        motor_velocity_reference_pub_topic_(kDefaultMotorVelocityReferencePubTopic),
        command_motor_speed_sub_topic_(kDefaultCommandMotorSpeedSubTopic),
        node_handle_(NULL){}
  ~GazeboControllerInterface();

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:

  Eigen::VectorXd input_reference;

  std::string namespace_;
  std::string motor_velocity_reference_pub_topic_;
  std::string command_motor_speed_sub_topic_;

  ros::NodeHandle* node_handle_;
  ros::Publisher motor_velocity_reference_pub_;
  ros::Subscriber cmd_motor_sub_;

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread();
  void CommandMotorCallback(const mav_msgs::CommandMotorSpeedPtr& input_reference_msg);
};
}

#endif // ROTORS_GAZEBO_PLUGINS_CONTROLLER_INTERFACE_H

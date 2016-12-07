/*
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
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

#ifndef ROTORS_GAZEBO_PLUGINS_MSG_INTERFACE_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_MSG_INTERFACE_PLUGIN_H

#include <random>

#include <Eigen/Core>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <mav_msgs/default_topics.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "gazebo/msgs/msgs.hh"
#include "SensorImu.pb.h"

#include "rotors_gazebo_plugins/common.h"

namespace gazebo {


class GazeboMsgInterfacePlugin : public ModelPlugin {
 public:

  GazeboMsgInterfacePlugin();
  ~GazeboMsgInterfacePlugin();

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// @brief  	This gets called by the world update start event.
  /// @details	Calculates IMU parameters and then publishes one IMU message.
  void OnUpdate(const common::UpdateInfo&);

 private:
  std::string namespace_;
  std::string imu_topic_;


  /// Handle for the Gazebo node.
  //ros::NodeHandle* node_handle_;
  transport::NodePtr node_handle_;

  // Old (ROS)
  //ros::Publisher imu_pub_;
  transport::PublisherPtr imu_pub_;

  std::string link_name_;


  // Pointer to the world
  physics::WorldPtr world_;
  // Pointer to the model
  physics::ModelPtr model_;
  // Pointer to the link
  physics::LinkPtr link_;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection_;

  common::Time last_time_;

};

}  // namespace gazebo

#endif // #ifndef ROTORS_GAZEBO_PLUGINS_MSG_INTERFACE_PLUGIN_H

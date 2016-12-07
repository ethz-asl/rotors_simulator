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

#include "rotors_gazebo_plugins/gazebo_msg_interface_plugin.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

namespace gazebo {

GazeboMsgInterfacePlugin::GazeboMsgInterfacePlugin()
    : ModelPlugin(),
      node_handle_(0) {}

GazeboMsgInterfacePlugin::~GazeboMsgInterfacePlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
//  if (node_handle_) {
//    //node_handle_->shutdown();
//	  // Should we be doing this? ASL code called shutdown() then delete for ROS publisher,
//	  // but PX4 code does not touch Gazebo publisher in destructor
//    delete node_handle_;
//  }
}


void GazeboMsgInterfacePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  // default params
  namespace_.clear();

  //==============================================//
  //========== READ IN PARAMS FROM SDF ===========//
  //==============================================//

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_imu_plugin] Please specify a robotNamespace.\n";

  // Get node handle
  //node_handle_ = new ros::NodeHandle(namespace_);
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_imu_plugin] Please specify a linkName.\n";
  // Get the pointer to the link
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_imu_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  
  getSdfParam<std::string>(_sdf, "imuTopic", imu_topic_,
                           mav_msgs::default_topics::IMU);

  last_time_ = world_->GetSimTime();

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboMsgInterfacePlugin::OnUpdate, this, _1));

  // Create publisher
  //imu_pub_ = node_handle_->advertise<sensor_msgs::Imu>(imu_topic_, 1);
  imu_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Imu>("~/" + model_->GetName() + imu_topic_, 1);
}



void GazeboMsgInterfacePlugin::OnUpdate(const common::UpdateInfo& _info) {
  common::Time current_time  = world_->GetSimTime();
  double dt = (current_time - last_time_).Double();
  last_time_ = current_time;
  double t = current_time.Double();


  // Publish the IMU message
//  imu_pub_->Publish(imu_message_);

}


GZ_REGISTER_MODEL_PLUGIN(GazeboMsgInterfacePlugin);

} // namespace gazebo

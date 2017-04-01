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

#include "rotors_gazebo_plugins/gazebo_pressure_plugin.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

namespace gazebo {

GazeboPressurePlugin::GazeboPressurePlugin()
    : ModelPlugin(),
      node_handle_(0),
      pressure_sequence_(0) {
}

GazeboPressurePlugin::~GazeboPressurePlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboPressurePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model and the world
  model_ = _model;
  world_ = model_->GetWorld();

  // Use the robot namespace to create the node handle
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_pressure_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  // Use the link name as the frame id
  std::string link_name;
  if (_sdf->HasElement("linkName"))
    link_name = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_pressure_plugin] Please specify a linkName.\n";
  // Get the pointer to the link
  link_ = model_->GetLink(link_name);
  if (link_ == NULL)
    gzthrow("[gazebo_pressure_plugin] Couldn't find specified link \"" << link_name << "\".");

  frame_id_ = link_name;

  // Retrieve the rest of the SDF parameters
  getSdfParam<std::string>(_sdf, "pressureTopic", pressure_topic_, kDefaultPressurePubTopic);
  getSdfParam<double>(_sdf, "referenceAltitude", ref_alt_, kDefaultRefAlt);
  getSdfParam<double>(_sdf, "pressureVariance", pressure_var_, kDefaultPressureVar);

  // Listen to the update event. This event is broadcast every simulation iteration
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboPressurePlugin::OnUpdate, this, _1));

  pressure_pub_ = node_handle_->advertise<sensor_msgs::FluidPressure>(pressure_topic_, 1);

  // Fill the pressure message
  pressure_message_.header.frame_id = frame_id_;
  pressure_message_.variance = pressure_var_;
}

void GazeboPressurePlugin::OnUpdate(const common::UpdateInfo& _info) {
  common::Time current_time  = world_->GetSimTime();

  // Compute the geopotential altitude
  double z = ref_alt_ + model_->GetWorldPose().pos.z;
  double h = kR0 * z / (kR0 + z);

  // Compute the temperature at the current altitude
  double t = kT0 - kTl * h;

  // Compute the current air pressure
  double p = kP0 * exp(kAs * log(kT0 / t));

  // Fill the pressure message
  pressure_message_.fluid_pressure = p;
  pressure_message_.header.seq = pressure_sequence_;
  pressure_message_.header.stamp.sec = current_time.sec;
  pressure_message_.header.stamp.nsec = current_time.nsec;

  // Publish the message
  pressure_pub_.publish(pressure_message_);
  pressure_sequence_++;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboPressurePlugin);
}

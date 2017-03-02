/*
 * Copyright 2017 Pavel Vechersky, ASL, ETH Zurich, Switzerland
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

// MODULE HEADER
#include "rotors_gazebo_plugins/gazebo_fw_dynamics_plugin.h"

// SYSTEM LIBS
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>

#include "ConnectRosToGazeboTopic.pb.h"

namespace gazebo {

GazeboFwDynamicsPlugin::GazeboFwDynamicsPlugin()
    : ModelPlugin(),
      node_handle_(0),
      wind_speed_W_(0, 0, 0),
      pubs_and_subs_created_(false) {
}

GazeboFwDynamicsPlugin::~GazeboFwDynamicsPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
}

void GazeboFwDynamicsPlugin::Load(physics::ModelPtr _model,
                                  sdf::ElementPtr _sdf) {
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  gzdbg << "_model = " << _model->GetName() << std::endl;

  // Store the pointer to the model.
  model_ = _model;
  world_ = model_->GetWorld();

  namespace_.clear();

  // Get the robot namespace.
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_fw_dynamics_plugin] Please specify a robotNamespace.\n";

  // Create the node handle.
  node_handle_ = transport::NodePtr(new transport::Node());

  // Initisalise with default namespace (typically /gazebo/default/).
  node_handle_->Init();

  // Get the link name.
  std::string link_name;
  if (_sdf->HasElement("linkName"))
    link_name = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_fw_dynamics_plugin] Please specify a linkName.\n";
  // Get the pointer to the link.
  link_ = model_->GetLink(link_name);
  if (link_ == NULL) {
    gzthrow("[gazebo_fw_dynamics_plugin] Couldn't find specified link \""
            << link_name << "\".");
  }

  // Get the path to fixed-wing aerodynamics parameters YAML file. If not
  // provided, default Techpod parameters are used.
  if (_sdf->HasElement("aeroParamsYAML")) {
    std::string aero_params_yaml =
        _sdf->GetElement("aeroParamsYAML")->Get<std::string>();

    fw_params_.aero_params_.LoadAeroParamsYAML(aero_params_yaml);
  } else {
    gzwarn << "[gazebo_fw_dynamics_plugin] No aerodynamic paramaters YAML file"
        << " specified, using default Techpod parameters.\n";
  }

  std::string wind_speed_sub_topic;
  getSdfParam<std::string>(_sdf, "windSpeedSubTopic",
                           wind_speed_sub_topic,
                           mav_msgs::default_topics::WIND_SPEED);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboFwDynamicsPlugin::OnUpdate, this, _1));
}

void GazeboFwDynamicsPlugin::OnUpdate(const common::UpdateInfo& _info) {
  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }
}

void GazeboFwDynamicsPlugin::UpdateForcesAndMoments() {

}


void GazeboFwDynamicsPlugin::CreatePubsAndSubs() {
  gzdbg << __PRETTY_FUNCTION__ << " called." << std::endl;

  // Wind speed subscriber (Gazebo).
  wind_speed_sub_ =
      node_handle_->Subscribe("~/" + namespace_ + "/" + wind_speed_sub_topic_,
                              &GazeboFwDynamicsPlugin::WindSpeedCallback,
                              this);
}

void GazeboFwDynamicsPlugin::WindSpeedCallback(
    GzWindSpeedMsgPtr& wind_speed_msg) {
  if (kPrintOnMsgCallback) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  wind_speed_W_.x = wind_speed_msg->velocity().x();
  wind_speed_W_.y = wind_speed_msg->velocity().y();
  wind_speed_W_.z = wind_speed_msg->velocity().z();
}

GZ_REGISTER_MODEL_PLUGIN(GazeboFwDynamicsPlugin);

}  // namespace gazebo


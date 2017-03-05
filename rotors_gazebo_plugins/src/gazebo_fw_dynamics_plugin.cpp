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
      W_wind_speed_W_B_(0, 0, 0),
      delta_aileron_left_(0.0),
      delta_aileron_right_(0.0),
      delta_elevator_(0.0),
      delta_flap_(0.0),
      delta_rudder_(0.0),
      throttle_(0.0),
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

  getSdfParam<std::string>(_sdf, "ActuatorsSubTopic",
                           actuators_sub_topic_,
                           mav_msgs::default_topics::COMMAND_ACTUATORS);
  getSdfParam<std::string>(_sdf, "windSpeedSubTopic",
                           wind_speed_sub_topic_,
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

  UpdateForcesAndMoments();
}

void GazeboFwDynamicsPlugin::UpdateForcesAndMoments() {

}

double NormalizedInputToAngle(const ControlSurface& surface, double input) {
  return (surface.deflection_max + surface.deflection_min) * 0.5 +
      (surface.deflection_max - surface.deflection_min) * 0.5 * input;
}

void GazeboFwDynamicsPlugin::CreatePubsAndSubs() {
  gzdbg << __PRETTY_FUNCTION__ << " called." << std::endl;

  // Create temporary "ConnectRosToGazeboTopic" publisher and message
  gazebo::transport::PublisherPtr gz_connect_ros_to_gazebo_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectRosToGazeboTopic>(
          "~/" + kConnectRosToGazeboSubtopic, 1);
  gz_std_msgs::ConnectRosToGazeboTopic connect_ros_to_gazebo_topic_msg;

  // ============================================ //
  // ==== WIND SPEED MSG SETUP (ROS->GAZEBO) ==== //
  // ============================================ //

  // Wind speed subscriber (Gazebo).
  wind_speed_sub_ =
      node_handle_->Subscribe("~/" + namespace_ + "/" + wind_speed_sub_topic_,
                              &GazeboFwDynamicsPlugin::WindSpeedCallback,
                              this);

  connect_ros_to_gazebo_topic_msg.set_ros_topic(namespace_ + "/" +
                                                wind_speed_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   wind_speed_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_msgtype(
      gz_std_msgs::ConnectRosToGazeboTopic::WIND_SPEED);
  gz_connect_ros_to_gazebo_topic_pub->Publish(connect_ros_to_gazebo_topic_msg,
                                              true);

  // ============================================ //
  // ===== ACTUATORS MSG SETUP (ROS->GAZEBO) ==== //
  // ============================================ //

  actuators_sub_ =
      node_handle_->Subscribe("~/" + namespace_ + "/" + actuators_sub_topic_,
                              &GazeboFwDynamicsPlugin::ActuatorsCallback,
                              this);

  connect_ros_to_gazebo_topic_msg.set_ros_topic(namespace_ + "/" +
                                                actuators_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   actuators_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_msgtype(
      gz_std_msgs::ConnectRosToGazeboTopic::ACTUATORS);
  gz_connect_ros_to_gazebo_topic_pub->Publish(connect_ros_to_gazebo_topic_msg,
                                              true);
}

void GazeboFwDynamicsPlugin::ActuatorsCallback(
    GzActuatorsMsgPtr &actuators_msg) {
  if (kPrintOnMsgCallback) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  delta_aileron_left_ = NormalizedInputToAngle(fw_params_.aileron_left_,
      actuators_msg->normalized(fw_params_.aileron_left_.channel));
  delta_aileron_right_ = NormalizedInputToAngle(fw_params_.aileron_right_,
      actuators_msg->normalized(fw_params_.aileron_right_.channel));
  delta_elevator_ = NormalizedInputToAngle(fw_params_.elevator_,
      actuators_msg->normalized(fw_params_.elevator_.channel));
  delta_flap_ = NormalizedInputToAngle(fw_params_.flap_,
      actuators_msg->normalized(fw_params_.flap_.channel));
  delta_rudder_ = NormalizedInputToAngle(fw_params_.rudder_,
      actuators_msg->normalized(fw_params_.rudder_.channel));

  throttle_ = actuators_msg->normalized(fw_params_.throttle_channel_);
}

void GazeboFwDynamicsPlugin::WindSpeedCallback(
    GzWindSpeedMsgPtr& wind_speed_msg) {
  if (kPrintOnMsgCallback) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  W_wind_speed_W_B_.x = wind_speed_msg->velocity().x();
  W_wind_speed_W_B_.y = wind_speed_msg->velocity().y();
  W_wind_speed_W_B_.z = wind_speed_msg->velocity().z();
}

GZ_REGISTER_MODEL_PLUGIN(GazeboFwDynamicsPlugin);

}  // namespace gazebo


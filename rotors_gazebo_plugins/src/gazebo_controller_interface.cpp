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


#include "rotors_gazebo_plugins/gazebo_controller_interface.h"

namespace gazebo {

GazeboControllerInterface::~GazeboControllerInterface() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboControllerInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model.
  model_ = _model;

  world_ = model_->GetWorld();

  namespace_.clear();

  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  }
  else {
    gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";
  }

  node_handle_ = new ros::NodeHandle(namespace_);

  getSdfParam<std::string>(_sdf, "commandMotorSpeedSubTopic", command_motor_speed_sub_topic_,
                           command_motor_speed_sub_topic_);
  getSdfParam<std::string>(_sdf, "motorSpeedCommandPubTopic", motor_velocity_reference_pub_topic_,
                           motor_velocity_reference_pub_topic_);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboControllerInterface::OnUpdate, this, _1));

  cmd_motor_sub_ = node_handle_->subscribe(command_motor_speed_sub_topic_, 1,
                                           &GazeboControllerInterface::CommandMotorCallback,
                                           this);

  motor_velocity_reference_pub_ = node_handle_->advertise<mav_msgs::Actuators>(motor_velocity_reference_pub_topic_, 1);
}

// This gets called by the world update start event.
void GazeboControllerInterface::OnUpdate(const common::UpdateInfo& /*_info*/) {

  if (!received_first_reference_) {
    return;
  }

  common::Time now = world_->GetSimTime();

  mav_msgs::ActuatorsPtr turning_velocities_msg(new mav_msgs::Actuators);

  for (int i = 0; i < input_reference_.size(); i++) {
    turning_velocities_msg->angular_velocities.push_back(input_reference_[i]);
  }
  turning_velocities_msg->header.stamp.sec = now.sec;
  turning_velocities_msg->header.stamp.nsec = now.nsec;

  motor_velocity_reference_pub_.publish(turning_velocities_msg);
}

void GazeboControllerInterface::CommandMotorCallback(const mav_msgs::ActuatorsConstPtr& input_reference_msg) {
  input_reference_.resize(input_reference_msg->angular_velocities.size());
  for (int i = 0; i < input_reference_msg->angular_velocities.size(); ++i) {
    input_reference_[i] = input_reference_msg->angular_velocities[i];
  }
  received_first_reference_ = true;
}


GZ_REGISTER_MODEL_PLUGIN(GazeboControllerInterface);
}

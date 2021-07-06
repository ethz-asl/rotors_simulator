/*
 * Copyright (c) 2014 Team DIANA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "rotors_gazebo_plugins/gazebo_ros_continous_joint_controller_plugin.h"
#include <math.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <sdf/sdf.hh>
#include "rotors_comm/SetFrequency.h"
#include "rotors_gazebo_plugins/common.h"
#include "rotors_gazebo_plugins/motor_state.h"

namespace gazebo {

ContinousJointControllerPlugin::ContinousJointControllerPlugin()
    : alive_(true) {}

// Destructor
ContinousJointControllerPlugin::~ContinousJointControllerPlugin() {
  delete nh_;
}

// Load the controller
void ContinousJointControllerPlugin::Load(physics::ModelPtr parent,
                                          sdf::ElementPtr sdf) {
  this->parent_ = parent;
  this->world_ = parent->GetWorld();
  getSdfParam<std::string>(sdf, "robot_namespace", robot_namespace_, "");

  joint_ = GetReferencedJoint(parent, sdf, "joint");

  if (joint_ == nullptr) {
    ROS_FATAL("No joint was found");
    return;
  }

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM(
        "A ROS node for Gazebo has not been initialized, unable to load "
        "plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the "
           "gazebo_ros package)");
    return;
  }

  nh_ = new ros::NodeHandle(this->robot_namespace_);

  // no need for position control here
  current_motor_state_.mode = MotorStateMode::Velocity;
  current_motor_state_.torque_enabled = true;

  // read sdf parameters
  getSdfParam<double>(sdf, "default_pos", current_motor_state_.goal_pos_rad,
                      0.0);
  current_motor_state_.current_pos_rad = current_motor_state_.goal_pos_rad;
  getSdfParam<double>(sdf, "default_velocity_limit",
                      current_motor_state_.velocity_limit_rad_s, 100.0);
  getSdfParam<double>(sdf, "default_torque_limit",
                      current_motor_state_.torque_limit, 10);
  getSdfParam<std::string>(sdf, "frequency_service_name", service_name_,
                           "set_frequency");

  std::string joint_state_topic_name;
  getSdfParam<std::string>(sdf, "joint_state_topic_name",
                           joint_state_topic_name, "joint_state");
  joint_->SetPosition(0, current_motor_state_.current_pos_rad);
  getSdfParam<double>(sdf, "spin_frequency_hz", spin_frequency_, 1.0);

  SetVelocity(spin_frequency_);

  InitServices();

  // listen to the update event (broadcast every simulation iteration)
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&ContinousJointControllerPlugin::OnWorldUpdate, this));
  ROS_INFO("Motor control plugin loaded");
}

void ContinousJointControllerPlugin::SetVelocity(double spin_frequency) {
  current_motor_state_.velocity_rad_s = 2.0 * M_PI * spin_frequency;
}

void ContinousJointControllerPlugin::InitServices() {
  set_frequency_service_ = nh_->advertiseService(
      service_name_,
      (boost::function<bool(rotors_comm::SetFrequencyRequest&,
                            rotors_comm::SetFrequencyResponse&)>)([&](
          rotors_comm::SetFrequencyRequest& req,
          rotors_comm::SetFrequencyResponse& res) {
        SetVelocity(req.frequency);
        current_motor_state_.mode = MotorStateMode::Velocity;
        return true;
      }));
}

// Finalize the controller
void ContinousJointControllerPlugin::Shutdown() {
  alive_ = false;
  nh_->shutdown();
}

MotorState gazebo::ContinousJointControllerPlugin::ReadMotor() const {
  MotorState read_motor_state = current_motor_state_;
  read_motor_state.current_pos_rad = joint_->GetAngle(0).Radian();
  read_motor_state.is_moving =
      read_motor_state.velocity_rad_s != 0 && read_motor_state.torque_enabled;
  read_motor_state.load = joint_->GetForceTorque(0).body2Torque.x;
  return read_motor_state;
}

void ContinousJointControllerPlugin::UpdateMotor(
    const MotorState& read_motor_state) {
  if (joint_->GetParam("vel", 0) != current_motor_state_.velocity_rad_s) {
    joint_->SetParam("vel", 0, current_motor_state_.velocity_rad_s);
    ROS_INFO("target velocity set to %f", current_motor_state_.velocity_rad_s);
  }

  if (read_motor_state.torque_enabled) {
    joint_->SetParam("fmax", 0, read_motor_state.torque_limit);
  } else {
    joint_->SetParam("fmax", 0, 0.0);
  }
}

void ContinousJointControllerPlugin::OnWorldUpdate() {
  current_motor_state_ = ReadMotor();
  UpdateMotor(current_motor_state_);
}

physics::JointPtr ContinousJointControllerPlugin::GetReferencedJoint(
    physics::ModelPtr parent, sdf::ElementPtr sdf,
    const std::string& jointParameterName) {
  std::string jointName;
  getSdfParam<std::string>(sdf, jointParameterName, jointName, "");
  return parent->GetJoint(jointName);
}

GZ_REGISTER_MODEL_PLUGIN(ContinousJointControllerPlugin)
}

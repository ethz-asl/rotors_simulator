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

#include "rotors_control/controller_base.h"

#include <math.h>

#define _USE_MATH_DEFINES

ControllerBase::ControllerBase() {
  initialized_params_ = false;

  position_.setZero();
  velocity_.setZero();
  attitude_.setIdentity();
  angular_rate_.setZero();

  position_reference_.setZero();
  velocity_reference_.setZero();
  acceleration_reference_.setZero();
  jerk_reference_.setZero();
  yaw_reference_ = 0;
  yaw_rate_reference_ = 0;

  control_attitude_thrust_reference_.setZero();
  control_rate_thrust_reference_.setZero();
  amount_rotors_ = 0;
}

ControllerBase::~ControllerBase() {
}

void ControllerBase::SetPosition(const Eigen::Vector3d& position) {
  position_ = position;
}

void ControllerBase::SetVelocity(const Eigen::Vector3d& velocity) {
  velocity_ = velocity;
}

void ControllerBase::SetAttitude(const Eigen::Quaternion<double>& attitude) {
  attitude_ = attitude;
}

void ControllerBase::SetAngularRate(const Eigen::Vector3d& angular_rate) {
  angular_rate_ = angular_rate;
}

void ControllerBase::SetReferenceAttitudeThrust(const Eigen::Vector4d& control_attitude_thrust_reference) {
  // default implementation to make the user aware that he's doing something wrong.
  ROS_WARN_STREAM_THROTTLE(2, "[ControllerBase]: Derived controller does not understand/accept AttitudeThrustCommands");
  //control_attitude_thrust_reference_ = control_attitude_thrust_reference;
}

void ControllerBase::SetReferenceRateThrust(const Eigen::Vector4d& control_rate_thrust_reference) {
  control_rate_thrust_reference_ = control_rate_thrust_reference;
}

void ControllerBase::SetReferenceMotor(const Eigen::VectorXd& motor_reference) {
  motor_reference_ = motor_reference;
}

void ControllerBase::SetReferencePosition(const Eigen::Vector3d& position_reference) {
  position_reference_ = position_reference;
}

void ControllerBase::SetReferenceVelocity(const Eigen::Vector3d& velocity_reference) {
  velocity_reference_ = velocity_reference;
}

void ControllerBase::SetReferenceAcceleration(const Eigen::Vector3d& acceleration_reference) {
  acceleration_reference_ = acceleration_reference;
}

void ControllerBase::SetReferenceJerk(const Eigen::Vector3d& jerk_reference) {
  jerk_reference_ = jerk_reference;
}

void ControllerBase::SetReferenceYaw(double yaw_reference) {
  yaw_reference_ = yaw_reference;
}

void ControllerBase::SetReferenceYawRate(double yaw_rate_reference) {
  yaw_rate_reference_ = yaw_rate_reference;
}

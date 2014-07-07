/*
 * Copyright (C) 2014 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Pascal Gohl, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Sammy Omari, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * This software is released to the Contestants of the european 
 * robotics challenges (EuRoC) for the use in stage 1. (Re)-distribution, whether 
 * in parts or entirely, is NOT PERMITTED. 
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 */

#include <mav_control/controller_base.h>
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

void ControllerBase::SetAttitudeThrustReference(const Eigen::Vector4d& control_attitude_thrust_reference) {
  control_attitude_thrust_reference_ = control_attitude_thrust_reference;
}

void ControllerBase::SetRateThrustReference(const Eigen::Vector4d& control_rate_thrust_reference) {
  control_rate_thrust_reference_ = control_rate_thrust_reference;
}

void ControllerBase::SetMotorReference(const Eigen::VectorXd& motor_reference) {
  motor_reference_ = motor_reference;
}

void ControllerBase::SetPositionReference(const Eigen::Vector3d& position_reference) {
  position_reference_ = position_reference;
}

void ControllerBase::SetVelocityReference(const Eigen::Vector3d& velocity_reference) {
  velocity_reference_ = velocity_reference;
}

void ControllerBase::SetAccelerationReference(const Eigen::Vector3d& acceleration_reference) {
  acceleration_reference_ = acceleration_reference;
}

void ControllerBase::SetJerkReference(const Eigen::Vector3d& jerk_reference) {
  jerk_reference_ = jerk_reference;
}

void ControllerBase::SetYawReference(double yaw_reference) {
  yaw_reference_ = yaw_reference;
}

void ControllerBase::SetYawRateReference(double yaw_rate_reference) {
  yaw_rate_reference_ = yaw_rate_reference;
}

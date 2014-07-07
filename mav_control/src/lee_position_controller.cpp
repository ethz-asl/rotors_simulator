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

#include <mav_control/lee_position_controller.h>
#include <iostream>

LeePositionController::LeePositionController()
    : gravity_(9.81),
      mass_(1.56779) {
}

LeePositionController::~LeePositionController() {
}

std::shared_ptr<ControllerBase> LeePositionController::Clone() {
  std::shared_ptr<ControllerBase> controller(new LeePositionController);
  return controller;
}

void LeePositionController::InitializeParams() {

  gain_position_(0) = 6; //8;
  gain_position_(1) = 6; //8
  gain_position_(2) = 6; //8

  gain_velocity_(0) = 4.7; //5.3;
  gain_velocity_(1) = 4.7; //5.3;
  gain_velocity_(2) = 4.7; //5.3;

  gain_attitude_(0) = 3; //4
  gain_attitude_(1) = 3; //4
  gain_attitude_(2) = 0.035;

  gain_angular_rate_(0) = 0.52;//0.6;
  gain_angular_rate_(1) = 0.52;//0.6;
  gain_angular_rate_(2) = 0.025;

  amount_rotors_ = 6;
  allocation_matrix_.resize(4,amount_rotors_);
  allocation_matrix_ << sin(M_PI/6),  1,  sin(M_PI/6), -sin(M_PI/6), -1, -sin(M_PI/6),
                       -cos(M_PI/6),  0,  cos(M_PI/6),  cos(M_PI/6), 0, -cos(M_PI/6),
                       -1,  1, -1,  1, -1, 1,
                        1,  1,  1,  1, 1, 1;

  inertia_matrix_<< 0.0347563,  0,  0,
                    0,  0.0458929,  0,
                    0,  0, 0.0977;

  // to make the tuning independent of the inertia matrix we divide here
  gain_attitude_ = gain_attitude_.transpose() * inertia_matrix_.inverse();

  // to make the tuning independent of the inertia matrix we divide here
  gain_angular_rate_ = gain_angular_rate_.transpose() * inertia_matrix_.inverse();

  const double rotor_force_constant = 0.00000854858;  //F_i = k_n * rotor_velocity_i^2
  const double rotor_moment_constant = 0.016;  // M_i = k_m * F_i

  angular_acc_to_rotor_velocities_.resize(amount_rotors_, 4);
  const double arm_length = 0.215;

  Eigen::Matrix4d K;
  K.setZero();
  K(0, 0) = arm_length * rotor_force_constant;
  K(1, 1) = arm_length * rotor_force_constant;
  K(2, 2) = rotor_force_constant * rotor_moment_constant;
  K(3, 3) = rotor_force_constant;

  Eigen::Matrix4d I;
  I.setZero();
  I.block<3, 3>(0, 0) = inertia_matrix_;
  I(3, 3) = 1;
  angular_acc_to_rotor_velocities_ = allocation_matrix_.transpose()
      * (allocation_matrix_ * allocation_matrix_.transpose()).inverse() * K.inverse() * I;

  initialized_params_ = true;
}

void LeePositionController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const {
  assert(rotor_velocities);
  assert(initialized_params_);

  rotor_velocities->resize(amount_rotors_);

  Eigen::Vector3d acceleration;
  ComputeDesiredAcceleration(&acceleration);

  Eigen::Vector3d angular_acceleration;
  ComputeDesiredAngularAcc(acceleration, &angular_acceleration);

  // project thrust to body z axis.
  Eigen::Vector3d e_3(0, 0, 1);
  double thrust = -mass_ * acceleration.dot(attitude_.toRotationMatrix() * e_3);

  Eigen::Vector4d angular_acceleration_thrust;
  angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
  angular_acceleration_thrust(3) = thrust;

  *rotor_velocities = angular_acc_to_rotor_velocities_ * angular_acceleration_thrust;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
}

void LeePositionController::ComputeDesiredAcceleration(Eigen::Vector3d* acceleration) const {
  assert(acceleration);

  Eigen::Vector3d position_error;
  position_error = position_ - position_reference_;

  Eigen::Vector3d velocity_error;
  velocity_error = velocity_ - velocity_reference_;

  Eigen::Vector3d e_3(0, 0, 1);

  *acceleration = position_error.cwiseProduct(gain_position_) / mass_
      + velocity_error.cwiseProduct(gain_velocity_) / mass_ - gravity_ * e_3 - acceleration_reference_;
}

// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void LeePositionController::ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
                                                     Eigen::Vector3d* angular_acceleration) const {
  assert(angular_acceleration);

  Eigen::Matrix3d R = attitude_.toRotationMatrix();

  // get desired rotation matrix
  Eigen::Vector3d b1_des;
  b1_des << cos(yaw_reference_), sin(yaw_reference_), 0;

  Eigen::Vector3d b3_des;
  b3_des = -acceleration / acceleration.norm();

  Eigen::Vector3d b2_des;
  b2_des = b3_des.cross(b1_des);
  b2_des.normalize();

  Eigen::Matrix3d R_des;
  R_des.col(0) = b2_des.cross(b3_des);
  R_des.col(1) = b2_des;
  R_des.col(2) = b3_des;

  // angle error according to lee et al.
  Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
  Eigen::Vector3d angle_error;
  angle_error << angle_error_matrix(2, 1),  // inverse skew operator
  angle_error_matrix(0, 2), angle_error_matrix(1, 0);

  // TODO(burrimi) include angular rate references at some point.
  Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
  angular_rate_des[2] = control_attitude_thrust_reference_(2);

  Eigen::Vector3d angular_rate_error = angular_rate_ - R_des.transpose() * R * angular_rate_des;

  *angular_acceleration = -1 * angle_error.cwiseProduct(gain_attitude_)
                           - angular_rate_error.cwiseProduct(gain_angular_rate_)
                           + angular_rate_.cross(angular_rate_); // we don't need the inertia matrix here
}


MAV_CONTROL_REGISTER_CONTROLLER(LeePositionController);

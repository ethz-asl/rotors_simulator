#include <mav_control/rate_controller.h>
#include <iostream>


RateController::RateController() {}

RateController::~RateController() {}

std::shared_ptr<ControllerBase> RateController::Clone() {
  std::shared_ptr<ControllerBase> controller(new RateController);
  return controller;
}

void RateController::InitializeParams() {
  amount_rotors_ = 6;
  allocation_matrix_.resize(4,amount_rotors_);
  allocation_matrix_ << sin(M_PI/6),  1,  sin(M_PI/6), -sin(M_PI/6), -1, -sin(M_PI/6),
                       -cos(M_PI/6),  0,  cos(M_PI/6),  cos(M_PI/6), 0, -cos(M_PI/6),
                       -1,  1, -1,  1, -1, 1,
                        1,  1,  1,  1, 1, 1;

  inertia_matrix_<< 0.0393,  0,  0,
                    0,  0.048,  0,
                    0,  0, 0.0977;

  gain_angular_rate_(0) = 0.1;
  gain_angular_rate_(1) = 0.1;
  gain_angular_rate_(2) = 0.025;

  // to make the tuning independent of the inertia matrix we divide here
  gain_angular_rate_ = gain_angular_rate_.transpose() * inertia_matrix_.inverse();
  const double mass = 1.5;
  const double motor_force_constant = 0.00001005; //F_i = k_n * rotor_velocity_i^2
  const double motor_moment_constant = 0.0243; // M_i = k_m * F_i

  angular_acc_to_rotor_velocities_.resize(amount_rotors_, 4);
  const double arm_length = 0.215;

  Eigen::Matrix4d K;
  K.setZero();
  K(0,0) = arm_length * motor_force_constant;
  K(1,1) = arm_length * motor_force_constant;
  K(2,2) = motor_force_constant * motor_moment_constant;
  K(3,3) = motor_force_constant;

  Eigen::Matrix4d I;
  I.setZero();
  I.block<3,3>(0, 0) = inertia_matrix_;
  I(3,3) = 1;
  angular_acc_to_rotor_velocities_ = allocation_matrix_.transpose()
    * (allocation_matrix_ * allocation_matrix_.transpose()).inverse() * K.inverse() * I;
  initialized_params_ = true;
}

void RateController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const {
  assert(rotor_velocities);
  assert(initialized_params_);

  rotor_velocities->resize(amount_rotors_);

  Eigen::Vector3d angular_acceleration;
  ComputeDesiredAngularAcc(&angular_acceleration);

  Eigen::Vector4d angular_acceleration_thrust;
  angular_acceleration_thrust.block<3,1>(0,0) = angular_acceleration;
  angular_acceleration_thrust(3) = control_rate_thrust_reference_(3);

  *rotor_velocities = angular_acc_to_rotor_velocities_ * angular_acceleration_thrust;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
}

// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void RateController::ComputeDesiredAngularAcc(Eigen::Vector3d* angular_acceleration) const {
  assert(angular_acceleration);

  // TODO(burrimi) include angular rate references at some point.
  Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
  angular_rate_des[0] = control_rate_thrust_reference_(0);
  angular_rate_des[1] = control_rate_thrust_reference_(1);
  angular_rate_des[2] = control_rate_thrust_reference_(2);

  Eigen::Vector3d angular_rate_error = angular_rate_ -  angular_rate_des;

  *angular_acceleration =  - angular_rate_error.cwiseProduct(gain_angular_rate_)
                           + angular_rate_.cross(angular_rate_); // we don't need the inertia matrix here
}


MAV_CONTROL_REGISTER_CONTROLLER(RateController);

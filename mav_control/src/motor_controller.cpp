#include <mav_control/motor_controller.h>
#include <iostream>

MotorController::MotorController() {
}

MotorController::~MotorController() {
}

std::shared_ptr<ControllerBase> MotorController::Clone() {
  std::shared_ptr<ControllerBase> controller(new MotorController);
  return controller;
}

void MotorController::InitializeParams() {
  amount_rotors_ = 6;
  initialized_params_ = true;
}

void MotorController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const {
  assert(rotor_velocities);
  assert(initialized_params_);

  rotor_velocities->resize(amount_rotors_);

  *rotor_velocities = motor_reference_;
}

MAV_CONTROL_REGISTER_CONTROLLER(MotorController);

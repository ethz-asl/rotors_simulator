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

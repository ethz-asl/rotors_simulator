#include <mav_control/attitude_controller.h>
#include <iostream>


AttitudeController::AttitudeController() {}

void AttitudeController::InitializeParams(){
  std::cout<<"initializing\n";
  amount_rotors_ = 4;
}

void AttitudeController::Publish(){
  std::cout<<"publishing\n";
}

void AttitudeController::UpdateStates(){
  // request updated states from system
  // position_ = ;
  // velocity_ = ;
  // attitude_ = ;
  // omega_ = ;
}

void AttitudeController::CalculateRefMotorVelocities() {
  // control_attitude_thrust_ = control_input_;
  ref_rotor_rot_vels_ = Eigen::VectorXd::Zero(amount_rotors_);
}

MAV_CONTROL_REGISTER_CONTROLLER(attitude_controller, AttitudeController);

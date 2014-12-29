#include <mav_control/px4_att_controller.h>
#include <iostream>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>

PX4_AttitudeController::PX4_AttitudeController() {
  time_last = ros::Time::now().toNSec() / 1e3;
}

PX4_AttitudeController::~PX4_AttitudeController() {
}

std::shared_ptr<ControllerBase> PX4_AttitudeController::Clone() {
  std::shared_ptr<ControllerBase> controller(new PX4_AttitudeController);

  return controller;
}

void PX4_AttitudeController::InitializeParams() {

  amount_rotors_ = 4;
  initialized_params_ = true;

}

void PX4_AttitudeController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const {
  // *rotor_velocities = mixer_output * 1000;    // bring the data out
}


void PX4_AttitudeController::ComputeDesiredAngularAcc(Eigen::Vector3d* angular_acceleration) const {

}

float PX4_AttitudeController::time_since_last_call() const {
  float now = ros::Time::now().toNSec() / 1e3;
  return (now - time_last) / 1e6;
}

MAV_CONTROL_REGISTER_CONTROLLER(PX4_AttitudeController);

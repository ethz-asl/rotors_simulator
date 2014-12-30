#include <mav_control/px4_dummy_controller.h>
#include <iostream>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <px4/actuator_controls_0.h>

PX4dummyController::PX4dummyController() {
  _sub = _n.subscribe("mixed_motor_commands",1000,&PX4dummyController::MotorVelCallback,this);
}

PX4dummyController::~PX4dummyController() {
}

std::shared_ptr<ControllerBase> PX4dummyController::Clone() {
  std::shared_ptr<ControllerBase> controller(new PX4dummyController);

  return controller;
}

void PX4dummyController::InitializeParams() {
  
  amount_rotors_ = 4;
  initialized_params_ = true;
  
}

void PX4dummyController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const{
  Eigen::VectorXd mixer_output;
  rotor_velocities->resize(amount_rotors_);
  for(int i = 0;i < amount_rotors_;i++) {
    (*rotor_velocities)(i) = _motor_speeds[i];
  }
}

void PX4dummyController::MotorVelCallback(const mav_msgs::MotorSpeed &msg) {
  for(int i = 0;i<amount_rotors_;i++) {
    _motor_speeds[i] = msg.motor_speed[i];
  }
}

void PX4dummyController::ComputeDesiredAngularAcc(Eigen::Vector3d* angular_acceleration) const {
  
}


MAV_CONTROL_REGISTER_CONTROLLER(PX4dummyController);
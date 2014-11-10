#include <mav_control/px4_att_controller.h>
#include <iostream>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>

PX4_AttitudeController::PX4_AttitudeController() {
  _PX4_base_att_controller = new MulticopterAttitudeControlBase();
  _mc_mixer                = new MultirotorMixer();
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
  rotor_velocities->resize(amount_rotors_);
  
  // first we need to update our data in the base class
  _PX4_base_att_controller->set_attitude(attitude_);
  _PX4_base_att_controller->set_attitude_reference(control_attitude_thrust_reference_);
  _PX4_base_att_controller->set_attitude_rates(angular_rate_);
   // now start control
  _PX4_base_att_controller->control_attitude(time_since_last_call());        // first control attitude
  _PX4_base_att_controller->control_attitude_rates(time_since_last_call());  // control attitude rates
  _PX4_base_att_controller->set_actuator_controls();                         // write result to struct
  
  // get the actuator inputs from controller
  Eigen::Vector4d mixer_input;
  _PX4_base_att_controller->get_mixer_input(mixer_input);
      
  // now mix and save motor commands
  Eigen::VectorXd mixer_output;
  mixer_output.resize(amount_rotors_);
  _mc_mixer->mix(&mixer_input,mixer_output);
  *rotor_velocities = mixer_output * 1000;    // bring the data out
}


void PX4_AttitudeController::ComputeDesiredAngularAcc(Eigen::Vector3d* angular_acceleration) const {
  
}

float PX4_AttitudeController::time_since_last_call() const {
  float now = ros::Time::now().toNSec() / 1e3;
  return (now - time_last) / 1e6;
}

MAV_CONTROL_REGISTER_CONTROLLER(PX4_AttitudeController);
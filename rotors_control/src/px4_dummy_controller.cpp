/* Copyright (c) 2014 PX4 Development Team. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in
* the documentation and/or other materials provided with the
* distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
* used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
    * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*g mav
****************************************************************************/

/**
 * @file px4_dummy_controller.cpp
 * Multicopter dummy controller to allow interface between euroc simulator and PX4
 * apps running on ROS
 *
 * @author Roman Bapst <romanbapst@yahoo.de>
 *
 */

#include <rotors_control/px4_dummy_controller.h>
#include <iostream>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>

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
  //XXX sometimes this is called on a uninitialized instance: find out why!
  InitializeParams();

  for(int i = 0;i<amount_rotors_;i++) {
    _motor_speeds[i] = msg.motor_speed[i];
  }
}

void PX4dummyController::ComputeDesiredAngularAcc(Eigen::Vector3d* angular_acceleration) const {

}


ROTORS_CONTROL_REGISTER_CONTROLLER(PX4dummyController);

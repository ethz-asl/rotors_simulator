/*
 * Copyright 2018 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2018 Luigi Iannelli, University of Sannio in Benevento, Italy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <mav_msgs/eigen_mav_msgs.h>

#include "rotors_control/transform_datatypes.h"
#include "rotors_control/Matrix3x3.h"
#include "rotors_control/Quaternion.h"

#include "rotors_control/crazyflie_onboard_controller.h"

#define SAMPLING_TIME_ATTITUDE_CONTROLLER         0.004 /* SAMPLING TIME ATTITUDE CONTROLLER [s] */
#define SAMPLING_TIME_RATE_CONTROLLER             0.002 /* SAMPLING TIME RATE CONTROLLER [s] */

namespace rotors_control{

CrazyflieOnboardController::CrazyflieOnboardController()
    : counter_(false),
    delta_psi_ki_(0),
    p_command_ki_(0), 
    q_command_ki_(0),
    p_command_(0),
    q_command_(0){

}

CrazyflieOnboardController::~CrazyflieOnboardController() {}

// Make a copy of control signals and get them private
void CrazyflieOnboardController::SetControlSignals(const control_s& control_t) {
    
    control_t_private_ = control_t;    
}

// Make a copy of the drone state and get it private
void CrazyflieOnboardController::SetDroneState(const state_s& state_t) {
    
    state_t_private_ = state_t;    
}

// Set the controller gains as local global variables
void CrazyflieOnboardController::SetControllerGains(PositionControllerParameters& controller_parameters_) {
    
      attitude_gain_kp_private_ = Eigen::Vector2f(controller_parameters_.attitude_gain_kp_.x(), controller_parameters_.attitude_gain_kp_.y());
      attitude_gain_ki_private_ = Eigen::Vector2f(controller_parameters_.attitude_gain_ki_.x(), controller_parameters_.attitude_gain_ki_.y());
  
      rate_gain_kp_private_ = Eigen::Vector3f(controller_parameters_.rate_gain_kp_.x(), controller_parameters_.rate_gain_kp_.y(), controller_parameters_.rate_gain_kp_.z());
      rate_gain_ki_private_ = Eigen::Vector3f(controller_parameters_.rate_gain_ki_.x(), controller_parameters_.rate_gain_ki_.y(), controller_parameters_.rate_gain_ki_.z());

      ROS_DEBUG("Attitude gains - Kp_p: %f, Ki_p: %f, Kp_q: %f, Ki_q: %f", attitude_gain_kp_private_.x(),
                 attitude_gain_ki_private_.x(), attitude_gain_kp_private_.y(), attitude_gain_ki_private_.y());
      ROS_DEBUG("Rate gains - Kp_phi: %f, Ki_phi: %f, Kp_theta: %f, Ki_theta: %f, Kp_psi: %f, Ki_psi: %f", rate_gain_kp_private_.x(),
                 rate_gain_ki_private_.x(), rate_gain_kp_private_.y(), rate_gain_ki_private_.y(), rate_gain_kp_private_.z(),
                 rate_gain_ki_private_.z());
  
}

void CrazyflieOnboardController::RateController(double* delta_phi, double* delta_theta, double* delta_psi) {
    assert(delta_phi);
    assert(delta_theta);
    assert(delta_psi);
    
    double p, q, r;
    p = state_t_private_.angularVelocity.x;
    q = state_t_private_.angularVelocity.y;
    r = state_t_private_.angularVelocity.z;

    double r_command;
    r_command = control_t_private_.yawRate;

    // Update the p and q commands with a frequency rate of 250Hz. The rate controller works with a frequency rate of 500Hz
    if(counter_){
       AttitudeController(&p_command_, &q_command_);
       counter_ = false;   
    }
    else 
      counter_ = true;

    double p_error, q_error, r_error;
    p_error = p_command_ - p;
    q_error = q_command_ - q;
    r_error = r_command - r;

    ROS_DEBUG("p_command: %f, q_command: %f", p_command_, q_command_);

    double delta_phi_kp, delta_theta_kp, delta_psi_kp;
    delta_phi_kp = rate_gain_kp_private_.x() * p_error;
    *delta_phi = delta_phi_kp;

    delta_theta_kp = rate_gain_kp_private_.y() * q_error;
    *delta_theta = delta_theta_kp;

    delta_psi_kp = rate_gain_kp_private_.z() * r_error;
    delta_psi_ki_ = delta_psi_ki_ + (rate_gain_ki_private_.z() * r_error * SAMPLING_TIME_RATE_CONTROLLER);
    *delta_psi = delta_psi_kp + delta_psi_ki_;

}

// The attitude controller runs with a frequency rate of 250Hz
void CrazyflieOnboardController::AttitudeController(double* p_command_internal, double* q_command_internal) {
    assert(p_command_internal);
    assert(q_command_internal); 

    double roll, pitch, yaw;
    Quaternion2Euler(&roll, &pitch, &yaw);  

    double theta_command, phi_command;
    theta_command = control_t_private_.pitch;
    phi_command = control_t_private_.roll;

    double phi_error, theta_error;
    phi_error = phi_command - roll;
    theta_error = theta_command - pitch;

    double p_command_kp, q_command_kp;
    p_command_kp = attitude_gain_kp_private_.x() * phi_error;
    p_command_ki_ = p_command_ki_ + (attitude_gain_ki_private_.x() * phi_error * SAMPLING_TIME_ATTITUDE_CONTROLLER);
    *p_command_internal = p_command_kp + p_command_ki_;

    q_command_kp = attitude_gain_kp_private_.y() * theta_error;
    q_command_ki_ = q_command_ki_ + (attitude_gain_ki_private_.y() * theta_error * SAMPLING_TIME_ATTITUDE_CONTROLLER);
    *q_command_internal = q_command_kp + q_command_ki_;

    ROS_INFO_ONCE("The p and q values have updated");

    ROS_DEBUG("Phi_c: %f, Phi_e: %f, Theta_c: %f, Theta_e: %f", phi_command, phi_error, theta_command, theta_error);
    ROS_DEBUG("p_command: %f, q_command: %f", *p_command_internal, *q_command_internal);
    
}

void CrazyflieOnboardController::Quaternion2Euler(double* roll, double* pitch, double* yaw) const {
    assert(roll);
    assert(pitch);
    assert(yaw);

    //The estimated quaternion values
    double x, y, z, w;
    x = state_t_private_.attitudeQuaternion.x;
    y = state_t_private_.attitudeQuaternion.y;
    z = state_t_private_.attitudeQuaternion.z;
    w = state_t_private_.attitudeQuaternion.w;
    
    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    m.getRPY(*roll, *pitch, *yaw);
   
    ROS_DEBUG("Roll: %f, Pitch: %f, Yaw: %f", *roll, *pitch, *yaw);
}

}

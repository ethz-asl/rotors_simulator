/*
 * Copyright 2018 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2018 Emanuele Aucone, University of Sannio in Benevento, Italy
 * Copyright 2018 Benjamin Rodriguez, MIT, USA
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

#include "rotors_control/position_controller.h"
#include "rotors_control/transform_datatypes.h"

#include <math.h> 

#include <angles/angles.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <bullet/LinearMath/btQuaternion.h>


#define M_PI                      3.14159265358979323846  /* pi */
#define OMEGA_OFFSET              65673  /* OMEGA OFFSET */
#define ANGULAR_MOTOR_COEFFICIENT 0.2685 /* ANGULAR_MOTOR_COEFFICIENT */
#define MOTORS_INTERCEPT          4070.3 /* MOTORS_INTERCEPT */
#define T                         0.01  /* TIME STEP */


namespace rotors_control{

PositionController::PositionController()
    : controller_active_(false),
    first_time_hovering_controller_(false),
    first_time_xy_controller_(false),
    first_time_attitude_controller_(false),
    first_time_rate_controller_(false){

}

PositionController::~PositionController() {}

void PositionController::SetOdometry(const EigenOdometry& odometry) {
    odometry_ = odometry;
}

void PositionController::SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
    command_trajectory_= command_trajectory;
    controller_active_= true;
}

void PositionController::CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities) {
    assert(rotor_velocities);
    
    //this serves to inactivate the controller if we don't recieve a trajectory
    if(!controller_active_){
       *rotor_velocities = Eigen::Vector4d::Zero(rotor_velocities->rows());
    return;
    }
    
    double PWM_1, PWM_2, PWM_3, PWM_4;
    ControlMixer(&PWM_1, &PWM_2, &PWM_3, &PWM_4);
 
    double omega_1, omega_2, omega_3, omega_4;
    omega_1 = ((PWM_1 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT); 
    omega_2 = ((PWM_2 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT); 
    omega_3 = ((PWM_3 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT); 
    omega_4 = ((PWM_4 * ANGULAR_MOTOR_COEFFICIENT) + MOTORS_INTERCEPT); 
    
    double omega_1_radians, omega_2_radians, omega_3_radians, omega_4_radians;
    omega_1_radians = omega_1 * M_PI/30;
    omega_2_radians = omega_2 * M_PI/30;
    omega_3_radians = omega_3 * M_PI/30;
    omega_4_radians = omega_4 * M_PI/30;

    if(!(omega_1_radians < 2618 && omega_1_radians > 0))
        if(omega_1_radians > 2618)
           omega_1_radians = 2618;
        else
           omega_1_radians = 0;

    if(!(omega_2_radians < 2618 && omega_2_radians > 0))
        if(omega_2_radians > 2618)
           omega_2_radians = 2618;
        else
           omega_2_radians = 0;

    if(!(omega_3_radians < 2618 && omega_3_radians > 0))
        if(omega_3_radians > 2618)
           omega_3_radians = 2618;
        else
           omega_3_radians = 0;

    if(!(omega_4_radians < 2618 && omega_4_radians > 0))
        if(omega_4_radians > 2618)
           omega_4_radians = 2618;
        else
           omega_4_radians = 0;

    *rotor_velocities = Eigen::Vector4d(omega_1_radians, omega_2_radians, omega_3_radians, omega_4_radians);
}

void PositionController::Quaternion2Euler(double* roll, double* pitch, double* yaw) const {
    assert(roll);
    assert(pitch);
    assert(yaw);
    
    tf::Quaternion q(odometry_.orientation.x(), odometry_.orientation.y(), odometry_.orientation.z(), odometry_.orientation.w());
    tf::Matrix3x3 m(q);
    m.getRPY(*roll, *pitch, *yaw);

    //We need to change the sign because the conversion is along XYZ and not ZYX, on wich the control algorithms has been 
    //designed
    *roll = -1 * *roll;    
  
}

void PositionController::XYController(double* theta_command_, double* phi_command_) {
    assert(theta_command_);
    assert(phi_command_);    

    double v_, u_;
    u_ = odometry_.velocity[0];  
    v_ = odometry_.velocity[1];

    double xe, ye;
    ErrorBodyFrame(&xe, &ye);

    double xe_error_, ye_error_;
    xe_error_ = xe - u_;
    ye_error_ = ye - v_;

    double theta_command_ki, theta_command_kp, phi_command_ki, phi_command_kp;
    if(!first_time_xy_controller_){
       theta_command_ki = 0;
       phi_command_ki = 0;
       first_time_xy_controller_ = true;
    }
    
    theta_command_kp = controller_parameters_.xy_gain_kp_.x() * xe_error_;
    theta_command_ki = theta_command_ki + (controller_parameters_.xy_gain_ki_.x() * xe_error_ * T);
    *theta_command_ = theta_command_kp + theta_command_ki;

   if(!(*theta_command_ < 30 && *theta_command_ > -30))
      if(*theta_command_ > 30)
         *theta_command_ = 30;
      else
         *theta_command_ = -30;

    phi_command_kp = controller_parameters_.xy_gain_kp_.y() * ye_error_;
    phi_command_ki = phi_command_ki + (controller_parameters_.xy_gain_ki_.y() * ye_error_ * T);
    *phi_command_ = phi_command_kp + phi_command_ki;

   if(!(*phi_command_ < 30 && *phi_command_ > -30))
      if(*phi_command_ > 30)
         *phi_command_ = 30;
      else
         *phi_command_ = -30;

}

void PositionController::AttitudeController(double* p_command_, double* q_command_) {
    assert(p_command_);
    assert(q_command_); 

    double roll, pitch, yaw;
    Quaternion2Euler(&roll, &pitch, &yaw);  

    double pitch_degree_, roll_degree_;
    pitch_degree_ = pitch * (180/M_PI); 
    roll_degree_ = roll * (180/M_PI);
 
    double theta_command_, phi_command_;
    XYController(&theta_command_, &phi_command_);

    double phi_error_, theta_error_;
    phi_error_ = phi_command_ - roll_degree_;
    theta_error_ = theta_command_ - pitch_degree_;

    double p_command_ki, p_command_kp, q_command_ki, q_command_kp;
    if(!first_time_attitude_controller_){
       p_command_ki = 0;
       q_command_ki = 0;
       first_time_attitude_controller_ = true;
    }

    p_command_kp = controller_parameters_.attitude_gain_kp_.x() * phi_error_;
    p_command_ki = p_command_ki + (controller_parameters_.attitude_gain_ki_.x() * phi_error_ * T);
    *p_command_ = p_command_kp + p_command_ki;

    q_command_kp = controller_parameters_.attitude_gain_kp_.y() * theta_error_;
    q_command_ki = q_command_ki + (controller_parameters_.attitude_gain_ki_.y() * theta_error_ * T);
    *q_command_ = q_command_kp + q_command_ki;

}

void PositionController::RateController(double* delta_phi, double* delta_theta, double* delta_psi) {
    assert(delta_phi);
    assert(delta_theta);
    assert(delta_psi);
    
    double p_radians, q_radians, r_radians;
    p_radians =  odometry_.angular_velocity[0];
    q_radians =  odometry_.angular_velocity[1];
    r_radians =  odometry_.angular_velocity[2];

    double p_degree, q_degree, r_degree;
    p_degree = p_radians * (180/M_PI);
    q_degree = q_radians * (180/M_PI);
    r_degree = r_radians * (180/M_PI);

    double p_command, q_command;
    AttitudeController(&p_command, &q_command);
   
    double r_command;
    YawPositionController(&r_command);

    double p_error_, q_error_, r_error_;
    p_error_ = p_command - p_degree;
    q_error_ = q_command - q_degree;
    r_error_ = r_command - r_degree;

    double delta_phi_kp, delta_theta_kp, delta_psi_kp, delta_psi_ki;
    if(!first_time_rate_controller_){
       delta_psi_ki = 0;
       first_time_rate_controller_ = true;
    }

    delta_phi_kp = controller_parameters_.rate_gain_kp_.x() * p_error_;
    *delta_phi = delta_phi_kp;

    delta_theta_kp = controller_parameters_.rate_gain_kp_.y() * q_error_;
    *delta_theta = delta_theta_kp;

    delta_psi_kp = controller_parameters_.rate_gain_kp_.z() * r_error_;
    delta_psi_ki = delta_psi_ki + (controller_parameters_.rate_gain_ki_.z() * r_error_ * T);
    *delta_psi = delta_psi_kp + delta_psi_ki;

}

void PositionController::ControlMixer(double* PWM_1, double* PWM_2, double* PWM_3, double* PWM_4) {
    assert(PWM_1);
    assert(PWM_2);
    assert(PWM_3);
    assert(PWM_4);
    
    double omega, delta_omega;
    HoveringController(&delta_omega);
    omega = OMEGA_OFFSET + delta_omega;
   
    double delta_phi, delta_theta, delta_psi;
    RateController(&delta_phi, &delta_theta, &delta_psi);

    *PWM_1 = omega - (delta_theta/2) - (delta_phi/2) - delta_psi;
    *PWM_2 = omega + (delta_theta/2) - (delta_phi/2) + delta_psi;
    *PWM_3 = omega + (delta_theta/2) + (delta_phi/2) - delta_psi;
    *PWM_4 = omega - (delta_theta/2) + (delta_phi/2) + delta_psi;

}

void PositionController::YawPositionController(double* r_command_) const {
    assert(r_command_);

    double yaw, roll, pitch;
    Quaternion2Euler(&roll, &pitch, &yaw);   

    double yaw_error_degree_, yaw_reference_degree_, yaw_degree_;
    yaw_degree_ = yaw * (180/M_PI);
    yaw_reference_degree_ = command_trajectory_.getYaw() * (180/M_PI);
    yaw_error_degree_ = yaw_reference_degree_ - yaw_degree_;

    *r_command_ = controller_parameters_.yaw_gain_kp_ * yaw_error_degree_;

   if(!(*r_command_ < 200 && *r_command_ > -200))
      if(*r_command_ > 200)
         *r_command_ = 200;
      else
         *r_command_ = -200;

}

void PositionController::HoveringController(double* delta_omega_) {
    assert(delta_omega_);

    double z_error_;
    z_error_ = command_trajectory_.position_W[2] - odometry_.position[2];

    double delta_omega_kp, delta_omega_ki, delta_omega_kd;

    if(!first_time_hovering_controller_){
       delta_omega_ki = 0;
       first_time_hovering_controller_ = true;
    }

    delta_omega_kp = controller_parameters_.hovering_gain_kp_ * z_error_;
    delta_omega_ki = delta_omega_ki + (controller_parameters_.hovering_gain_ki_ * z_error_* T);
    delta_omega_kd = controller_parameters_.hovering_gain_kd_ * odometry_.velocity[2];
    *delta_omega_ = delta_omega_kp + delta_omega_ki + delta_omega_kd;

    if(!(*delta_omega_ < 15000 && *delta_omega_ > -20000))
      if(*delta_omega_ > 15000)
         *delta_omega_ = 15000;
      else
         *delta_omega_ = -20000;

}

void PositionController::ErrorBodyFrame(double* xe, double* ye) const {
    assert(xe);
    assert(ye);
    
    double xe_error_, ye_error_;
    xe_error_ = command_trajectory_.position_W[0] - odometry_.position[0];
    ye_error_ = command_trajectory_.position_W[1] - odometry_.position[1];

    double yaw, roll, pitch;
    Quaternion2Euler(&roll, &pitch, &yaw);   
    
    *xe = xe_error_ * cos(yaw) + ye_error_ * sin(yaw);
    *ye = ye_error_ * cos(yaw) - xe_error_ * sin(yaw);

}
}

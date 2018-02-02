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

#ifndef ROTORS_CONTROL_POSITION_CONTROLLER_H
#define ROTORS_CONTROL_POSITION_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "rotors_control/common.h"
#include "rotors_control/parameters.h"


namespace rotors_control {

// Default values for the position controller of the Crazyflie2. XYController [x,y], AttitudeController [phi,theta] 
//RateController [p,q,r], YawController[yaw], HoveringController[z]
static const Eigen::Vector2d kPDefaultXYController = Eigen::Vector2d(15, -15);
static const Eigen::Vector2d kIDefaultXYController = Eigen::Vector2d(1, -1);

static const Eigen::Vector2d kPDefaultAttitudeController = Eigen::Vector2d(3.5, 3.5);
static const Eigen::Vector2d kIDefaultAttitudeController = Eigen::Vector2d(2, 2);

static const Eigen::Vector3d kPDefaultRateController = Eigen::Vector3d(70, 70, 70);
static const Eigen::Vector3d kIDefaultRateController = Eigen::Vector3d(0, 0, 16.7);

static const double kPDefaultYawController = 3;

static const double kPDefaultHoveringController = 14000;
static const double kIDefaultHoveringController = 15000;
static const double kDDefaultHoveringController = -20000;


class PositionControllerParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PositionControllerParameters()
      : xy_gain_kp_(kPDefaultXYController), 
        xy_gain_ki_(kIDefaultXYController), 
        attitude_gain_kp_(kPDefaultAttitudeController), 
        attitude_gain_ki_(kIDefaultAttitudeController), 
        rate_gain_kp_(kPDefaultRateController), 
        rate_gain_ki_(kIDefaultRateController),  
        yaw_gain_kp_(kPDefaultYawController), 
        hovering_gain_kp_(kPDefaultHoveringController), 
        hovering_gain_ki_(kIDefaultHoveringController), 
        hovering_gain_kd_(kDDefaultHoveringController) {
  }

  Eigen::Vector2d xy_gain_kp_;
  Eigen::Vector2d xy_gain_ki_;
  
  Eigen::Vector2d attitude_gain_kp_;
  Eigen::Vector2d attitude_gain_ki_;
  Eigen::Vector2d attitude_gain_kd_;
  
  Eigen::Vector3d rate_gain_kp_;
  Eigen::Vector3d rate_gain_ki_;
  
  double yaw_gain_kp_;
  
  double hovering_gain_kp_;
  double hovering_gain_ki_;
  double hovering_gain_kd_;
};
    
    class PositionController{
        public:
            PositionController();
            ~PositionController();
            void CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities);

            void SetOdometry(const EigenOdometry& odometry);
            void SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory);
            
            PositionControllerParameters controller_parameters_;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        private:
            bool controller_active_;
            bool first_time_hovering_controller_;
            bool first_time_xy_controller_;
            bool first_time_attitude_controller_;
            bool first_time_rate_controller_;

            mav_msgs::EigenTrajectoryPoint command_trajectory_;
            EigenOdometry odometry_;

            void ErrorBodyFrame(double* xe, double* ye) const;
            void HoveringController(double* delta_omega_);
            void YawPositionController(double* r_command) const;
            void XYController(double* theta_command_, double* phi_command_);
            void AttitudeController(double* p_command_, double* q_command_);
            void RateController(double* delta_phi, double* delta_theta, double* delta_psi);
            void ControlMixer(double* PWM_1, double* PWM_2, double* PWM_3, double* PWM_4); 
            void Quaternion2Euler(double* roll, double* pitch, double* yaw) const;

    };

}
#endif // ROTORS_CONTROL_POSITION_CONTROLLER_H

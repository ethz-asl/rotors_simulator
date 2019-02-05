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

#ifndef CRAZYFLIE_2_POSITION_CONTROLLER_H
#define CRAZYFLIE_2_POSITION_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "common.h"
#include "parameters.h"
#include "stabilizer_types.h"
#include "crazyflie_complementary_filter.h"
#include "crazyflie_onboard_controller.h"
#include "sensfusion6.h"
#include "controller_parameters.h"

#include <time.h>

namespace rotors_control {
    
    class PositionController{
        public:
            PositionController();
            ~PositionController();
            void CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities);

            void SetOdometryWithStateEstimator(const EigenOdometry& odometry);
	          void SetOdometryWithoutStateEstimator(const EigenOdometry& odometry);
            void SetSensorData(const sensorData_t& sensors);
            void SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory);
	          void SetControllerGains();
            void CallbackAttitudeEstimation();
            void CallbackHightLevelControl(); 

            PositionControllerParameters controller_parameters_;
            ComplementaryFilterCrazyflie2 complementary_filter_crazyflie_;
            CrazyflieOnboardController crazyflie_onboard_controller_;

        private:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            bool controller_active_;
            bool state_estimator_active_;

            control_s control_t_;

            //Integrator initial conditions
            double theta_command_ki_;
            double phi_command_ki_;
            double p_command_ki_;
            double q_command_ki_;
            double delta_psi_ki_;
            double r_command_ki_;
            double delta_omega_ki_;

            //Controller gains
	          Eigen::Vector2f xy_gain_kp_, xy_gain_ki_;
            Eigen::Vector2f attitude_gain_kp_, attitude_gain_ki_;
            Eigen::Vector3f rate_gain_kp_, rate_gain_ki_;
            double yaw_gain_kp_, yaw_gain_ki_;
            double hovering_gain_kp_, hovering_gain_ki_, hovering_gain_kd_;

            void SetSensorData();

            mav_msgs::EigenTrajectoryPoint command_trajectory_;
            EigenOdometry odometry_;
            sensorData_t sensors_;
            state_t state_;

            void RateController(double* delta_phi, double* delta_theta, double* delta_psi);
            void AttitudeController(double* p_command, double* q_command);
            void ErrorBodyFrame(double* xe, double* ye) const;
            void HoveringController(double* delta_omega);
            void YawPositionController(double* r_command);
            void XYController(double* theta_command, double* phi_command);
            void ControlMixer(double* PWM_1, double* PWM_2, double* PWM_3, double* PWM_4); 
            void Quaternion2Euler(double* roll, double* pitch, double* yaw) const;

    };

}
#endif // CRAZYFLIE_2_POSITION_CONTROLLER_H

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

#ifndef CRAZYFLIE_2_ONBOARD_CONTROLLER_H
#define CRAZYFLIE_2_ONBOARD_CONTROLLER_H

#include "stabilizer_types.h"
#include "controller_parameters.h"

namespace rotors_control {

    class CrazyflieOnboardController{
        public:
            
            CrazyflieOnboardController();
            ~CrazyflieOnboardController();

            void SetControlSignals(const control_s& control_t);
            void SetDroneState(const state_s& state_t);
            void SetControllerGains(PositionControllerParameters& controller_parameters_);
            void RateController(double* delta_phi, double* delta_theta, double* delta_psi);
 
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        private:

            control_s control_t_private_;
            state_s state_t_private_;
 
            // counter for critical section. It allows to synchronize the attitude and rate parts of
            // the crazyflie on board controller.
            bool counter_;          

            //Integrator intial condition
            double delta_psi_ki_;
            double p_command_ki_, q_command_ki_;
            double p_command_, q_command_;

            Eigen::Vector2f attitude_gain_kp_private_, attitude_gain_ki_private_;
            Eigen::Vector3f rate_gain_kp_private_, rate_gain_ki_private_;

            void Quaternion2Euler(double* roll, double* pitch, double* yaw) const;
            void AttitudeController(double* p_command, double* q_command);

     };

}

#endif // CRAZYFLIE_2_ONBOARD_CONTROLLER_H

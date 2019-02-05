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
 *
 * Originally authored by BitCraze (https://github.com/bitcraze/crazyflie-firmware), commit #0ad0181 on 20 Dec 2017
 *
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * estimator_complementary.c - Complementary estimator
 */

#include "rotors_control/crazyflie_complementary_filter.h"
#include "rotors_control/stabilizer_types.h"

namespace rotors_control {

#define ATTITUDE_UPDATE_RATE 250
#define ATTITUDE_UPDATE_DT 1.0/ATTITUDE_UPDATE_RATE

ComplementaryFilterCrazyflie2::ComplementaryFilterCrazyflie2() {}

ComplementaryFilterCrazyflie2::~ComplementaryFilterCrazyflie2() {}

void ComplementaryFilterCrazyflie2::PositionUpdateAngularVelocity(double* angularVelocityZ, double* angularVelocityY,          
           double* angularVelocityX, double* angularAccZ, double* angularAccY, double* angularAccX) {

  *angularVelocityY = *angularAccY;
  *angularVelocityZ = *angularAccZ;
  *angularVelocityX = *angularAccX;
}

void ComplementaryFilterCrazyflie2::EstimateAttitude(state_t* state, sensorData_t* sensorData){

  // Sensors values are read at full rate (500Hz). The IMU topic publishes data with a frequency around 800Hz but such data is passed to the position control
  // algorithm with a frequency of 500Hz. There is a timer that manages data exchange.
  sensors_fusion_.Sensfusion6UpdateQ(&sensorData->gyro.x, &sensorData->gyro.y, &sensorData->gyro.z, &sensorData->acc.x, &sensorData->acc.y, &sensorData->acc.z, (double) ATTITUDE_UPDATE_DT);

  // Saving attitude values, adjusted for the legacy CF2 body coordinate system
  sensors_fusion_.Sensfusion6GetEulerRPY(&state->attitude.roll, &state->attitude.pitch, &state->attitude.yaw);

  // Saving quaternion values, hopefully one day this could be used in a better controller.
  // Note that this is not adjusted for the legacy coordinate system
  sensors_fusion_.Sensfusion6GetQuaternion(
      &state->attitudeQuaternion.x,
      &state->attitudeQuaternion.y,
      &state->attitudeQuaternion.z,
      &state->attitudeQuaternion.w);

   sensors_fusion_.Sensfusion6GetAccZWithoutGravity(&state->angularVelocity.z, &sensorData->acc.x, &sensorData->acc.y, &sensorData->acc.z);

}

void ComplementaryFilterCrazyflie2::EstimateRate(state_t* state, sensorData_t* sensorData){

   PositionUpdateAngularVelocity(&state->angularVelocity.z, &state->angularVelocity.y, &state->angularVelocity.x, &sensorData->gyro.z, &sensorData->gyro.y, &sensorData->gyro.x);

}

}




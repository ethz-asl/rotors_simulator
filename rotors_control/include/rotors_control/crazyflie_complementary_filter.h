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
 * Originally authored by BitCraze (https://github.com/bitcraze/crazyflie-firmware), commit #3df0d39 on 22 Jun 2017
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
 * estimator_complementary.h - Complementary estimator interfaced
 */

#ifndef __ESTIMATOR_COMPLEMENTARY_CRAZYFLIE2_H__
#define __ESTIMATOR_COMPLEMENTARY_CRAZYFLIE2_H__

#include "stabilizer_types.h"
#include "sensfusion6.h"

namespace rotors_control {

	class ComplementaryFilterCrazyflie2 {
	  public:
	    ComplementaryFilterCrazyflie2();
	    ~ComplementaryFilterCrazyflie2();

	    void EstimateRate(state_t* state, sensorData_t* sensorData);
	    void EstimateAttitude(state_t* state, sensorData_t* sensorData);

	  private:

	    SensFusion sensors_fusion_;
	 
	    void PositionUpdateAngularVelocity(double* angularVelocityZ, double* angularVelocityY,          
		   double* angularVelocityX, double* angularAccZ, double* angularAccY, double* angularAccX);
	};

}

#endif //_ESTIMATOR_COMPLEMENTARY_CRAZYFLIE2_H_

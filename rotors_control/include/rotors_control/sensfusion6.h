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
 * Originally authored by BitCraze (https://github.com/bitcraze/crazyflie-firmware), commit #0ad0181 on 22 Jun 2017
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
 * sensfusion6.h - Sensor fusion interface
 */

#ifndef SENSORFUSION6_CRAZYFLIE_H_
#define SENSORFUSION6_CRAZYFLIE_H_

namespace rotors_control {

static double q0 = 1.0f;
static double q1 = 0.0f;
static double q2 = 0.0f;
static double q3 = 0.0f;  // quaternion of sensor frame relative to auxiliary frame [w,x,y,z]

// The acc in Z for static position (g) in m/s^2
// Set on first update, assuming we are in a static position since the sensors were just calibrates.
// This value will be better the more level the copter is at calibration time
static double baseZacc = 9.81;

class SensFusion {
  public:
    SensFusion();
    ~SensFusion();

    void Sensfusion6UpdateQ(double *gx, double *gy, double *gz, double *ax, double *ay, double *az, double dt);
    void Sensfusion6GetQuaternion(double* qx, double* qy, double* qz, double* qw);
    void Sensfusion6GetEulerRPY(double* roll, double* pitch, double* yaw);
    void Sensfusion6GetAccZWithoutGravity(double* angularAccZ, double* ax, double* ay, double* az);

  private:
   
    double q0_ = q0;
    double q1_ = q1;
    double q2_ = q2;
    double q3_ = q3;
    double baseZacc_ = baseZacc;

    // Unit vector in the estimated gravity direction
    double gravX_, gravY_, gravZ_;

    bool isCalibrated_;
   
    double Sensfusion6GetInvThrustCompensationForTilt();
    double InvSqrt(double x);
    void EstimatedGravityDirection(double* gx, double* gy, double* gz);
    void Sensfusion6GetAccZ(double* baseZacc_, double* ax, double* ay, double* az);

  };
}

#endif /* SENSORFUSION6_CRAZYFLIE_H_ */

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
 * Originally authored by BitCraze (https://github.com/bitcraze/crazyflie-firmware), commit #e6a3e26 on 22 Dec 2017
 *
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * stabilizer.h: Stabilizer orchestrator interface
 */

#ifndef __STABILIZER_TYPES_H__
#define __STABILIZER_TYPES_H__

#include <ros/ros.h>

namespace rotors_control {

// Frequencies to bo used with the RATE_DO_EXECUTE_HZ macro. Do NOT use an arbitrary number.
#define RATE_1000_HZ 1000
#define RATE_500_HZ 500
#define RATE_250_HZ 250
#define RATE_100_HZ 100
#define RATE_50_HZ 50
#define RATE_25_HZ 25

#define RATE_MAIN_LOOP RATE_1000_HZ
#define ATTITUDE_RATE RATE_500_HZ
#define POSITION_RATE RATE_100_HZ

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (RATE_MAIN_LOOP / RATE_HZ)) == 0)


/** Attitude in euler angle form */
typedef struct attitude_s {
  double roll; //rad
  double pitch; //rad
  double yaw;  //rad
} attitude_t;

/* x,y,z vector */
struct vec3_s {
  double x;
  double y;
  double z;
};

typedef struct vec3_s point_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;

/* Orientation as a quaternion */
typedef struct quaternion_s {
  union {
    struct {
      double q0;
      double q1;
      double q2;
      double q3;
    };
    struct {
      double x;
      double y;
      double z;
      double w;
    };
  };
} quaternion_t;


typedef union Axis_s {
   struct {
         double x;
         double y;
         double z;
   };
   double axis[3];
 } Axis3f;


typedef struct sensorData_s {
  Axis3f acc;               // m/s^2
  Axis3f gyro;              // rad/s
} sensorData_t;

typedef struct state_s {
  attitude_t attitude;             // rad
  quaternion_t attitudeQuaternion;
  point_t position;                // m
  velocity_t angularVelocity;      // rad/s
  acc_t angularAcc;                // m/s^2
  velocity_t linearVelocity;       //m/s 
} state_t;

typedef struct control_s {
  double roll; //rad
  double pitch; //rad
  double yawRate; //rad/s
  double thrust; //N
} control_t;

}

#endif //__STABILIZER_TYPES_H__

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
 * sensfusion6.cpp - Sensor fusion
 */

#include "rotors_control/sensfusion6.h"
#include "rotors_control/Matrix3x3.h"
#include "rotors_control/Quaternion.h" 
#include "rotors_control/stabilizer_types.h"

#include <math.h>
#include <ros/ros.h>

#define GRAVITY_MAGNITUDE 9.81 // m/s^2

//#define MADWICK_QUATERNION_IMU

#ifdef MADWICK_QUATERNION_IMU
  #define BETA_DEF     0.01    // 2 * proportional gain
#else // MAHONY_QUATERNION_IMU
    #define TWO_KP_DEF  (2.0 * 0.4) // 2 * proportional gain
    #define TWO_KI_DEF  (2.0 * 0.001) // 2 * integral gain
#endif

#ifdef MADWICK_QUATERNION_IMU
  double beta = BETA_DEF;     // 2 * proportional gain (Kp)
#else // MAHONY_QUATERNION_IMU
  double twoKp = TWO_KP_DEF;    // 2 * proportional gain (Kp)
  double twoKi = TWO_KI_DEF;    // 2 * integral gain (Ki)
  double integralFBx = 0.0f;
  double integralFBy = 0.0f;
  double integralFBz = 0.0f;  // integral error terms scaled by Ki
#endif

namespace rotors_control{

// The acc in Z for static position (g) in m/s^2
// Set on first update, assuming we are in a static position since the sensors were just calibrates.
// This value will be better the more level the copter is at calibration time
SensFusion::SensFusion()
   : isCalibrated_(false),
    q0_(1),
    q1_(0),
    q2_(0),
    q3_(0),
    gravX_(0),
    gravY_(0),
    gravZ_(0),
    baseZacc_(9.81){

}

SensFusion::~SensFusion() {}


#ifdef MADWICK_QUATERNION_IMU
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-ahrs-with-x-imu
//
// Date     Author          Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick    Optimized for reduced CPU load
void SensFusion::Sensfusion6UpdateQ(double* gx, double* gy, double* gz, double* ax, double* ay, double* az, double dt){
  assert(gx);
  assert(gy);
  assert(gz);
  assert(ax);
  assert(ay);
  assert(az);
  assert(dt);

  double recipNorm;
  double s0, s1, s2, s3;
  double qDot1, qDot2, qDot3, qDot4;
  double _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  double gx_, gy_, gz_, ax_, ay_, az_, dt_;
  //The code has been developed taking into account that the gyroscope values are expressed in deg/s
  gx_ = *gx * (180/M_PI);
  gy_ = *gy * (180/M_PI);
  gz_ = *gz * (180/M_PI);
  //The code has been developed taking into account that the accelerometer values are express in G
  ax_ = *ax/GRAVITY_MAGNITUDE; 
  ay_ = *ay/GRAVITY_MAGNITUDE;
  az_ = *az/GRAVITY_MAGNITUDE;
  dt_ = dt;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5 * (-q1_ * gx_ - q2_ * gy_ - q3_ * gz_);
  qDot2 = 0.5 * (q0_ * gx_ + q2_ * gz_ - q3_ * gy_);
  qDot3 = 0.5 * (q0_ * gy_ - q1_ * gz_ + q3_ * gx_);
  qDot4 = 0.5 * (q0_ * gz_ + q1_ * gy_ - q2_ * gx_);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalization)
  if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))){

    // Normalize accelerometer measurement
    recipNorm = InvSqrt(ax_ * ax_ + ay_ * ay_ + az_ * az_);
    ax_ *= recipNorm;
    ay_ *= recipNorm;
    az_ *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0 * q0_;
    _2q1 = 2.0 * q1_;
    _2q2 = 2.0 * q2_;
    _2q3 = 2.0 * q3_;
    _4q0 = 4.0 * q0_;
    _4q1 = 4.0 * q1_;
    _4q2 = 4.0 * q2_;
    _8q1 = 8.0 * q1_;
    _8q2 = 8.0 * q2_;
    q0q0 = q0_ * q0_;
    q1q1 = q1_ * q1_;
    q2q2 = q2_ * q2_;
    q3q3 = q3_ * q3_;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax_ + _4q0 * q1q1 - _2q1 * ay_;
    s1 = _4q1 * q3q3 - _2q3 * ax_ + 4.0 * q0q0 * q1_ - _2q0 * ay_ - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az_;
    s2 = 4.0 * q0q0 * q2_ + _2q0 * ax_ + _4q2 * q3q3 - _2q3 * ay_ - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az_;
    s3 = 4.0 * q1q1 * q3_ - _2q1 * ax_ + 4.0 * q2q2 * q3_ - _2q2 * ay_;
    recipNorm = InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalize step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0_ += qDot1 * dt_;
  q1_ += qDot2 * dt_;
  q2_ += qDot3 * dt_;
  q3_ += qDot4 * dt_;

  // Normalize quaternion
  recipNorm = invSqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
  q0_ *= recipNorm;
  q1_ *= recipNorm;
  q2_ *= recipNorm;
  q3_ *= recipNorm;

  EstimatedGravityDirection(&gravX_, &gravY_, &gravZ_);

  if (!isCalibrated_) {
    Sensfusion6GetAccZ(&baseZacc_, ax, ay, az);
    isCalibrated_ = true;
  }

}

#else // MAHONY_QUATERNION_IMU
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-ahrs-with-x-imu
//
// Date     Author      Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick    Optimized for reduced CPU load
void SensFusion::Sensfusion6UpdateQ(double* gx, double* gy, double* gz, double* ax, double* ay, double* az, double dt){
  assert(gx);
  assert(gy);
  assert(gz);
  assert(ax);
  assert(ay);
  assert(az);
  assert(dt);
  
  double recipNorm;
  double halfvx, halfvy, halfvz;
  double halfex, halfey, halfez;
  double qa, qb, qc;

  double gx_, gy_, gz_, ax_, ay_, az_, dt_;
  gx_ = *gx;
  gy_ = *gy;
  gz_ = *gz;
  //The code has been developed taking into account that the accelerometer values are express in G
  ax_ = *ax/GRAVITY_MAGNITUDE; 
  ay_ = *ay/GRAVITY_MAGNITUDE;
  az_ = *az/GRAVITY_MAGNITUDE;
  dt_ = dt;

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalization)
  if(!((ax_ == 0.0) && (ay_ == 0.0) && (az_ == 0.0))){

    // Normalise accelerometer measurement
    recipNorm = InvSqrt(ax_ * ax_ + ay_ * ay_ + az_ * az_);
    ax_ *= recipNorm;
    ay_ *= recipNorm;
    az_ *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1_ * q3_ - q0_ * q2_;
    halfvy = q0_ * q1_ + q2_ * q3_;
    halfvz = q0_ * q0_ - 0.5 + q3_ * q3_;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay_ * halfvz - az_ * halfvy);
    halfey = (az_ * halfvx - ax_ * halfvz);
    halfez = (ax_ * halfvy - ay_ * halfvx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0){

      integralFBx += twoKi * halfex * dt_;  // integral error scaled by Ki
      integralFBy += twoKi * halfey * dt_;
      integralFBz += twoKi * halfez * dt_;
      gx_ += integralFBx;  // apply integral feedback
      gy_ += integralFBy;
      gz_ += integralFBz;
    }
    else{

      integralFBx = 0.0; // prevent integral windup
      integralFBy = 0.0;
      integralFBz = 0.0;
    }

    // Apply proportional feedback
    gx_ += twoKp * halfex;
    gy_ += twoKp * halfey;
    gz_ += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx_ *= (0.5 * dt_);   // pre-multiply common factors
  gy_ *= (0.5 * dt_);
  gz_ *= (0.5 * dt_);
  qa = q0_;
  qb = q1_;
  qc = q2_;
  q0_ += (-qb * gx_ - qc * gy_ - q3_ * gz_);
  q1_ += (qa * gx_ + qc * gz_ - q3_ * gy_);
  q2_ += (qa * gy_ - qb * gz_ + q3_ * gx_);
  q3_ += (qa * gz_ + qb * gy_ - qc * gx_);

  // Normalize quaternion
  recipNorm = InvSqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
  q0_ *= recipNorm;
  q1_ *= recipNorm;
  q2_ *= recipNorm;
  q3_ *= recipNorm;

  EstimatedGravityDirection(&gravX_, &gravY_, &gravZ_);

  if (!isCalibrated_) {
    Sensfusion6GetAccZ(&baseZacc_, ax, ay, az);
    isCalibrated_ = true;
  }
}
#endif

void SensFusion::Sensfusion6GetQuaternion(double* qx, double* qy, double* qz, double* qw){
  assert(qx);
  assert(qy);
  assert(qz);
  assert(qw);

  *qx = q1_;
  *qy = q2_;
  *qz = q3_;
  *qw = q0_;
}

void SensFusion::Sensfusion6GetEulerRPY(double* roll, double* pitch, double* yaw){
  assert(roll);
  assert(pitch);
  assert(yaw);
    
  tf::Quaternion q(q1_, q2_, q3_, q0_);
  tf::Matrix3x3 m(q);
  m.getRPY(*roll, *pitch, *yaw);

}

void SensFusion::Sensfusion6GetAccZWithoutGravity(double* angularAccZ, double* ax, double* ay, double* az){
  assert(angularAccZ);
  assert(ax);
  assert(ay);
  assert(az);
  
  // Return unbiased (baseZacc) accelerations
  double baseZacc;
  Sensfusion6GetAccZ(&baseZacc, ax, ay, az);
  *angularAccZ = baseZacc - baseZacc_;
}

double SensFusion::Sensfusion6GetInvThrustCompensationForTilt(){

  // Return the z component of the estimated gravity direction
  // (0, 0, 1) dot G
  return gravZ_ * GRAVITY_MAGNITUDE;
}


void SensFusion::Sensfusion6GetAccZ(double* baseZacc, double* ax, double* ay, double* az){

  // return vertical acceleration
  // (A dot G) / |G|,  (|G| = 1) -> (A dot G)
  *baseZacc = (*ax * gravX_)/GRAVITY_MAGNITUDE + (*ay * gravY_)/GRAVITY_MAGNITUDE + (*az * gravZ_)/GRAVITY_MAGNITUDE;
}

void SensFusion::EstimatedGravityDirection(double* gx, double* gy, double* gz){

  *gx = 2 * (q1_ * q3_ - q0_ * q2_);
  *gy = 2 * (q0_ * q1_ + q2_ * q3_);
  *gz = q0_ * q0_ - q1_ * q1_ - q2_ * q2_ + q3_ * q3_;
}

// Fast inverse square-root 1/sqrt(norm)
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
double SensFusion::InvSqrt(double x){

  double y;
  y = 1/sqrt(x * x);
  return y;

}

}

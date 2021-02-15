/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#ifndef ROTORS_GAZEBO_PLUGINS_MOTOR_MODEL_H
#define ROTORS_GAZEBO_PLUGINS_MOTOR_MODEL_H

#include <Eigen/Eigen>

namespace spin {
const static int CCW = 1;
const static int CW = -1;
}  // namespace spin

namespace gazebo {
static constexpr double kDefaultMotorConstant = 8.55e-06;
// Less confusing naming, keep previous for compatibility
static constexpr double kDefaultThrustConstant = 8.55e-06;
static constexpr double kDefaultMomentConstant = 0.016;
static constexpr double kDefaultTimeConstantUp = 1.0 / 80.0;
static constexpr double kDefaultTimeConstantDown = 1.0 / 40.0;
static constexpr double kDefaultMaxRotVelocity = 838.0;
static constexpr double kDefaultMinRotVelocity = 100.0;
static constexpr double kDefaultMaxTorque = 10.0;
static constexpr double kDefaultMaxForce = std::numeric_limits<double>::max();
static constexpr double kDefaultMaxRotPosition = -M_PI;
static constexpr double kDefaultMinRotPosition = M_PI;
static constexpr double kDefaultPositionOffset = 0.0;
static constexpr double kDefaultRotorDragCoefficient = 1.0e-4;
static constexpr double kDefaultRollingMomentCoefficient = 1.0e-6;
static constexpr double kDefaultPGain = 100;
static constexpr double kDefaultIGain = 0;
static constexpr double kDefaultDGain = 50;
}  // namespace gazebo

class MotorModel {
 public:
  MotorModel()
      : motor_rot_pos_(0.0),
        motor_rot_vel_(0.0),
        motor_rot_effort_(0.0),
        ref_motor_rot_pos_(0.0),
        ref_motor_rot_vel_(0.0),
        ref_motor_rot_effort_(0.0),
        prev_sim_time_(0.0),
        sampling_time_(0.01) {}

  virtual ~MotorModel() {}

  void GetMotorVelocity(double &result) const { result = motor_rot_vel_; }

  void SetReferenceMotorVelocity(double ref_motor_rot_vel) {
    ref_motor_rot_vel_ = ref_motor_rot_vel;
  }

  void UpdatePhysics() { UpdateForcesAndMoments(); }

  void GetActuatorState(double *position, double *velocity, double *effort) {
    *position = motor_rot_pos_;
    *velocity = motor_rot_vel_;
    *effort = motor_rot_effort_;
  }

  void SetActuatorReference(double ref_position, double ref_velocity,
                            double ref_effort) {
    ref_motor_rot_pos_ = ref_position;
    ref_motor_rot_vel_ = ref_velocity;
    ref_motor_rot_effort_ = ref_effort;
  }

  virtual void InitializeParams() = 0;

  virtual void Publish() = 0;

 protected:
  double motor_rot_pos_;
  double motor_rot_vel_;
  double motor_rot_effort_;
  double ref_motor_rot_pos_;
  double ref_motor_rot_vel_;
  double ref_motor_rot_effort_;
  double prev_ref_motor_rot_vel_;
  double prev_sim_time_;
  double sampling_time_;

  virtual void UpdateForcesAndMoments() = 0;
};

#endif  // ROTORS_GAZEBO_PLUGINS_MOTOR_MODEL_H

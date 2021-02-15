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

#ifndef ROTORS_GAZEBO_PLUGINS_MOTOR_MODEL_SERVO_H
#define ROTORS_GAZEBO_PLUGINS_MOTOR_MODEL_SERVO_H

// 3RD PARTY
#include <Eigen/Core>
#include <boost/bind.hpp>
#include <gazebo/physics/physics.hh>

// USER
#include "rotors_gazebo_plugins/common.h"
#include "rotors_gazebo_plugins/motor_model.hpp"

enum class ControlMode { kVelocity, kPosition, kEffort };

namespace gazebo {

class MotorModelServo : public MotorModel {
 public:
  MotorModelServo(const physics::ModelPtr _model, const sdf::ElementPtr _motor)
      : MotorModel(),
        mode_(ControlMode::kPosition),
        turning_direction_(spin::CCW),
        max_rot_velocity_(kDefaultMaxRotVelocity),
        max_torque_(kDefaultMaxTorque),
        max_rot_position_(kDefaultMaxRotPosition),
        min_rot_position_(kDefaultMinRotPosition),
        position_zero_offset_(kDefaultPositionOffset) {
    joint_controller_ = _model->GetJointController();
    motor_ = _motor;
    joint_ = _model->GetJoint(motor_->GetElement("jointName")->Get<std::string>());
    InitializeParams();
  }

  virtual ~MotorModelServo() {}

 protected:
  // Parameters
  ControlMode mode_;
  std::string joint_name_;
  int turning_direction_;
  double max_rot_velocity_;
  double max_torque_;
  double max_rot_position_;
  double min_rot_position_;
  double position_zero_offset_;

  // Gazebo PID implementation:
  // https://github.com/arpg/Gazebo/blob/master/gazebo/common/PID.cc
  common::PID pid_;

  physics::JointControllerPtr joint_controller_;
  sdf::ElementPtr motor_;
  physics::JointPtr joint_;

  void InitializeParams() {
    // Check motor type.
    if (motor_->HasElement("controlMode")) {
      std::string motor_type = motor_->GetElement("controlMode")->Get<std::string>();
      if (motor_type == "velocity")
        mode_ = ControlMode::kVelocity;
      else if (motor_type == "position")
        mode_ = ControlMode::kPosition;
      else if (motor_type == "effort") {
        mode_ = ControlMode::kEffort;
      } else
        gzwarn << "[motor_model_servo] controlMode not valid, using position.\n";
    } else {
      gzwarn << "[motor_model_servo] controlMode not specified, using position.\n";
    }

    // Check spin direction.
    if (motor_->HasElement("spinDirection")) {
      std::string turning_direction = motor_->GetElement("spinDirection")->Get<std::string>();
      if (turning_direction == "cw")
        turning_direction_ = spin::CW;
      else if (turning_direction == "ccw")
        turning_direction = spin::CCW;
      else
        gzerr << "[motor_model_servo] Spin not valid, using 'ccw.'\n";
    } else {
      gzwarn << "[motor_model_servo] spinDirection not specified, using ccw.\n";
    }
    getSdfParam<double>(motor_, "maxRotVelocity", max_rot_velocity_, max_rot_velocity_);
    getSdfParam<double>(motor_, "maxRotPosition", max_rot_position_, max_rot_position_);
    getSdfParam<double>(motor_, "minRotPosition", min_rot_position_, min_rot_position_);
    getSdfParam<double>(motor_, "zeroOffset", position_zero_offset_, position_zero_offset_);

    // Set up joint control PID to control joint.
    if (motor_->HasElement("joint_control_pid")) {
      double p, i, d, iMax, iMin, cmdMax, cmdMin;
      sdf::ElementPtr pid = motor_->GetElement("joint_control_pid");
      getSdfParam<double>(pid, "p", p, 0.0);
      getSdfParam<double>(pid, "i", i, 0.0);
      getSdfParam<double>(pid, "d", d, 0.0);
      getSdfParam<double>(pid, "iMax", iMax, 0.0);
      getSdfParam<double>(pid, "iMin", iMin, 0.0);
      getSdfParam<double>(pid, "cmdMax", cmdMax, 0.0);
      getSdfParam<double>(pid, "cmdMin", cmdMin, 0.0);

      // Setup a pid controller
      pid_ = common::PID(p, i, d);
      // pid_ = common::PID(p, i, d, iMax, iMin, cmdMax, cmdMin);

      switch (mode_) {
        case (ControlMode::kPosition): {
          joint_controller_->SetPositionPID(joint_->GetScopedName(), pid_);
          break;
        }
        case (ControlMode::kVelocity): {
          joint_controller_->SetVelocityPID(joint_->GetScopedName(), pid_);
          break;
        }
        default: {
          gzwarn << "[motor_model_servo] Position PID values found, but not used in effort mode!\n";
        }
      }
    } else {
      gzerr << "[motor_model_servo] Position PID values not found, default all "
               "values to zero!\n";
    }
  }

  void Publish() {}  // No publishing here

  double NormalizeAngle(double input) {
    // Constrain magnitude to be max 2*M_PI.
    double wrapped = std::fmod(std::abs(input), 2 * M_PI);
    wrapped = std::copysign(wrapped, input);

    // Constrain result to be element of [0, 2*pi).
    // Set angle to zero if sufficiently close to 2*pi.
    if (std::abs(wrapped - 2 * M_PI) < 1e-8) {
      wrapped = 0;
    }

    // Ensure angle is positive.
    if (wrapped < 0) {
      wrapped += 2 * M_PI;
    }

    return wrapped;
  }

  void UpdateForcesAndMoments() {
    motor_rot_pos_ = turning_direction_ * joint_->Position(0);
    motor_rot_vel_ = turning_direction_ * joint_->GetVelocity(0);
    motor_rot_effort_ = turning_direction_ * joint_->GetForce(0);

    switch (mode_) {
      case (ControlMode::kPosition): {
        joint_controller_->SetPositionTarget(joint_->GetScopedName(),
                                             turning_direction_ * ref_motor_rot_pos_);
        joint_controller_->Update();
        break;
      }
      case (ControlMode::kVelocity): {
        joint_controller_->SetVelocityTarget(joint_->GetScopedName(),
                                             turning_direction_ * ref_motor_rot_vel_);
        joint_controller_->Update();
        break;
      }
      case (ControlMode::kEffort): {
        // TODO(@kajabo): make motor torque feedback w gain.
        double ref_torque = std::copysign(ref_motor_rot_effort_,
                                          std::min(std::abs(ref_motor_rot_effort_), max_torque_));
        joint_->SetForce(0, turning_direction_ * ref_torque);
        break;
      }
      default: {}
    }
  }
};

}  // namespace gazebo

#endif  // ROTORS_GAZEBO_PLUGINS_MOTOR_MODEL_SERVO_H

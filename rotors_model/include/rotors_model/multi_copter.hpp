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


#ifndef ROTORS_MODEL_MULTI_COPTER_H
#define ROTORS_MODEL_MULTI_COPTER_H

#include <Eigen/Eigen>

#include "rotors_model/motor_controller.hpp"
#include "rotors_model/motor_model.hpp"

class MultiCopter
{
  public:
    MultiCopter(int amount_rotors) :
      position_(Eigen::Vector3d::Zero()),
      velocity_(Eigen::Vector3d::Zero()),
      attitude_(Eigen::Quaterniond::Identity()),
      rotor_rot_vels_(Eigen::VectorXd::Zero(amount_rotors))
    {}
    MultiCopter(int amount_rotors,
      MotorController& motor_controller) :
      MultiCopter(amount_rotors)
    {
      setMotorController(motor_controller);
    }
    ~MultiCopter();

    void setMotorController(
      MotorController& motor_controller) {
      motor_controller_ = motor_controller;
    }

    Eigen::VectorXd getRefMotorVelocities(double dt) {
      return motor_controller_.getMotorVelocities(dt);
    }

    Eigen::VectorXd getMotorVelocities() {
      return rotor_rot_vels_;
    }

    virtual Eigen::Vector4d simulateMAV(double dt,
      Eigen::VectorXd ref_rotor_rot_vels) = 0;
    virtual void initializeParams() = 0;
    virtual void publish() = 0;

    const MotorController motorController() {
      return motor_controller_;
    }
    const Eigen::Vector3d position() {return position_;}
    const Eigen::Vector3d velocity() {return velocity_;}
    const Eigen::Quaterniond attitude() {return attitude_;}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  protected:
    MotorController motor_controller_;
    Eigen::Vector3d position_;
    Eigen::Vector3d velocity_;
    Eigen::Quaterniond attitude_;
    Eigen::Vector3d angular_rate_;
    Eigen::VectorXd rotor_rot_vels_;
};

#endif // ROTORS_MODEL_MULTI_COPTER_H

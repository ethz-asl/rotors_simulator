/*
 * Copyright (C) 2014 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Pascal Gohl, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Sammy Omari, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * This software is released to the Contestants of the european 
 * robotics challenges (EuRoC) for the use in stage 1. (Re)-distribution, whether 
 * in parts or entirely, is NOT PERMITTED. 
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 */


#ifndef MAV_MODEL_MULTI_COPTER_H
#define MAV_MODEL_MULTI_COPTER_H
#include <Eigen/Eigen>
#include <mav_model/motor_controller.hpp>
#include <mav_model/motor_model.hpp>


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

#endif // MAV_MODEL_MULTI_COPTER_H

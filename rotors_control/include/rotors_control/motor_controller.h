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

#ifndef ROTORS_CONTROL_MOTOR_CONTROLLER_H
#define ROTORS_CONTROL_MOTOR_CONTROLLER_H

#include "rotors_control/controller_base.h"
#include "rotors_control/controller_factory.h"

class MotorController : public ControllerBase {
 public:
  MotorController();
  virtual ~MotorController();
  virtual void InitializeParams();
  virtual std::shared_ptr<ControllerBase> Clone();
  virtual void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  Eigen::Matrix4Xd allocation_matrix_;
  Eigen::MatrixX4d angular_acc_to_rotor_velocities_;
  Eigen::Vector3d gain_attitude_;
  Eigen::Vector3d gain_angular_rate_;
  Eigen::Matrix3d inertia_matrix_;
};

#endif // ROTORS_CONTROL_MOTOR_CONTROLLER_H

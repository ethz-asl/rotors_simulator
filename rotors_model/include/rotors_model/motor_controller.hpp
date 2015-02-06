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


#ifndef ROTORS_MODEL_MOTOR_CONTROLLER_H
#define ROTORS_MODEL_MOTOR_CONTROLLER_H

#include <Eigen/Eigen>

class MotorController
{
  public:
    MotorController(int amount_motors) :
      ref_rotor_rot_vels_(Eigen::VectorXd::Zero(amount_motors)){};
    virtual ~MotorController();
    void getMotorVelocities() {
      calculateRefMotorVelocities();
      return ref_rotor_rot_vels_;
    }

    virtual void calculateRefMotorVelocities() = 0;
    virtual void initializeParams() = 0;
    virtual void publish() = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  protected:
    // imu_
    // odom_

    Eigen::Vector3d position_;
    Eigen::Vector3d velocity_;
    Eigen::Quaternion attitude_;
    Eigen::VectorXd ref_rotor_rot_vels_;
};

#endif // ROTORS_MODEL_MOTOR_CONTROLLER_H

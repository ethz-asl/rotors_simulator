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


#ifndef MAV_MODEL_MOTOR_CONTROLLER_H
#define MAV_MODEL_MOTOR_CONTROLLER_H
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

#endif // MAV_MODEL_MOTOR_CONTROLLER_H

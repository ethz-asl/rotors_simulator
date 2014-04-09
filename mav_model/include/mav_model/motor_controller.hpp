//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>
// All rights reserved.
//
// TODO(ff): Enter some license
//==============================================================================

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

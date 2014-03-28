//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>
// All rights reserved.
//
// TODO(ff): Enter some license
//==============================================================================

#ifndef MAV_MODEL_MOTOR_MODEL_H
#define MAV_MODEL_MOTOR_MODEL_H
#include <Eigen/Eigen>

class MotorModel
{
  public:
    MotorModel() : 
      motor_rot_vel_(0),
      ref_motor_rot_vel_(0) {}
    ~MotorModel();
    double getMotorVelocity(double dt,
      double ref_motor_rot_vel) {
      dt_ = dt;
      ref_motor_rot_vel_ = ref_motor_rot_vel;
      calculateMotorVelocity();
      return motor_rot_vel_;
    }

    virtual void initializeParams() = 0;
    virtual void publish() = 0;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  protected:
    double dt_;
    double motor_rot_vel_;
    double ref_motor_rot_vel_;

    virtual void calculateMotorVelocity() = 0;
};

#endif // MAV_MODEL_MOTOR_MODEL_H

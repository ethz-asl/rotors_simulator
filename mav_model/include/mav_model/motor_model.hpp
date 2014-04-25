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
    virtual ~MotorModel() {}
    void GetMotorVelocity(double &result) const {
      result = motor_rot_vel_;
    }
    void SetReferenceMotorVelocity(double ref_motor_rot_vel) {
      ref_motor_rot_vel_ = ref_motor_rot_vel;
    }

    virtual void InitializeParams() = 0;
    virtual void Publish() = 0;

  protected:
    double motor_rot_vel_;
    double ref_motor_rot_vel_;

    virtual void UpdateForcesAndMoments() = 0;
};

#endif // MAV_MODEL_MOTOR_MODEL_H

//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>
// All rights reserved.
//
// TODO(ff): Enter some license
//==============================================================================

#ifndef MAV_MODEL_MOTOR_CONTROLLER_H
#define MAV_MODEL_MOTOR_CONTROLLER_H
#include <Eigen/Eigen>

template<class InputT, class ParamsT>
class MotorController
{
  public:
    MotorController(int amount_motors) : 
      ref_rotor_rot_vels_(Eigen::VectorXd::Zero(amount_motors)){};
    virtual ~MotorController();
    Eigen::VectorXd getMotorVelocities(InputT input_reference) {
      input_reference_ = input_reference;
      calculateRefMotorVelocities();
      return ref_rotor_rot_vels_;
    }
    
    virtual void calculateRefMotorVelocities() = 0;
    virtual void initializeParams() = 0;
    virtual void publish() = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  protected:
    InputT input_reference_;
    // imu_
    // odom_
    Eigen::VectorXd ref_rotor_rot_vels_;
    ParamsT params;
};

#endif // MAV_MODEL_MOTOR_CONTROLLER_H

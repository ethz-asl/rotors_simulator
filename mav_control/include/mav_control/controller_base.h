//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>, Michael Burri <....>
// All rights reserved.
//
// TODO(ff): Enter some license
//==============================================================================

#ifndef MAV_CONTROL_CONTROLLER_BASE_H
#define MAV_CONTROL_CONTROLLER_BASE_H
#include <Eigen/Eigen>
#include <memory>
#include <assert.h>

class ControllerBase {
  public:
    ControllerBase();
    virtual ~ControllerBase();

    virtual void InitializeParams() = 0;
    virtual std::shared_ptr<ControllerBase> Clone() = 0;
    void SetPosition(Eigen::Vector3d position);
    void SetVelocity(Eigen::Vector3d velocity);
    void SetAttitude(Eigen::Quaternion<double> attitude);
    void SetAngularRate(Eigen::Vector3d angular_rate);
    void SetAttitudeThrustReference(
      Eigen::Vector4d control_attitude_thrust_reference);
    virtual void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const = 0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  protected:
    Eigen::Vector3d position_;
    Eigen::Vector3d velocity_;
    Eigen::Quaternion<double> attitude_;
    Eigen::Vector3d angular_rate_;
    Eigen::Vector4d control_attitude_thrust_reference_;
    int amount_rotors_;
    bool initialized_params_;
};


#endif // MAV_CONTROL_CONTROLLER_BASE_H

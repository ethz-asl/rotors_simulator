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

class ControllerBase {
  public:
    // ControllerBase();
    // ~ControllerBase();
    Eigen::VectorXd GetMotorVelocities();
    virtual void InitializeParams();
    virtual void Publish();
    virtual std::shared_ptr<ControllerBase> New();
    void SetPosition(Eigen::Vector3d position);
    void SetVelocity(Eigen::Vector3d velocity);
    void SetAttitude(Eigen::Quaternion<double> attitude);
    void SetAngularRate(Eigen::Vector3d angular_rate);
    void SetAttitudeThrustReference(
      Eigen::Vector4d control_attitude_thrust_reference);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  protected:
    virtual void UpdateStates();
    virtual void CalculateRefMotorVelocities();
    Eigen::Vector3d position_;
    Eigen::Vector3d velocity_;
    Eigen::Quaternion<double> attitude_;
    Eigen::Vector3d angular_rate_;
    Eigen::VectorXd ref_rotor_rot_vels_;
    // Eigen::VectorXd control_input_;
    Eigen::Vector4d control_attitude_thrust_reference_;
    int amount_rotors_;
};


#endif // MAV_CONTROL_CONTROLLER_BASE_H

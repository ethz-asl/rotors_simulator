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

#ifndef MAV_CONTROL_CONTROLLER_BASE_H
#define MAV_CONTROL_CONTROLLER_BASE_H

#include <assert.h>
#include <memory>

#include <Eigen/Eigen>

template <class Derived>
class ControllerParameterLoaderBase{

};

class VehicleParameters{
 public:
  static Eigen::Matrix3d GetDefaultInertia() const;
  static double GetDefaultMass() const;

  bool parameters_initialized_;
  Eigen::Matrix3d inertia_; // I think we need the inverse the most, so we could also hide the parameters private and do the inversion when setting the parameters
  double mass_;
  double arm_length_;
};

class ControllerBase {
 public:
  ControllerBase();
  virtual ~ControllerBase();

  virtual void SetPosition(const Eigen::Vector3d& position);
  virtual void SetVelocity(const Eigen::Vector3d& velocity);
  virtual void SetAttitude(const Eigen::Quaternion<double>& attitude);
  virtual void SetAngularRate(const Eigen::Vector3d& angular_rate);

  virtual void SetReferenceAttitudeThrust(const Eigen::Vector4d& control_attitude_thrust_reference);
  virtual void SetReferenceRateThrust(const Eigen::Vector4d& control_rate_thrust_reference);
  virtual void SetReferencePosition(const Eigen::Vector3d& position_reference);
  virtual void SetReferenceVelocity(const Eigen::Vector3d& velocity_reference);
  virtual void SetReferenceAcceleration(const Eigen::Vector3d& acceleration_reference);
  virtual void SetReferenceJerk(const Eigen::Vector3d& jerk_reference);
  virtual void SetReferenceYaw(double yaw_reference);
  virtual void SetReferenceYawRate(double yaw_rate_reference);
  virtual void SetReferenceMotor(const Eigen::VectorXd& motor_reference);

  virtual void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const = 0;
  virtual const char* GetName() const = 0;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  Eigen::Vector3d position_;
  Eigen::Vector3d velocity_;
  Eigen::Quaterniond attitude_;
  Eigen::Vector3d angular_rate_;

  Eigen::Vector3d position_reference_;
  Eigen::Vector3d velocity_reference_;
  Eigen::Vector3d acceleration_reference_;
  Eigen::Vector3d jerk_reference_;
  double yaw_reference_;
  double yaw_rate_reference_;

  Eigen::Vector4d control_attitude_thrust_reference_;
  Eigen::Vector4d control_rate_thrust_reference_;
  Eigen::VectorXd motor_reference_;
  int amount_rotors_;
  bool initialized_params_;
};

#endif // MAV_CONTROL_CONTROLLER_BASE_H

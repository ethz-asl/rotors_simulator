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
#include <Eigen/Eigen>
#include <memory>
#include <assert.h>

class ControllerBase {
 public:
  ControllerBase();
  virtual ~ControllerBase();

  virtual void InitializeParams() = 0;
  virtual std::shared_ptr<ControllerBase> Clone() = 0;
  void SetPosition(const Eigen::Vector3d& position);
  void SetVelocity(const Eigen::Vector3d& velocity);
  void SetAttitude(const Eigen::Quaternion<double>& attitude);
  void SetAngularRate(const Eigen::Vector3d& angular_rate);
  void SetAttitudeThrustReference(const Eigen::Vector4d& control_attitude_thrust_reference);
  void SetRateThrustReference(const Eigen::Vector4d& control_rate_thrust_reference);
  void SetPositionReference(const Eigen::Vector3d& position_reference);
  void SetVelocityReference(const Eigen::Vector3d& velocity_reference);
  void SetAccelerationReference(const Eigen::Vector3d& acceleration_reference);
  void SetJerkReference(const Eigen::Vector3d& jerk_reference);
  void SetYawReference(double yaw_reference);
  void SetYawRateReference(double yaw_rate_reference);

  void SetMotorReference(const Eigen::VectorXd& motor_reference);
  virtual void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const = 0;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  Eigen::Vector3d position_;
  Eigen::Vector3d velocity_;
  Eigen::Quaternion<double> attitude_;
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

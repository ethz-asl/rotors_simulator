#include <mav_control/controller_base.h>
#include <math.h>
#define _USE_MATH_DEFINES

ControllerBase::ControllerBase() {
  initialized_params_ = false;
}

ControllerBase::~ControllerBase() { }

void ControllerBase::SetPosition(const Eigen::Vector3d& position) {
  position_ = position;
}

void ControllerBase::SetVelocity(const Eigen::Vector3d& velocity) {
  velocity_ = velocity;
}

void ControllerBase::SetAttitude(const Eigen::Quaternion<double>& attitude) {
  attitude_ = attitude;
}

void ControllerBase::SetAngularRate(const Eigen::Vector3d& angular_rate) {
  angular_rate_ = angular_rate;
}

void ControllerBase::SetAttitudeThrustReference(
  const Eigen::Vector4d& control_attitude_thrust_reference) {
  control_attitude_thrust_reference_ = control_attitude_thrust_reference;
}

void ControllerBase::SetMotorReference(const Eigen::VectorXd& motor_reference) {
  motor_reference_ = motor_reference;
}

void ControllerBase::SetPositionReference(const Eigen::Vector3d& position_reference) {
  position_reference_ = position_reference;
}

void ControllerBase::SetVelocityReference(const Eigen::Vector3d& velocity_reference) {
  velocity_reference_ = velocity_reference;
}

void ControllerBase::SetAccelerationReference(const Eigen::Vector3d& acceleration_reference) {
  acceleration_reference_ = acceleration_reference;
}

void ControllerBase::SetJerkReference(const Eigen::Vector3d& jerk_reference) {
  jerk_reference_ = jerk_reference;
}

void ControllerBase::SetYawReference(double yaw_reference) {
  yaw_reference_ = yaw_reference;
}

void ControllerBase::SetYawRateReference(double yaw_rate_reference) {
  yaw_rate_reference_ = yaw_rate_reference;
}

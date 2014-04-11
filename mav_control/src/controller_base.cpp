#include <mav_control/controller_base.h>

ControllerBase::ControllerBase() {
  initialized_params_ = false;
}

ControllerBase::~ControllerBase() { }

void ControllerBase::SetPosition(Eigen::Vector3d position) {
  position_ = position;
}

void ControllerBase::SetVelocity(Eigen::Vector3d velocity) {
  velocity_ = velocity;
}

void ControllerBase::SetAttitude(Eigen::Quaternion<double> attitude) {
  attitude_ = attitude;
}

void ControllerBase::SetAngularRate(Eigen::Vector3d angular_rate) {
  angular_rate_ = angular_rate;
}

void ControllerBase::SetAttitudeThrustReference(
  Eigen::Vector4d control_attitude_thrust_reference) {
  control_attitude_thrust_reference_ = control_attitude_thrust_reference;
}

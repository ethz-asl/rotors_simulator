#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "rotors_control/common.h"
#include <ros/ros.h>

namespace rotors_control{
class PIDAttitudeController{
 public:

  PIDAttitudeController();
  ~PIDAttitudeController();
  void InitializeParams();

  void SetDesiredAttitude(double desired_roll, double desired_pitch, double desired_yaw_rate, double desired_thrust){
    attitude_thrust_reference_ <<  desired_roll, desired_pitch, desired_yaw_rate, desired_thrust;
  }

void SetOdometry(const EigenOdometry& odometry);

void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:

  bool initialized_params_;




  //Parameters for attitude controller from SysID

  Eigen::MatrixX4d angular_acc_to_rotor_velocities_;
  Eigen::Matrix3d inertia_;
  double yaw_rate_gain_;

  double roll_gain_;
  double pitch_gain_;

  double p_gain_;
  double q_gain_;
  double r_gain_;

  Eigen::Vector4d attitude_thrust_reference_;
  EigenOdometry odometry_ ;



  double mass_;
  const double gravity_;


  

  void ComputeDesiredAngularAcc(Eigen::Vector3d* angular_acc) const;
  void quat2rpy(Eigen::Quaternion<double> q, Eigen::Vector3d* rpy) const;

};
}

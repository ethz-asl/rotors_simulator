/*
 * KFDisturbanceObserver.h
 *
 *  Created on: Jan 27, 2015
 *      Author: Mina Kamel, ASL - ETH Zurich
 */

#ifndef KFDisturbanceObserver_H_
#define KFDisturbanceObserver_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <rotors_control/calibrateKF.h>
//#include "rotors_control/common.h"

namespace rotors_control{
class KFDisturbanceObserver
{
 public:

  KFDisturbanceObserver();
  void Initialize();
  void Reset(Eigen::Vector3d initial_position,
             Eigen::Vector3d initial_velocity,
             Eigen::Vector3d initial_attitude,
             Eigen::Vector3d initial_angular_rate,
             Eigen::Vector3d initial_external_forces,
             Eigen::Vector3d initial_external_moments);


  //Getters
  Eigen::Vector3d GetEstimatedPosition(){if(initialized_) return state_.segment(0, 3); else return Eigen::Vector3d::Zero();};
  Eigen::Vector3d GetEstimatedVelocity(){if(initialized_) return state_.segment(3, 3); else return Eigen::Vector3d::Zero();};
  Eigen::Vector3d GetEstimatedAttitude(){if(initialized_) return state_.segment(6, 3); else return Eigen::Vector3d::Zero();};
  Eigen::Vector3d GetEstimatedAngularVelocity(){if(initialized_) return state_.segment(9, 3); else return Eigen::Vector3d::Zero();};

  Eigen::Vector3d GetEstimatedExternalForces(){
    if(initialized_ == true && calibrate_ == false)
      return state_.segment(12, 3) - forces_offset_;
    else
      return Eigen::Vector3d::Zero();
  };
  Eigen::Vector3d GetEstimatedExternalMoments(){
    if(initialized_ && calibrate_ == false)
      return state_.segment(15, 3) - moments_offset_;
    else
      return Eigen::Vector3d::Zero();
  };

  void GetEstimatedState(Eigen::VectorXd* estimated_state) const;


  //Feeding
  void FeedPositionMeasurement(const  Eigen::Vector3d position );
  void FeedVelocityMeasurement(  Eigen::Vector3d velocity );
  void FeedRotationMatrix(const Eigen::Matrix3d &rotation_matrix );
  void FeedAttitudeCommand( const Eigen::Vector4d &roll_pitch_yaw_thrust_cmd);
 // void FeedOdometryMsg( EigenOdometry odometry);


  void Calibrate(ros::WallDuration calibration_time); //TODO(mina)
  void UpdateEstimator();



  virtual ~KFDisturbanceObserver();



 private:
  bool initialized_;
  Eigen::Matrix<double, 18, 1> state_; //[position, velocity, attitude , angular_velocity,  external_forces, external_moments, ]
  Eigen::Matrix<double, 18, 1> predicted_state_;
  Eigen::Matrix<double, 9, 1> measurements_; //[position, velocity, attitude]
  Eigen::Matrix3d rotation_matrix_;
  Eigen::Vector4d roll_pitch_yaw_thrust_cmd_;
  Eigen::Matrix<double, 18, 18> process_noise_covariance_;
  Eigen::Matrix<double, 18, 18> state_covariance_;
  Eigen::Matrix<double, 9, 9> measurement_covariance_;
  Eigen::Matrix3d drag_coefficients_matrix_;

  Eigen::Matrix<double, 18, 18> F_; //dynamic matrix
  Eigen::Matrix<double, 18, 9> K_;  //Kalman gain
  Eigen::Matrix<double, 9, 18> H_;  //measurement matrix

  Eigen::Vector3d external_forces_limit_;
  Eigen::Vector3d external_moments_limit_;
  Eigen::Vector3d omega_limit_;




  // Parameters
  double roll_damping_;
  double roll_omega_;
  double roll_gain_;

  double pitch_damping_;
  double pitch_omega_;
  double pitch_gain_;

  double yaw_damping_;
  double yaw_omega_;
  double yaw_gain_;

  double gravity_;

  ros::ServiceServer service_;
  ros::NodeHandle* nh_;



 bool calibrate_;         // true if calibrating
 ros::Time start_calibration_time_;   // t0 calibration
 ros::Duration calibration_time_;     // calibration duration
 Eigen::Vector3d forces_offset_;
 Eigen::Vector3d moments_offset_;
 int calibration_counter_;




  void SystemDynamics(double dt);
  bool StartCalibrationCallback(calibrateKF::Request &req, calibrateKF::Response &res);




};
}
#endif /* SRC_KFDisturbanceObserver_H_ */

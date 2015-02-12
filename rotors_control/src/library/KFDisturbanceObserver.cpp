/*
 * KFDisturbanceObserver.cpp
 *
 *  Created on: Jan 27, 2015
 *      Author: mina
 */

#include <rotors_control/KFDisturbanceObserver.h>
#include <iostream>


KFDisturbanceObserver::KFDisturbanceObserver(): gravity_(9.81),
                                                initialized_(false){

}


bool KFDisturbanceObserver::test(rotors_control::InitKF::Request &req, rotors_control::InitKF::Response &res){
	printf("Service InitKF is called\n");
	res.result = "OK!";
	return true;
}

void KFDisturbanceObserver::Initialize(){

  printf("Start initializing KF\n");

  nh_ = new ros::NodeHandle("KF");
  service_ = nh_->advertiseService("init_KF", &KFDisturbanceObserver::test, this);

  std::vector<double> temporary_F, temporary_Q, temporary_P, temporary_R, temporary_drag;
  std::vector<double> temporary_external_forces_limit, temporary_external_moments_limit, temporary_omega_limit;

   //parameters namespace
   std::string MAV_param_prefix = "MPC_position_controller/MAV_parameters/";
   std::string Observer_prefix = "MPC_position_controller/observer/";


   if(!ros::param::get(MAV_param_prefix + "drag_coefficients",  temporary_drag))
        ROS_ERROR("Drag Coefficients in KF are not loaded from ros parameter server");

   if(!ros::param::get(MAV_param_prefix + "roll_omega",  roll_omega_))
        ROS_ERROR("roll_omega in KF is not loaded from ros parameter server");
   if(!ros::param::get(MAV_param_prefix + "roll_damping",  roll_damping_))
        ROS_ERROR("roll_damping in KF is not loaded from ros parameter server");
   if(!ros::param::get(MAV_param_prefix + "roll_gain",  roll_gain_))
        ROS_ERROR("roll_gain in KF is not loaded from ros parameter server");

   if(!ros::param::get(MAV_param_prefix + "pitch_omega",  pitch_omega_))
        ROS_ERROR("pitch_omega in KF is not loaded from ros parameter server");
   if(!ros::param::get(MAV_param_prefix + "pitch_damping",  pitch_damping_))
        ROS_ERROR("pitch_damping in KF is not loaded from ros parameter server");
   if(!ros::param::get(MAV_param_prefix + "pitch_gain",  pitch_gain_))
        ROS_ERROR("pitch_gain in KF is not loaded from ros parameter server");

   if(!ros::param::get(MAV_param_prefix + "yaw_omega",  yaw_omega_))
        ROS_ERROR("yaw_omega in KF is not loaded from ros parameter server");
   if(!ros::param::get(MAV_param_prefix + "yaw_damping",  yaw_damping_))
        ROS_ERROR("yaw_damping in KF is not loaded from ros parameter server");
   if(!ros::param::get(MAV_param_prefix + "yaw_gain",  yaw_gain_))
        ROS_ERROR("yaw_gain in KF is not loaded from ros parameter server");


   if(!ros::param::get(Observer_prefix + "F",  temporary_F))
      ROS_ERROR("F in KF is not loaded from ros parameter server");
   if(!ros::param::get(Observer_prefix + "Q",  temporary_Q))
     ROS_ERROR("Q in KF is not loaded from ros parameter server");
   if(!ros::param::get(Observer_prefix  + "P_0",  temporary_P))
     ROS_ERROR("P_0 in KF is not loaded from ros parameter server");
   if(!ros::param::get(Observer_prefix  + "R",  temporary_R))
     ROS_ERROR("R in KF is not loaded from ros parameter server");

   if(!ros::param::get(Observer_prefix  + "external_forces_limit",  temporary_external_forces_limit))
        ROS_ERROR("external_forces_limit in KF is not loaded from ros parameter server");
   if(!ros::param::get(Observer_prefix  + "external_moments_limit",  temporary_external_moments_limit))
        ROS_ERROR("external_moments_limit in KF is not loaded from ros parameter server");
   if(!ros::param::get(Observer_prefix  + "omega_limit",  temporary_omega_limit))
        ROS_ERROR("omega_limit in KF is not loaded from ros parameter server");


   printf("Param loaded :KF\n");

   printf("KF CHKPNT\n");
   Eigen::Map<Eigen::MatrixXd> P_map(temporary_P.data(),18,18);
   Eigen::Map<Eigen::MatrixXd> Q_map(temporary_Q.data(),18,18);
   Eigen::Map<Eigen::MatrixXd> F_map(temporary_F.data(),18,18);
   Eigen::Map<Eigen::MatrixXd> R_map(temporary_R.data(),9,9);

   Eigen::Map<Eigen::Vector3d> external_forces_limit_map(temporary_external_forces_limit.data(), 3,1);
   Eigen::Map<Eigen::Vector3d> external_moments_limit_map(temporary_external_moments_limit.data(), 3,1);
   Eigen::Map<Eigen::Vector3d> omega_limit_map(temporary_omega_limit.data(), 3,1);

   external_forces_limit_ = external_forces_limit_map;
   external_moments_limit_ = external_moments_limit_map;
   omega_limit_ = omega_limit_map;

   state_covariance_ = P_map;

   process_noise_covariance_ = Q_map;

   measurement_covariance_ = R_map;

   F_ = F_map;



   drag_coefficients_matrix_ = Eigen::Matrix3d::Identity(3,3) ;
   drag_coefficients_matrix_(0,0) = temporary_drag.at(0);
   drag_coefficients_matrix_(1,1) = temporary_drag.at(1);
   drag_coefficients_matrix_(2,2) = temporary_drag.at(2);

   state_.setZero();
   predicted_state_.setZero();

   forces_offset_.setZero();
   moments_offset_.setZero();

   H_ = Eigen::Matrix<double,9,18>::Identity();

   time_last_update_ = ros::Time::now();

   initialized_ = true;

   ROS_INFO("Kalman Filter Initialized!");

}

void KFDisturbanceObserver::Calibrate(ros::WallDuration calibration_time){

}

void KFDisturbanceObserver::FeedPositionMeasurement(Eigen::Vector3d position){
  this->measurements_(0) = position(0);
  this->measurements_(1) = position(1);
  this->measurements_(2) = position(2);
}


void KFDisturbanceObserver::FeedVelocityMeasurement(Eigen::Vector3d velocity){
  this->measurements_(3) = velocity(0);
  this->measurements_(4) = velocity(1);
  this->measurements_(5) = velocity(2);
}

void KFDisturbanceObserver::FeedRotationMatrix( Eigen::Matrix3d &rotation_matrix ){
  this->rotation_matrix_ = rotation_matrix;
  this->measurements_(6) = atan2((double)rotation_matrix(2,1), (double)rotation_matrix(2,2));
  this->measurements_(7) = -asin((double)rotation_matrix(2,0));
  this->measurements_(8) = atan2((double)rotation_matrix(1,0), (double)rotation_matrix(0,0));
}

void KFDisturbanceObserver::FeedAttitudeCommand(Eigen::Vector4d rpyt_cmd){
  this->roll_pitch_yaw_thrust_cmd_ = rpyt_cmd;
}

void KFDisturbanceObserver::Reset( Eigen::Vector3d initial_position,
                                   Eigen::Vector3d initial_velocity,
                                   Eigen::Vector3d initial_attitude,
                                   Eigen::Vector3d initial_angular_rate,
                                   Eigen::Vector3d initial_external_forces,
                                   Eigen::Vector3d initial_external_moments){

  printf("KF reset 1\n");
  std::string Observer_prefix = "MPC_position_controller/observer/";

  //get P_0
  std::vector<double> temporary_P;

  if(!ros::param::get(Observer_prefix  + "P_0",  temporary_P))
       ROS_ERROR("P_0 in KF is not loaded from ros parameter server");


  Eigen::Map<Eigen::MatrixXd> P_map(temporary_P.data(), 18, 18);
  state_covariance_ = P_map;



  state_.setZero();

  state_.segment(0,3) = initial_position;
  state_.segment(3,3) = initial_velocity;
  state_.segment(6,3) = initial_attitude;
  state_.segment(9,3) = initial_angular_rate;
  state_.segment(12,3) = initial_external_forces;
  state_.segment(15,3) = initial_external_moments;


  time_last_update_ = ros::Time::now();

}

void KFDisturbanceObserver::UpdateEstimator(ros::Time time){

  if(initialized_ == false) return;

  double dt = (time - time_last_update_).toSec();

  if(dt>0.015){
      dt = 0.015;
    }

    if(dt<0.005){
      dt = 0.005;
    }


  state_covariance_ = F_*state_covariance_*F_.transpose() + process_noise_covariance_;

  //predict state
  SystemDynamics(dt);
  std::cout << "state = \n" << state_ << std::endl;

  Eigen::MatrixXd tmp = H_*state_covariance_*H_.transpose() + measurement_covariance_;
  K_ = state_covariance_*H_.transpose()*tmp.inverse();

  //Update with measurements
  state_ = predicted_state_ + K_*(measurements_ - H_*state_);

  //Update covariance
  state_covariance_ = (Eigen::MatrixXd::Identity(18, 18) - K_*H_)*state_covariance_;


  //Limits on estimated_disturbances
  if(state_.allFinite() == false){
	  throw std::out_of_range("The estimated state in KF Disturbance Observer has a not finite element");
  }

  Eigen::Vector3d omega = state_.segment(9,3);
  Eigen::Vector3d external_forces = state_.segment(12,3);
  Eigen::Vector3d external_moments= state_.segment(15,3);


  omega = omega.cwiseMax(-omega_limit_);
  omega = omega.cwiseMin(omega_limit_);

  external_forces = external_forces.cwiseMax(-external_forces_limit_);
  external_forces = external_forces.cwiseMin(external_forces_limit_);

  external_moments = external_moments.cwiseMax(-external_moments_limit_);
  external_moments = external_moments.cwiseMin(external_moments_limit_);



  state_.segment(9,9) << omega, external_forces, external_moments ;


  time_last_update_ = time;


}


void KFDisturbanceObserver::SystemDynamics(double dt){
 // printf("dt in observer = %f\n", dt);
  Eigen::Vector3d old_position = state_.segment(0,3);
  Eigen::Vector3d old_velocity = state_.segment(3,3);
  Eigen::Vector3d old_attitude = state_.segment(6,3);
  Eigen::Vector3d old_omega = state_.segment(9,3);
  Eigen::Vector3d old_external_forces = state_.segment(12,3);
  Eigen::Vector3d old_external_moments= state_.segment(15,3);

  Eigen::Vector3d Thrust(0.0, 0.0, this->roll_pitch_yaw_thrust_cmd_(3));

  Eigen::Vector3d acceleration = rotation_matrix_*Thrust + Eigen::Vector3d(0,0,-gravity_) + this->drag_coefficients_matrix_*old_velocity + old_external_forces;

  Eigen::Vector3d new_velocity = old_velocity + acceleration*dt;

  Eigen::Vector3d new_position = old_position + old_velocity*dt + 0.5*acceleration*dt*dt;

  Eigen::Vector3d angular_acceleration;
  angular_acceleration(0) = -2.0*roll_damping_*roll_omega_*old_omega(0) - roll_omega_*roll_omega_*old_attitude(0) + roll_gain_*roll_omega_*roll_omega_*roll_pitch_yaw_thrust_cmd_(0) + old_external_moments(0);
  angular_acceleration(1) = -2.0*pitch_damping_*pitch_omega_*old_omega(1) - pitch_omega_*pitch_omega_*old_attitude(1) + pitch_gain_*pitch_omega_*pitch_omega_*roll_pitch_yaw_thrust_cmd_(1) + old_external_moments(1);
  angular_acceleration(2) = -2.0*yaw_damping_*yaw_omega_*old_omega(2) - yaw_omega_*yaw_omega_*old_attitude(2) + yaw_gain_*yaw_omega_*yaw_omega_*roll_pitch_yaw_thrust_cmd_(2) + old_external_moments(2);

  Eigen::Vector3d new_omega = old_omega + angular_acceleration*dt;

  Eigen::Vector3d new_attitude = old_attitude + old_omega*dt + 0.5*angular_acceleration*dt*dt;

  Eigen::Vector3d new_external_forces = old_external_forces;
  Eigen::Vector3d new_external_moments = old_external_moments;

  //Update the state vector


  predicted_state_.segment(0,3) = new_position;
  predicted_state_.segment(3,3) = new_velocity;
  predicted_state_.segment(6,3) = new_attitude;
  predicted_state_.segment(9,3) = new_omega;
  predicted_state_.segment(12,3) = new_external_forces;
  predicted_state_.segment(15,3) = new_external_moments;

}

void KFDisturbanceObserver::GetEstimatedState(Eigen::VectorXd* estimated_state) const{
    assert(estimated_state);
    assert(initialized_);

    estimated_state->resize(18);
    *estimated_state = this->state_;

  }


KFDisturbanceObserver::~KFDisturbanceObserver() {
  // TODO Auto-generated destructor stub
}


#ifndef MPC_POSITION_CONTROLLER_H
#define MPC_POSITION_CONTROLLER_H

#include "rotors_control/KFDisturbanceObserver.h"
#include "rotors_control/SteadyStateCalculation.h"

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "rotors_control/common.h"


#include <stdio.h>
#include <memory.h>
#include <ros/ros.h>
#include <deque>
#include <Eigen/Eigen>
#include <iostream>
#include <sys/time.h>
#include <ctime>


#ifdef UseForcesSolver
  #include "rotors_control/FireFlyOffsetFreeMPC.h"
#endif

#ifdef UseCVXGENSolver
  #include "rotors_control/solver.h"
#endif


#ifdef UseCVXGENSolver
    //CVXGEN Global variables
    Vars vars;
    Params params;
    Workspace work;
    Settings settings;
#endif

namespace rotors_control{

class MPCPositionController{
 public:

  MPCPositionController();
  ~MPCPositionController();
  void InitializeParams();

  void SetOdometry(const EigenOdometry& odometry);
  void SetCommandTrajectory(const mav_msgs::EigenCommandTrajectory& command_trajectory);

  void CalculateAttitudeThrust(Eigen::Vector4d* ref_attitude_thrust) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:

  double mass_;
  const double gravity_;

  int state_size_;
  int input_size_;
  int disturbance_size_;
  int measurement_size_;


  double yaw_gain_;

  bool initialized_params_;

  bool use_KF_state_;
  bool offset_free_controller_;


  /* MPC Matrices */
    //Model: A, B, Bd
    Eigen::MatrixXd model_A_;   //dynamic matrix
    Eigen::MatrixXd model_B__;  //transfer matrix
    Eigen::MatrixXd model_Bd_;  //Disturbance transfer matrix


    //Cost matrices
#ifdef UseForcesSolver
    Eigen::MatrixXd cost_Hessian_;
    Eigen::MatrixXd cost_Hessian_final_;

    //FORCES parameters struct
    FireFlyOffsetFreeMPC_params FORCES_params_;


#endif

#ifdef UseCVXGENSolver
    //CVXGEN variables


#endif



  mav_msgs::EigenCommandTrajectory command_trajectory_;
  EigenOdometry odometry_;


  std::unique_ptr<KFDisturbanceObserver> StateObserver;
  std::unique_ptr<SteadyStateCalculation> SteadyStateCalculator;

  


  void quat2rpy(Eigen::Quaternion<double> q, Eigen::Vector3d* rpy) const;

};
}

#endif

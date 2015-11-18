/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Simone Comari, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROTORS_CONTROL_MULTI_OBJECTIVE_CONTROLLER_H
#define ROTORS_CONTROL_MULTI_OBJECTIVE_CONTROLLER_H

#include <ros/ros.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <manipulator_msgs/eigen_manipulator_msgs.h>
#include <manipulator_msgs/conversions.h>

#include "rotors_control/common.h"
#include "rotors_control/parameters.h"

#include <Eigen/Dense>
#include <igl/slice.h>
#include <ooqp_eigen_interface/OoqpEigenInterface.hpp>

#include "rotors_control/dynamic_terms_fun/q34_12_fun/q34_12_fun.h"
#include "rotors_control/dynamic_terms_fun/M_triu_fun/M_triu_fun.h"
#include "rotors_control/dynamic_terms_fun/G_fun/G_fun.h"
#include "rotors_control/dynamic_terms_fun/C_fun/C_fun.h"
#include "rotors_control/dynamic_terms_fun/J_e_fun/J_e_fun.h"
#include "rotors_control/dynamic_terms_fun/J_e_dot_fun/J_e_dot_fun.h"


typedef Eigen::SparseMatrix<double, Eigen::RowMajor> SpMatrixXd;


namespace rotors_control {

// Default values for the multi objective controller and the Asctec Neo+Delta manipulator.
static const Eigen::Vector3d kDefaultMavPositionGain = Eigen::Vector3d(400., 400., 150.);
static const Eigen::Vector3d kDefaultMavVelocityGain = Eigen::Vector3d(600., 600., 150.);
static const Eigen::Vector3d kDefaultMavAttitudeGain = Eigen::Vector3d(40., 40., 40.);
static const Eigen::Vector3d kDefaultMavAngularRateGain = Eigen::Vector3d(10., 10., 10.);
static const Eigen::Vector3d kDefaultEePositionGain = Eigen::Vector3d(200., 200., 200.);
static const Eigen::Vector3d kDefaultEeVelocityGain = Eigen::Vector3d(300., 300., 300.);
static const Eigen::Vector3d kDefaultJointAngleGain = Eigen::Vector3d(30., 30., 30.);
static const Eigen::Vector3d kDefaultJointAngRateGain = Eigen::Vector3d(10., 10., 10.);
static const Eigen::Vector3d kDefaultRPYMax = Eigen::Vector3d(M_PI/2, M_PI/2., 2*M_PI);
static const Eigen::Vector3d kDefaultRPYMin = Eigen::Vector3d(-M_PI/2, -M_PI/2., -2*M_PI);
static const Eigen::Vector3d kDefaultArmJointsAngleMax = Eigen::Vector3d(0., M_PI, 0.5556*M_PI);
static const Eigen::Vector3d kDefaultArmJointsAngleMin = Eigen::Vector3d(-M_PI/2, 0.4444*M_PI, 0.);
static const Eigen::Vector2d kDefaultThrustLimits = Eigen::Vector2d(0., 54.);
static const Eigen::VectorXd kDefaultObjectivesWeight = Eigen::VectorXd::Zero(6);
static const unsigned int kDefaultMavDof = 6;
static const unsigned int kDefaultArmDof = 3;
static const unsigned int kDefaultMuAttidute = 1000;
static const unsigned int kDefaultMuArm = 2000;
static const double kDefaultSafeRange = 0.9;




class MultiObjectiveControllerParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MultiObjectiveControllerParameters()
      : mav_position_gain_(kDefaultMavPositionGain),
        mav_velocity_gain_(kDefaultMavVelocityGain),
        mav_attitude_gain_(kDefaultMavAttitudeGain),
        mav_angular_rate_gain_(kDefaultMavAngularRateGain),
        ee_position_gain_(kDefaultEePositionGain),
        ee_velocity_gain_(kDefaultEeVelocityGain),
        arm_joints_angle_gain_(kDefaultJointAngleGain),
        arm_joints_ang_rate_gain_(kDefaultJointAngRateGain),
        rpy_max_(kDefaultRPYMax),
        rpy_min_(kDefaultRPYMin),
        arm_joints_angle_max_(kDefaultArmJointsAngleMax),
        arm_joints_angle_min_(kDefaultArmJointsAngleMin),
        thrust_limits_(kDefaultThrustLimits),
        objectives_weight_(kDefaultObjectivesWeight),
        mu_attitude_(kDefaultMuAttidute),
        mu_arm_(kDefaultMuArm),
        safe_range_rpy_(kDefaultSafeRange),
        safe_range_joints_(kDefaultSafeRange) {
    calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
  }

  Eigen::Matrix4Xd allocation_matrix_;
  Eigen::Vector3d mav_position_gain_;
  Eigen::Vector3d mav_velocity_gain_;
  Eigen::Vector3d mav_attitude_gain_;
  Eigen::Vector3d mav_angular_rate_gain_;
  Eigen::Vector3d ee_position_gain_;
  Eigen::Vector3d ee_velocity_gain_;
  Eigen::Vector3d arm_joints_angle_gain_;
  Eigen::Vector3d arm_joints_ang_rate_gain_;
  Eigen::Vector3d rpy_max_;
  Eigen::Vector3d rpy_min_;
  Eigen::Vector3d arm_joints_angle_max_;
  Eigen::Vector3d arm_joints_angle_min_;
  Eigen::Vector2d thrust_limits_;
  Eigen::VectorXd objectives_weight_;
  int mu_attitude_;
  int mu_arm_;
  double safe_range_rpy_;
  double safe_range_joints_;
  RotorConfiguration rotor_configuration_;
};


struct DynamicModelTerms {
  DynamicModelTerms() {};

  DynamicModelTerms(const Eigen::MatrixXd& _inertia_matrix,
                    const Eigen::MatrixXd& _coriolis_matrix,
                    const Eigen::MatrixXd& _damping_matrix,
                    const Eigen::VectorXd& _gravity_vector) {
    inertia_matrix = _inertia_matrix;
    coriolis_matrix = _coriolis_matrix;
    damping_matrix = _damping_matrix;
    gravity_vector = _gravity_vector;
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::MatrixXd inertia_matrix;
  Eigen::MatrixXd coriolis_matrix;
  Eigen::MatrixXd damping_matrix;
  Eigen::VectorXd gravity_vector;

  inline void setAll(const Eigen::MatrixXd& _inertia_matrix,
                     const Eigen::MatrixXd& _coriolis_matrix,
                     const Eigen::MatrixXd& _damping_matrix,
                     const Eigen::VectorXd& _gravity_vector) {
    inertia_matrix = _inertia_matrix;
    coriolis_matrix = _coriolis_matrix;
    damping_matrix = _damping_matrix;
    gravity_vector = _gravity_vector;

    //debug
//    std::cout << "inertia_matrix\n" << inertia_matrix<< std::endl;
//    std::cout << "coriolis_matrix\n" << coriolis_matrix<< std::endl;
//    std::cout << "damping_matrix\n" << damping_matrix<< std::endl;
//    std::cout << "gravity_vector\n" << gravity_vector<< std::endl;
  }
};


struct EigenEndEffector {
  EigenEndEffector() {};

  EigenEndEffector(const EigenOdometry& _odometry,
                   const Eigen::MatrixXd& _jacobian,
                   const Eigen::MatrixXd& _jacobian_dot) {
    odometry = _odometry;
    jacobian_W = _jacobian;
    jacobian_dot_W = _jacobian_dot;
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EigenOdometry odometry;
  Eigen::MatrixXd jacobian_W;
  Eigen::MatrixXd jacobian_dot_W;

  inline void setPosJac(const Eigen::Vector3d& _position,
                        const Eigen::MatrixXd& _jacobian,
                        const Eigen::MatrixXd& _jacobian_dot) {
    odometry.position = _position;
    jacobian_W = _jacobian;
    jacobian_dot_W = _jacobian_dot;

    //debug
    std::cout << "odometry.position\n" << odometry.position << std::endl;
    std::cout << "jacobian_W\n" << jacobian_W << std::endl;
    std::cout << "jacobian_dot_W\n" << jacobian_dot_W << std::endl;
  }
};


class MultiObjectiveController {

 public:
  MultiObjectiveController();
  ~MultiObjectiveController();
  void InitializeParameters();

  // Called after state update
  void CalculateControlInputs(Eigen::VectorXd* rotor_velocities, Eigen::Vector3d* torques);

  // Called on state update callback (synchronized)
  void SetOdometry(const EigenOdometry& odometry);
  void SetArmJointsState(const manipulator_msgs::EigenJointsState& joints_state);

  // Called asynchronously and independently (when new command is set)
  void SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory);
  void SetEndEffTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory);
  void SetDesiredJointsAngle(const manipulator_msgs::EigenJointTrajectoryPoint& joints_state);

  void SetObjectiveFunctionsWeight(const Eigen::VectorXd& objectives_weight);

  void SetExternalForces(const Eigen::Vector3d& forces);

  MultiObjectiveControllerParameters controller_parameters_;
  VehicleParameters vehicle_parameters_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  bool initialized_params_;
  bool controller_active_;
  bool mav_trajectory_received_;
  bool arm_trajectory_received_;
  bool ee_trajectory_received_;

  unsigned int robot_dof_;
  unsigned int mav_dof_;
  unsigned int arm_dof_;
  unsigned int minimizer_sz_;

  Eigen::MatrixX4d torque_to_rotor_velocities_;

  // Solve min 1/2 x' Q x + c' x, such that A x = b, d <= Cx <= f, and l <= x <= u.
  SpMatrixXd Q_;
  Eigen::VectorXd c_;
  SpMatrixXd A_;
  Eigen::VectorXd b_;
  Eigen::VectorXd d_;
  SpMatrixXd C_;
  Eigen::VectorXd f_;
  Eigen::VectorXd l_;
  Eigen::VectorXd u_;
  Eigen::VectorXd x_;

  mav_msgs::EigenTrajectoryPoint command_trajectory_;
  mav_msgs::EigenTrajectoryPoint command_trajectory_ee_;
  Eigen::VectorXd joints_angle_des_;

  EigenOdometry odometry_;
  EigenEndEffector end_effector_;
  manipulator_msgs::EigenJointsState joints_state_;

  Eigen::Vector3d ext_forces_;

  DynamicModelTerms dyn_mdl_terms_;

  void SolveMultiObjectiveOptimization() ;

  void GetSetPointObjective(const Eigen::VectorXd& g, const Eigen::VectorXd& g_ref,
                            const Eigen::VectorXd& kp, const Eigen::VectorXd& kv,
                            const Eigen::MatrixXd& Jg, const Eigen::MatrixXd& Jg_dot,
                            const Eigen::VectorXd& q_dot, Eigen::MatrixXd* Q,
                            Eigen::VectorXd* c) const;

  void GetAttitudeSetPtObjective(Eigen::MatrixXd* _Q, Eigen::VectorXd* _c) const;
  void GetYawSetPtObjective(Eigen::MatrixXd* _Q, Eigen::VectorXd* _c) const;
  void GetFreeHoverSetPtObjective(Eigen::MatrixXd* _Q, Eigen::VectorXd* _c) const;
  void GetPositionSetPtObjective(const Eigen::Vector3d& _position_ref,
                                 Eigen::MatrixXd* _Q, Eigen::VectorXd* _c) const;
  void GetOrientationSetPtObjective(const Eigen::Vector3d& _orientation_ref,
                                    Eigen::MatrixXd* _Q, Eigen::VectorXd* _c) const;
  void GetManipulatorSetPtObjective(Eigen::MatrixXd* _Q, Eigen::VectorXd* _c) const;
  void GetDeadArmSetPtObjective(Eigen::MatrixXd* _Q, Eigen::VectorXd* _c) const;
  void GetEndEffectorSetPtObjective(Eigen::MatrixXd* _Q, Eigen::VectorXd* _c);

  Eigen::VectorXd GetRobotVelocities() const;

  void UpdateLinearConstraints();

  void UpdateEndEffectorState();

  void UpdateDynamicModelTerms();

};

}

#endif // ROTORS_CONTROL_MULTI_OBJECTIVE_CONTROLLER_H

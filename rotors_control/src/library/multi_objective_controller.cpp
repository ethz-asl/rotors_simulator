/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#include "rotors_control/multi_objective_controller.h"

namespace rotors_control {

MultiObjectiveController::MultiObjectiveController()
    : initialized_params_(false),
      controller_active_(false),
      mav_dof_(kDefaultMavDof),
      arm_dof_(kDefaultArmDof) {
  InitializeParameters();

//  q34_12_fun_initialize();
//  M_triu_fun_initialize();
//  G_fun_initialize();
//  C_fun_initialize();
//  J_e_fun_initialize();
//  J_e_dot_fun_initialize();
}


MultiObjectiveController::~MultiObjectiveController() {}


void MultiObjectiveController::InitializeParameters() {
  robot_dof_ = mav_dof_+arm_dof_;
  unsigned int control_in_sz = robot_dof_;
  minimizer_sz = robot_dof_ + control_in_sz;

  calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));
  torque_to_rotor_velocities_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  torque_to_rotor_velocities_ = pseudoInv(controller_parameters_.allocation_matrix_);

  // Initialize linear contraints
  Aeq_ = Eigen::MatrixXd::Zero(robot_dof_+2,minimizer_sz);
  Aeq_.block(control_in_sz,control_in_sz,0,robot_dof_) = Eigen::MatrixXd::Identity(control_in_sz,control_in_sz);
  Beq_ = Eigen::VectorXd::Zero(robot_dof_+2);
  Aineq_ = Eigen::Matrix2Xd::Zero(2,minimizer_sz);
  Bineq_ << 54, 0; // TBD use some parameter instead of 54 (max vertical thrust)
  upper_bounds_ <<  Eigen::Vector3d::Ones()*Eigen::Infinity,
                    Eigen::VectorXd::Zero(robot_dof_-3),
                    Eigen::Vector3d::Ones()*Eigen::Infinity,
                    Eigen::Vector3d::Ones(),
                    Eigen::VectorXd::Ones(arm_dof_)*2;
  lower_bounds_ = -upper_bounds_;
  initialized_params_ = true;
}


void MultiObjectiveController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) {
  assert(rotor_velocities);
  assert(initialized_params_);

  rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());
  // Return 0 velocities on all rotors, until the first command is received.
  if (!controller_active_) {
    *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
    return;
  }

  Eigen::VectorXd minimizer_;
  SolveMultiObjectiveOptimization(&minimizer_);

  // Extract thrust force from aerodynamic forces given in world frame and current UAV orientation
  double thrust = (odometry_.orientation.toRotationMatrix().transpose() * minimizer_.segment(robot_dof_,robot_dof_+2)).tail<1>()(0);
  Eigen::Vector4d thrust_torque;
  thrust_torque << thrust, minimizer_.segment(robot_dof_+3,robot_dof_+5);

  *rotor_velocities = torque_to_rotor_velocities_ * thrust_torque;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
}


void MultiObjectiveController::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}


void MultiObjectiveController::SetOdometryEndEffector(const EigenOdometry& odometry) {
  odometry_ee_ = odometry;
}


void MultiObjectiveController::SetArmJointsState(const EigenJointsState& joints_state) {
  joints_state_ = joints_state;
}


void MultiObjectiveController::SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}


/////////////////////// PRIVATE METHODs //////////////////////

void MultiObjectiveController::SolveMultiObjectiveOptimization(Eigen::VectorXd* _minimizer) {
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(minimizer_sz,minimizer_sz);
  Eigen::VectorXd c = Eigen::VectorXd::Zero(minimizer_sz);
  Eigen::MatrixXd* Q_array = new Eigen::MatrixXd[controller_parameters_.objectives_weight_.size()];
  Eigen::VectorXd* c_array = new Eigen::VectorXd[controller_parameters_.objectives_weight_.size()];
  static Eigen::VectorXd f = Eigen::VectorXd::Ones(minimizer_sz)*(-Eigen::Infinity);

  UpdateLinearConstraints();

  unsigned int i = 0;
  GetAttitudeSetPtObjective(&Q_array[i],&c_array[i]);
  GetManipulatorSetPtObjective(&Q_array[i++],&c_array[i++]);

  for (i=0; i<controller_parameters_.objectives_weight_.size(); i++) {
    Q += controller_parameters_.objectives_weight_(i) * Q_array[i];
    c += controller_parameters_.objectives_weight_(i) * c_array[i];
  }

//  if (!ooqpei::OoqpEigenInterface::solve(Q, c, Aeq_, Beq_, Aineq_, Bineq_, f, lower_bounds_, upper_bounds_, _minimizer)) {
//    ROS_INFO("[multi_objective_controller] Optimization failed.");
//    return;
//  }
  return;
}


/*
 * Template to compute quadratic terms for a set-point objective:
 * servo the task 'g' around a given reference value 'g_ref'.
*/
void MultiObjectiveController::GetSetPointObjective(const Eigen::VectorXd& g, const Eigen::VectorXd& g_ref,
                                                   const Eigen::VectorXd& kp, const Eigen::VectorXd& kv,
                                                   const Eigen::MatrixXd& Jg, const Eigen::MatrixXd& Jg_dot,
                                                   const Eigen::VectorXd& q_dot, Eigen::MatrixXd* Q,
                                                   Eigen::VectorXd* c) const {

  *Q = Eigen::MatrixXd::Zero(robot_dof_,robot_dof_);
  (*Q).topLeftCorner(robot_dof_,robot_dof_) = Eigen::MatrixXd::Identity(robot_dof_, robot_dof_);//Jg.transpose() * Jg;

  *c = Eigen::VectorXd::Zero(robot_dof_*2);
  (*c).head(robot_dof_) = -Jg.transpose()*(kp.cwiseProduct(g_ref-g) - kv.cwiseProduct(Jg*q_dot) - Jg_dot*q_dot);
}


void MultiObjectiveController::GetAttitudeSetPtObjective(Eigen::MatrixXd* _Q, Eigen::VectorXd* _c) const {
  Eigen::MatrixXd* Q_pos_ptr;
  Eigen::MatrixXd* Q_orient_ptr;
  Eigen::VectorXd* c_pos_ptr;
  Eigen::VectorXd* c_orient_ptr;

  GetPositionSetPtObjective(Q_pos_ptr,c_pos_ptr);
  GetOrientationSetPtObjective(Q_orient_ptr,c_orient_ptr);

  *_Q = *Q_pos_ptr + *Q_orient_ptr;
  *_c = *c_pos_ptr + *c_orient_ptr;
}


void MultiObjectiveController::GetPositionSetPtObjective(Eigen::MatrixXd* _Q, Eigen::VectorXd* _c) const {
  static Eigen::Matrix3Xd Jac_ = Eigen::Matrix3Xd::Zero(3,robot_dof_);
  static Eigen::Matrix3Xd Jac_dot_ = Jac_;
  Jac_.topLeftCorner(3,3) = Eigen::Matrix3d::Identity();

  Eigen::VectorXd robot_vel = GetRobotVelocities();

  GetSetPointObjective(odometry_.position, command_trajectory_.position_W, controller_parameters_.mav_position_gain_,
                       controller_parameters_.mav_velocity_gain_, Jac_, Jac_dot_, robot_vel, _Q, _c);
}


void MultiObjectiveController::GetOrientationSetPtObjective(Eigen::MatrixXd* _Q, Eigen::VectorXd* _c) const {
  static Eigen::Matrix3Xd Jac_ = Eigen::Matrix3Xd::Zero(3,robot_dof_);
  static Eigen::Matrix3Xd Jac_dot_ = Jac_;
  Jac_.block<3,3>(0,3) = Eigen::Matrix3d::Identity();

  Eigen::VectorXd robot_vel = GetRobotVelocities();

  Eigen::Vector3d rpy_des(0,0,command_trajectory_.getYaw());

  Eigen::Vector3d rpy = odometry_.orientation.toRotationMatrix().eulerAngles(0,1,2);

  GetSetPointObjective(rpy, rpy_des, controller_parameters_.mav_attitude_gain_,
                       controller_parameters_.mav_angular_rate_gain_, Jac_, Jac_dot_, robot_vel, _Q, _c);
}


void MultiObjectiveController::GetManipulatorSetPtObjective(Eigen::MatrixXd* _Q, Eigen::VectorXd* _c) const {
  static Eigen::MatrixXd Jac_ = Eigen::MatrixXd::Zero(arm_dof_,robot_dof_);
  static Eigen::MatrixXd Jac_dot_ = Jac_;
  Jac_.topRightCorner(arm_dof_,arm_dof_) = Eigen::MatrixXd::Identity(arm_dof_,arm_dof_);

  Eigen::VectorXd robot_vel = GetRobotVelocities();

  GetSetPointObjective(joints_state_.angles, joints_angle_des_, controller_parameters_.arm_joints_angle_gain_,
                       controller_parameters_.arm_joints_ang_rate_gain_, Jac_, Jac_dot_, robot_vel, _Q, _c);
}


Eigen::VectorXd MultiObjectiveController::GetRobotVelocities() const {
  Eigen::VectorXd robot_vel(robot_dof_);
  robot_vel.head(3) = odometry_.velocity;
  robot_vel.segment(3,mav_dof_-1) = odometry_.angular_velocity; // TBD: convert ang.vel. from body frame to world frame!!
  robot_vel.tail(arm_dof_) = joints_state_.angular_rates;

  return robot_vel;
}


void MultiObjectiveController::UpdateLinearConstraints() {
  Eigen::Matrix3d Rot_w2v = odometry_.orientation.toRotationMatrix().transpose();

  Aeq_.topLeftCorner(robot_dof_,robot_dof_) = dyn_mdl_terms_.inertia_matrix;
  Aeq_.block<2,3>(robot_dof_,robot_dof_) = Rot_w2v.topRows(2);

  Beq_.head(robot_dof_) = -(dyn_mdl_terms_.coriolis_matrix + dyn_mdl_terms_.damping_matrix)*GetRobotVelocities()
                          - dyn_mdl_terms_.gravity_vector;

  Aineq_.block<1,3>(0,robot_dof_) = Rot_w2v.bottomRows(1);
  Aineq_.block<1,3>(1,robot_dof_) = -Rot_w2v.bottomRows(1);

  upper_bounds_.segment(3,mav_dof_-1) = controller_parameters_.mu_attitude_*(
                                        controller_parameters_.safe_range_*controller_parameters_.rpy_max_ -
                                        odometry_.orientation.toRotationMatrix().eulerAngles(0,1,2) );

  upper_bounds_.segment(mav_dof_,robot_dof_-1) = controller_parameters_.mu_arm_*(
                                        controller_parameters_.safe_range_*controller_parameters_.arm_joints_angle_max_ -
                                        joints_state_.angles );

  lower_bounds_.segment(3,mav_dof_-1) = controller_parameters_.mu_attitude_*(
                                        controller_parameters_.safe_range_*controller_parameters_.rpy_min_ -
                                        odometry_.orientation.toRotationMatrix().eulerAngles(0,1,2) );

  lower_bounds_.segment(mav_dof_,robot_dof_-1) = controller_parameters_.mu_arm_*(
                                        controller_parameters_.safe_range_*controller_parameters_.arm_joints_angle_min_ -
                                        joints_state_.angles );
}


// Specific to Aerial Delta Manipulator
void MultiObjectiveController::UpdateDynamicModelTerms() {
  double *q_in;
  Eigen::VectorXd q_eig;
  q_eig << odometry_.position, odometry_.orientation.toRotationMatrix().eulerAngles(0,1,2), joints_state_.angles;

  double *q34_12;
  Eigen::Map<Eigen::VectorXd>(q_in,2,1) = q_eig.segment(7,8);
  q34_12_fun(q_in,q34_12);

  double *C_;
  q_eig << q_eig.tail(3+arm_dof_), q34_12[0], q34_12[1], GetRobotVelocities();
  Eigen::Map<Eigen::VectorXd>(q_in, q_eig.rows(), 1) = q_eig;
  C_fun(q_in,C_);

  double *M_triu_;
  Eigen::Map<Eigen::VectorXd>(q_in, q_eig.head(5+arm_dof_).rows(), 1) = q_eig.head(5+arm_dof_);
  M_triu_fun(q_in,M_triu_);

  double *G_;
  q_eig.tail(q_eig.rows()-2) = q_eig.tail(q_eig.rows()-3);
  Eigen::Map<Eigen::VectorXd>(q_in, q_eig.head(5+arm_dof_).rows(), 1) = q_eig;//.resize(q_eig.rows()-1);
  G_fun(q_in,G_);

  Eigen::MatrixXd M_eig = Eigen::Map<Eigen::MatrixXd>(M_triu_, robot_dof_, robot_dof_);
  M_eig.triangularView<Eigen::StrictlyLower>() = M_eig.transpose();
  Eigen::MatrixXd C_eig = Eigen::Map<Eigen::MatrixXd>(C_, robot_dof_, robot_dof_);
  Eigen::VectorXd G_eig = Eigen::Map<Eigen::VectorXd>(G_, robot_dof_, 1);
  Eigen::MatrixXd D_eig = Eigen::MatrixXd::Identity(robot_dof_, robot_dof_) * 0.01;

  SetDynamicModelTerms(M_eig,C_eig,D_eig,G_eig);
}


void MultiObjectiveController::SetDynamicModelTerms(const Eigen::MatrixXd& _inertia_matrix,
                                                    const Eigen::MatrixXd& _coriolis_matrix,
                                                    const Eigen::MatrixXd& _damping_matrix,
                                                    const Eigen::VectorXd& _gravity_vector) {
  dyn_mdl_terms_.inertia_matrix = _inertia_matrix;
  dyn_mdl_terms_.coriolis_matrix = _coriolis_matrix;
  dyn_mdl_terms_.damping_matrix = _damping_matrix;
  dyn_mdl_terms_.gravity_vector = _gravity_vector;
}

}

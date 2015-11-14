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
#include <stdlib.h>


namespace rotors_control {

MultiObjectiveController::MultiObjectiveController()
    : initialized_params_(false),
      controller_active_(false),
      mav_trajectory_received_(false),
      arm_trajectory_received_(false),
      ee_trajectory_received_(false),
      mav_dof_(kDefaultMavDof),
      arm_dof_(kDefaultArmDof) {

  robot_dof_ = mav_dof_+arm_dof_;
  minimizer_sz_ = 2*robot_dof_;
}


MultiObjectiveController::~MultiObjectiveController() {}


void MultiObjectiveController::InitializeParameters() {
  calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));
  torque_to_rotor_velocities_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  torque_to_rotor_velocities_ = pseudoInv(controller_parameters_.allocation_matrix_);

  // Initialize linear contraints
  Aeq_ = Eigen::MatrixXd::Zero(robot_dof_+2,minimizer_sz_);
  Aeq_.block(0,robot_dof_,robot_dof_,robot_dof_) = Eigen::MatrixXd::Identity(robot_dof_,robot_dof_);
  Beq_ = Eigen::VectorXd::Zero(robot_dof_+2);
  Aineq_ = Eigen::Matrix2Xd::Zero(2,minimizer_sz_);
  Bineq_.resize(2);
  Bineq_ << 54, 0; // TBD use some parameter instead of 54 (max vertical thrust)
  upper_bounds_.resize(minimizer_sz_);
  upper_bounds_ <<  Eigen::Vector3d::Ones()*Eigen::Infinity,
                    Eigen::VectorXd::Zero(robot_dof_-3),
                    Eigen::Vector3d::Ones()*Eigen::Infinity,
                    Eigen::Vector3d::Ones(),
                    Eigen::VectorXd::Ones(arm_dof_)*2;
  lower_bounds_.resize(minimizer_sz_);
  lower_bounds_ = -upper_bounds_;

  initialized_params_ = true;

  //debug
//  std::cout << robot_dof_ << std::endl;
//  std::cout << "Aeq_ : " << Aeq_.size() <<" = "<< Aeq_.rows()<<"x"<< Aeq_.cols()<< std::endl;
//  std::cout << "Beq_ : " << Beq_.size() <<" = "<< Beq_.rows()<<"x"<< Beq_.cols()<< std::endl;
//  std::cout << "Aineq_ : " << Aineq_.size() <<" = "<< Aineq_.rows()<<"x"<< Aineq_.cols()<< std::endl;
//  std::cout << "Bineq_ : " << Bineq_.size() <<" = "<< Bineq_.rows()<<"x"<< Bineq_.cols()<< std::endl;
//  std::cout << "upper_bounds_ : " << upper_bounds_.size() <<" = "<< upper_bounds_.rows()<<"x"<< upper_bounds_.cols()<< std::endl;
//  std::cout << "lower_bounds_ : " << lower_bounds_.size() <<" = "<< lower_bounds_.rows()<<"x"<< lower_bounds_.cols()<< std::endl;
}


void MultiObjectiveController::CalculateControlInputs(Eigen::VectorXd* rotor_velocities, Eigen::Vector3d* torques) {
  assert(rotor_velocities);
  assert(torques);
  assert(initialized_params_);

  rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());
  // Return 0 velocities on all rotors, until the first command is received.
  if (!controller_active_) {
    *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
    return;
  }

  Eigen::VectorXd minimizer_;
  SolveMultiObjectiveOptimization(&minimizer_);
//  ROS_INFO("CHECK POINT after SolveMultiObjectiveOptimization");  //debug

  // Extract thrust force from aerodynamic forces given in world frame and current UAV orientation
  double thrust = (odometry_.orientation.toRotationMatrix().transpose() * minimizer_.segment<3>(robot_dof_)).tail<1>()(0);
  Eigen::Vector4d thrust_torque;
  thrust_torque << thrust, minimizer_.segment<3>(robot_dof_+3);

  *rotor_velocities = torque_to_rotor_velocities_ * thrust_torque;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();

  *torques = minimizer_.tail(arm_dof_);
}


void MultiObjectiveController::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}


void MultiObjectiveController::SetArmJointsState(const manipulator_msgs::EigenJointsState& joints_state) {
  joints_state_ = joints_state;

  //debug
//  std::cout << "Joints state position vector :\t" << joints_state_.angles.transpose() << std::endl;
//  std::cout << "Joints state velocity vector :\t" << joints_state_.angular_rates.transpose() << std::endl;
}


void MultiObjectiveController::SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  ROS_INFO_ONCE("MultiObjectiveController got first helicopter trajectory point.");

  mav_trajectory_received_ = true;
  controller_active_ = true;
}


void MultiObjectiveController::SetEndEffTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ee_ = command_trajectory;
  ROS_INFO_ONCE("MultiObjectiveController got first end-effector trajectory point.");

  ee_trajectory_received_ = true;
  controller_active_ = true;
}


void MultiObjectiveController::SetDesiredJointsAngle(const manipulator_msgs::EigenJointTrajectoryPoint& joints_state) {
  joints_angle_des_ = joints_state.angles;
  ROS_INFO_ONCE("MultiObjectiveController got first joints angle trajectory point.");

  arm_trajectory_received_ = true;
  controller_active_ = true;
}


void MultiObjectiveController::SetObjectiveFunctionsWeight(const Eigen::VectorXd& objectives_weight) {
  controller_parameters_.objectives_weight_.resizeLike(objectives_weight);
  controller_parameters_.objectives_weight_ = objectives_weight;
}


void MultiObjectiveController::SetExternalForces(const Eigen::Vector3d& forces) {
  //Todo : ignore small forces (just noise)
  ext_forces_ = forces;
}

/////////////////////// PRIVATE METHODs //////////////////////

void MultiObjectiveController::SolveMultiObjectiveOptimization(Eigen::VectorXd* _minimizer) {
  Eigen::MatrixXd Q(minimizer_sz_,minimizer_sz_);
  Eigen::VectorXd c(minimizer_sz_);
  Eigen::MatrixXd Q_sum = Eigen::MatrixXd::Zero(minimizer_sz_,minimizer_sz_);
  Eigen::VectorXd c_sum = Eigen::VectorXd::Zero(minimizer_sz_);
  static Eigen::VectorXd f = Eigen::VectorXd::Ones(minimizer_sz_)*(-Eigen::Infinity);
  static Eigen::VectorXd weights_normalized = controller_parameters_.objectives_weight_.normalized();

  UpdateLinearConstraints();
//  ROS_INFO("CHECK POINT after UpdateLinearConstraints");  //debug

  Eigen::VectorXd current_weights = weights_normalized;
  if (!mav_trajectory_received_) {
    current_weights << 0, 0, 2, controller_parameters_.objectives_weight_.tail<3>();
    current_weights.normalize();
  }
  if (!(arm_trajectory_received_ or ee_trajectory_received_)) {
    current_weights << controller_parameters_.objectives_weight_.head<3>(), 0, 0, 2;
    current_weights.normalize();
  }


  for (unsigned int i=0; i<current_weights.size(); i++) {
    if (current_weights(i) == 0)
      continue;

    switch (i) {
      case (0):
        GetAttitudeSetPtObjective(&Q,&c);
//        ROS_INFO("CHECK POINT after GetAttitudeSetPtObjective");  //debug
        break;

      case (1):
        GetYawSetPtObjective(&Q,&c);
//        ROS_INFO("CHECK POINT after GetYawSetPtObjective");  //debug
        break;

      case (2):
        GetFreeHoverSetPtObjective(&Q,&c);
//        ROS_INFO("CHECK POINT after GetFreeHoverSetPtObjective");  //debug
        break;

      case (3):
        if (!arm_trajectory_received_)
          continue;
        GetManipulatorSetPtObjective(&Q,&c);
//        ROS_INFO("CHECK POINT after GetManipulatorSetPtObjective");  //debug
        break;

      case (4):
        if (!ee_trajectory_received_)
          continue;
        GetEndEffectorSetPtObjective(&Q,&c);
//        ROS_INFO("CHECK POINT after GetEndEffectorSetPtObjective");  //debug
        break;

      case (5):
        GetDeadArmSetPtObjective(&Q,&c);
//        ROS_INFO("CHECK POINT after GetDeadArmSetPtObjective");  //debug
        break;

      default:
        Q.fill(0.0);
        c.fill(0.0);
    }

    Q_sum += current_weights(i) * Q;
    c_sum += current_weights(i) * c;
  }

  //TODO
//  if (!ooqpei::OoqpEigenInterface::solve(Q, c, Aeq_, Beq_, Aineq_, Bineq_, f, lower_bounds_, upper_bounds_, _minimizer)) {
//    ROS_INFO("[multi_objective_controller] Optimization failed.");
//    return;
//  }

  //dummy output
  *_minimizer = Eigen::VectorXd::Zero(minimizer_sz_);
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

  *Q = Eigen::MatrixXd::Zero(minimizer_sz_,minimizer_sz_);
  (*Q).topLeftCorner(robot_dof_,robot_dof_) = Jg.transpose() * Jg;

  *c = Eigen::VectorXd::Zero(minimizer_sz_);
  (*c).head(robot_dof_) = -Jg.transpose()*(kp.cwiseProduct(g_ref-g) - kv.cwiseProduct(Jg*q_dot) - Jg_dot*q_dot);
}


void MultiObjectiveController::GetAttitudeSetPtObjective(Eigen::MatrixXd* _Q, Eigen::VectorXd* _c) const {

  assert(mav_trajectory_received_);

  Eigen::MatrixXd Q_pos;
  Eigen::MatrixXd Q_orient;
  Eigen::VectorXd c_pos;
  Eigen::VectorXd c_orient;

  GetPositionSetPtObjective(command_trajectory_.position_W, &Q_pos, &c_pos);

  Eigen::Vector3d rpy_des(0,0,command_trajectory_.getYaw());
  GetOrientationSetPtObjective(rpy_des, &Q_orient, &c_orient);

  *_Q = Q_pos + Q_orient;
  *_c = c_pos + c_orient;
}


void MultiObjectiveController::GetYawSetPtObjective(Eigen::MatrixXd* _Q, Eigen::VectorXd* _c) const {

  assert(mav_trajectory_received_);

  Eigen::MatrixXd Q_pos;
  Eigen::MatrixXd Q_orient;
  Eigen::VectorXd c_pos;
  Eigen::VectorXd c_orient;

  GetPositionSetPtObjective(odometry_.position, &Q_pos, &c_pos);

  Eigen::Vector3d rpy_des(0,0,command_trajectory_.getYaw());
  GetOrientationSetPtObjective(rpy_des, &Q_orient, &c_orient);

  *_Q = Q_pos + Q_orient;
  *_c = c_pos + c_orient;
}


void MultiObjectiveController::GetFreeHoverSetPtObjective(Eigen::MatrixXd* _Q, Eigen::VectorXd* _c) const {
  Eigen::MatrixXd Q_pos;
  Eigen::MatrixXd Q_orient;
  Eigen::VectorXd c_pos;
  Eigen::VectorXd c_orient;

  GetPositionSetPtObjective(odometry_.position, &Q_pos, &c_pos);

  Eigen::Vector3d rpy_des(0,0,odometry_.orientation.toRotationMatrix().eulerAngles(0,1,2)(2));
  GetOrientationSetPtObjective(rpy_des, &Q_orient, &c_orient);

  *_Q = Q_pos + Q_orient;
  *_c = c_pos + c_orient;
}


void MultiObjectiveController::GetPositionSetPtObjective(const Eigen::Vector3d& _position_ref,
                                                         Eigen::MatrixXd* _Q, Eigen::VectorXd* _c) const {
  static Eigen::Matrix3Xd Jac_ = Eigen::Matrix3Xd::Zero(3,robot_dof_);
  static Eigen::Matrix3Xd Jac_dot_ = Jac_;
  Jac_.topLeftCorner(3,3) = Eigen::Matrix3d::Identity();

  Eigen::VectorXd robot_vel = GetRobotVelocities();

  GetSetPointObjective(odometry_.position, _position_ref, controller_parameters_.mav_position_gain_,
                       controller_parameters_.mav_velocity_gain_, Jac_, Jac_dot_, robot_vel, _Q, _c);
}


void MultiObjectiveController::GetOrientationSetPtObjective(const Eigen::Vector3d& _orientation_ref,
                                                            Eigen::MatrixXd* _Q, Eigen::VectorXd* _c) const {
  static Eigen::Matrix3Xd Jac_ = Eigen::Matrix3Xd::Zero(3,robot_dof_);
  static Eigen::Matrix3Xd Jac_dot_ = Jac_;
  Jac_.block<3,3>(0,3) = Eigen::Matrix3d::Identity();

  Eigen::VectorXd robot_vel = GetRobotVelocities();

  Eigen::Vector3d rpy = odometry_.orientation.toRotationMatrix().eulerAngles(0,1,2);

  GetSetPointObjective(rpy, _orientation_ref, controller_parameters_.mav_attitude_gain_,
                       controller_parameters_.mav_angular_rate_gain_, Jac_, Jac_dot_, robot_vel, _Q, _c);
}


void MultiObjectiveController::GetManipulatorSetPtObjective(Eigen::MatrixXd* _Q, Eigen::VectorXd* _c) const {
  static Eigen::MatrixXd Jac_ = Eigen::MatrixXd::Zero(arm_dof_,robot_dof_);
  static Eigen::MatrixXd Jac_dot_ = Jac_;

  assert(arm_trajectory_received_);

  Jac_.topRightCorner(arm_dof_,arm_dof_) = Eigen::MatrixXd::Identity(arm_dof_,arm_dof_);

  Eigen::VectorXd robot_vel = GetRobotVelocities();

  GetSetPointObjective(joints_state_.angles, joints_angle_des_, controller_parameters_.arm_joints_angle_gain_,
                       controller_parameters_.arm_joints_ang_rate_gain_, Jac_, Jac_dot_, robot_vel, _Q, _c);
}


void MultiObjectiveController::GetDeadArmSetPtObjective(Eigen::MatrixXd* _Q, Eigen::VectorXd* _c) const {
  static Eigen::MatrixXd Jac_ = Eigen::MatrixXd::Zero(arm_dof_,robot_dof_);
  static Eigen::MatrixXd Jac_dot_ = Jac_;
  Jac_.topRightCorner(arm_dof_,arm_dof_) = Eigen::MatrixXd::Identity(arm_dof_,arm_dof_);

  Eigen::VectorXd robot_vel = GetRobotVelocities();

  GetSetPointObjective(joints_state_.angles, joints_state_.angles, controller_parameters_.arm_joints_angle_gain_,
                       controller_parameters_.arm_joints_ang_rate_gain_, Jac_, Jac_dot_, robot_vel, _Q, _c);
}


void MultiObjectiveController::GetEndEffectorSetPtObjective(Eigen::MatrixXd* _Q, Eigen::VectorXd* _c) {

  assert(ee_trajectory_received_);

  UpdateEndEffectorState();

  Eigen::VectorXd robot_vel = GetRobotVelocities();

  GetSetPointObjective(end_effector_.odometry.position, command_trajectory_ee_.position_W, controller_parameters_.ee_position_gain_,
                       controller_parameters_.ee_velocity_gain_, end_effector_.jacobian_W, end_effector_.jacobian_dot_W, robot_vel, _Q, _c);
}


Eigen::VectorXd MultiObjectiveController::GetRobotVelocities() const {
  Eigen::VectorXd robot_vel(robot_dof_);
  robot_vel.head(3) = odometry_.velocity;
  //TODO : convert ang.vel. from body frame to world frame!!
  robot_vel.segment<3>(3) = odometry_.angular_velocity;
  robot_vel.tail(arm_dof_) = joints_state_.angular_rates;

  return robot_vel;
}


void MultiObjectiveController::UpdateLinearConstraints() {
  static Eigen::Vector3d rpy_max = pow(controller_parameters_.safe_range_,3)*
                                  (controller_parameters_.rpy_max_ - controller_parameters_.rpy_min_) +
                                  controller_parameters_.rpy_min_;
  static Eigen::Vector3d rpy_min = pow(controller_parameters_.safe_range_,3)*
                                  (controller_parameters_.rpy_min_ - controller_parameters_.rpy_max_) +
                                  controller_parameters_.rpy_max_;
  static Eigen::Vector3d arm_joints_angle_max = controller_parameters_.safe_range_*
                                  (controller_parameters_.arm_joints_angle_max_ - controller_parameters_.arm_joints_angle_min_) +
                                  controller_parameters_.arm_joints_angle_min_;
  static Eigen::Vector3d arm_joints_angle_min = controller_parameters_.safe_range_*
                                  (controller_parameters_.arm_joints_angle_min_ - controller_parameters_.arm_joints_angle_max_) +
                                  controller_parameters_.arm_joints_angle_max_;

  Eigen::Matrix3d Rot_w2v = odometry_.orientation.toRotationMatrix().transpose();

  UpdateDynamicModelTerms();
//  ROS_INFO("CHECK POINT after UpdateDynamicModelTerms");  //debug

  Aeq_.topLeftCorner(robot_dof_,robot_dof_) = dyn_mdl_terms_.inertia_matrix;
  Aeq_.block<2,3>(robot_dof_,robot_dof_) = Rot_w2v.topRows(2);

  Beq_.head(robot_dof_) = -(dyn_mdl_terms_.coriolis_matrix + dyn_mdl_terms_.damping_matrix)*GetRobotVelocities()
                          - dyn_mdl_terms_.gravity_vector;

  Aineq_.block<1,3>(0,robot_dof_) = Rot_w2v.bottomRows(1);
  Aineq_.block<1,3>(1,robot_dof_) = -Rot_w2v.bottomRows(1);

  upper_bounds_.segment<3>(3) = controller_parameters_.mu_attitude_*(rpy_max -
                                  odometry_.orientation.toRotationMatrix().eulerAngles(0,1,2) );
  upper_bounds_.segment<3>(6) = controller_parameters_.mu_arm_*(arm_joints_angle_max -
                                  joints_state_.angles );

  lower_bounds_.segment<3>(3) = controller_parameters_.mu_attitude_*(rpy_min -
                                  odometry_.orientation.toRotationMatrix().eulerAngles(0,1,2) );
  lower_bounds_.segment<3>(6) = controller_parameters_.mu_arm_*(arm_joints_angle_min -
                                  joints_state_.angles );
}


// Specific to Aerial Delta Manipulator
void MultiObjectiveController::UpdateEndEffectorState() {

  static Eigen::VectorXi p_ee_idx = [] {
      Eigen::VectorXi tmp(9);
      tmp << 0, 1, 2, 3, 4, 6, 7, 8, 10; // x, y, z, roll, pitch, yaw, q0, q1, q3
      return tmp;
  }();
  static Eigen::VectorXi J_e_dot_idx = [] {
      Eigen::VectorXi tmp(14);
      tmp << 3, 4, 5, 6, 7, 8, 9, 10, 14, 15, 16, 17, 18, 19; // roll, pitch, yaw, q0, q1, q2, q3, q4,
      return tmp;
  }();

  Eigen::VectorXd q_eig(robot_dof_);
  q_eig << odometry_.position, odometry_.orientation.toRotationMatrix().eulerAngles(0,1,2), joints_state_.angles;

  double q_in[2];
  double q34_12[2];
  Eigen::Vector2d::Map(q_in) = q_eig.tail<2>(); // q1,q2
  q34_12_fun(q_in,q34_12);
  Eigen::Map<Eigen::Vector2d> q34_12_map(q34_12);

  Eigen::VectorXd X_eig(2*robot_dof_+2);
  X_eig << q_eig, q34_12_map, GetRobotVelocities();

  double J_e[6*robot_dof_];
  double q_in_J_e[8];
  Eigen::VectorXd::Map(q_in_J_e, 8) = X_eig.segment<8>(3);
  J_e_fun(q_in_J_e,J_e);

  double J_e_dot[6*robot_dof_];
  double q_in_J_e_dot[J_e_dot_idx.size()];
  Eigen::VectorXd q_in_eig(J_e_dot_idx.size());
  igl::slice(X_eig, J_e_dot_idx, q_in_eig);
  Eigen::VectorXd::Map(q_in_J_e_dot, J_e_dot_idx.size()) = q_in_eig;
  J_e_dot_fun(q_in_J_e_dot,J_e_dot);

  double p_ee[6];
  double q_in_p_ee[p_ee_idx.size()];
  q_in_eig.resize(p_ee_idx.size());
  igl::slice(X_eig, J_e_dot_idx, q_in_eig);
  Eigen::VectorXd::Map(q_in_p_ee, p_ee_idx.size()) = q_in_eig;
  p_ee_fun(q_in_p_ee,p_ee);

  Eigen::Map<Eigen::MatrixXd> J_e_eig(J_e, 6, robot_dof_);
  Eigen::Map<Eigen::MatrixXd> J_e_dot_eig(J_e_dot, 6, robot_dof_);
  Eigen::Map<Eigen::VectorXd> p_ee_eig(p_ee, 6);

  end_effector_.setPosJac(p_ee_eig, J_e_eig, J_e_dot_eig);
}


// Specific to Aerial Delta Manipulator
void MultiObjectiveController::UpdateDynamicModelTerms() {

  static Eigen::VectorXi G_idx = [] {
      Eigen::VectorXi tmp(7);
      tmp << 3, 4, 6, 7, 8, 9, 10; // roll, pitch, q0, q1, q2, q3, q4
      return tmp;
  }();
  static Eigen::VectorXi C_idx = [] {
      Eigen::VectorXi tmp(17);
      tmp << 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19; // roll, pitch, yaw, q0, q1, q2, q3, q4, dq[9]
      return tmp;
  }();

  Eigen::VectorXd q_eig(robot_dof_);
  q_eig << odometry_.position, odometry_.orientation.toRotationMatrix().eulerAngles(0,1,2), joints_state_.angles;

  double q_in[2];
  double q34_12[2];
  Eigen::Vector2d::Map(q_in) = q_eig.tail<2>(); // q1,q2
  q34_12_fun(q_in,q34_12);
  Eigen::Map<Eigen::Vector2d> q34_12_map(q34_12);

  Eigen::VectorXd X_eig(2*robot_dof_+2);
  X_eig << q_eig, q34_12_map, GetRobotVelocities();

  double C_[robot_dof_*robot_dof_];
  double q_in_C[C_idx.rows()];
  Eigen::VectorXd q_in_eig(C_idx.rows());
  igl::slice(X_eig, C_idx, q_in_eig);
  Eigen::VectorXd::Map(q_in_C, q_in_eig.rows()) = q_in_eig;
  C_fun(q_in_C,C_);

  double M_triu_[robot_dof_*robot_dof_];
  double q_in_M[5+arm_dof_];
  Eigen::VectorXd::Map(q_in_M, 5+arm_dof_) = q_in_eig.head(5+arm_dof_);
  M_triu_fun(q_in_M,M_triu_);

  double G_[robot_dof_];
  double q_in_G[G_idx.size()];
  q_in_eig.resize(G_idx.size());
  igl::slice(X_eig, G_idx, q_in_eig);
  Eigen::VectorXd::Map(q_in_G, q_in_eig.rows()) = q_in_eig;
  G_fun(q_in_G,G_);

  Eigen::Map<Eigen::MatrixXd> M_eig(M_triu_, robot_dof_, robot_dof_);
  M_eig.triangularView<Eigen::StrictlyLower>() = M_eig.transpose();
  Eigen::Map<Eigen::MatrixXd> C_eig(C_, robot_dof_, robot_dof_);
  Eigen::Map<Eigen::VectorXd> G_eig(G_, robot_dof_);
  Eigen::MatrixXd D_eig = Eigen::MatrixXd::Identity(robot_dof_, robot_dof_) * 0.01;

  dyn_mdl_terms_.setAll(M_eig,C_eig,D_eig,G_eig);
}


}

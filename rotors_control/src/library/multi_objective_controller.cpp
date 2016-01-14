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
#include <ctime>


#define PRECISION 7 //StreamPrecision FullPrecision

using namespace Eigen;

// Matrix and vector output format for matlab friendly printings when debugging.
IOFormat MatlabMatrixFmt(PRECISION, 0, ", ", ";\n", "", "", "[", "]");
IOFormat MatlabVectorFmt(PRECISION, DontAlignCols, ", ", "; ", "", "", "[ ", " ]");
IOFormat MatlabVector2Fmt(PRECISION, DontAlignCols, ", ", "; ", "", "", " ", " ");
/* Example usage:
 *
 *  std::cout << "Matrix_name = " << Matrix.format(MatlabMatrixFmt) << std::endl;
 *  std::cout << "SparseMatrix_name = " << SparseMatrix.toDense().format(MatlabMatrixFmt) << std::endl;
 *  std::cout << "Vector_name = " << Vector.format(MatlabVectorFmt) << std::endl;
 *  std::cout << "SplitVector = [ " << Vector1.format(MatlabVector2Fmt) << "; Number ;" << Vector2.format(MatlabVector2Fmt) << " ]" << std::endl;
 */


namespace rotors_control {


MultiObjectiveController::MultiObjectiveController()
    : initialized_params_(false),
      controller_active_(false),
      mav_trajectory_received_(false),
      arm_trajectory_received_(false),
      ee_trajectory_received_(false),
      set_frozen_joints_angle_des_(false),
      mav_dof_(kDefaultMavDof),
      arm_dof_(kDefaultArmDof) {

  // Dimensions.
  robot_dof_ = mav_dof_+arm_dof_;   // currently: 6+3 = 9
  minimizer_sz_ = 2*robot_dof_;     // currently: 9+9 = 18

  // Initialize robot velocities and Lagrangian coordinates.
  q_eig_ = VectorXd::Zero(robot_dof_);
  robot_vel_ = VectorXd::Zero(robot_dof_);
  X_eig_ = VectorXd::Zero(2*robot_dof_+2);
}


MultiObjectiveController::~MultiObjectiveController() {}

/************************************************************/
/******  PUBLIC METHODs  ************************************/
/************************************************************/

void MultiObjectiveController::InitializeParameters() {
  // Compute constant transformation matrix from aerodynamic thrust and torques to rotors speed.
  calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));
  size_t rotors_sz = vehicle_parameters_.rotor_configuration_.rotors.size();
  size_t ineq_no = 2+arm_dof_*2+rotors_sz;
  torque_to_rotor_velocities_.resize(rotors_sz, 4);
  torque_to_rotor_velocities_ = pseudoInv(controller_parameters_.allocation_matrix_);

  /* Solve QP problem:
   *           min    1/2 x' Q x + c' x ,
   *           s.t.       A x = b ,
   *                  d <= Cx <= f .
   */

  // Clear cost function quadratic and linear term and minimizer.
  Q_.resize(minimizer_sz_,minimizer_sz_);
  Q_.setZero();
  c_ = VectorXd::Zero(minimizer_sz_);
  x_ = VectorXd::Zero(minimizer_sz_);

  /* Initialize linear constraints setting right values for constant elements or constant portions
   * of them and filling with zeros remaining to-be-dynamically-updated entries. Refer to literature
   * for more details about structure of such elements.
   */
  A_.resize(robot_dof_+2,minimizer_sz_);
  A_.setZero();
  b_ = VectorXd::Zero(robot_dof_+2);
  C_.resize(ineq_no, minimizer_sz_);
  C_.setZero();
  MatrixXd C_temp = MatrixXd::Zero(ineq_no,minimizer_sz_);
  C_temp.block<2,2>(0,3) = Matrix2d::Identity();
  C_temp.block(2,mav_dof_,arm_dof_,arm_dof_) = MatrixXd::Identity(arm_dof_,arm_dof_);
  C_temp.bottomRightCorner(arm_dof_,arm_dof_) = MatrixXd::Identity(arm_dof_,arm_dof_);
  C_ = C_temp.sparseView();
  d_.resize(ineq_no);
  d_.setZero();
  d_.segment(2+arm_dof_,rotors_sz) = VectorXd::Zero(rotors_sz);
  d_.tail(arm_dof_) = VectorXd::Constant(arm_dof_,controller_parameters_.arm_joint_torque_lim_.x());
  f_.resize(ineq_no);
  f_.setZero();
  f_.segment(2+arm_dof_,rotors_sz) = VectorXd::Constant(rotors_sz,pow(controller_parameters_.max_rot_velocity_,2));
  f_.tail(arm_dof_) = VectorXd::Constant(arm_dof_,controller_parameters_.arm_joint_torque_lim_.y());

  initialized_params_ = true;
}


bool MultiObjectiveController::CalculateControlInputs(VectorXd* rotor_velocities, Vector3d* torques) {
  assert(rotor_velocities);
  assert(torques);
  assert(initialized_params_);

  rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());
  // Return 0 velocities on all rotors, until the first command is received.
  if (!controller_active_) {
    *rotor_velocities = VectorXd::Zero(rotor_velocities->rows());
    torques->setZero();
    return true;
  }

  // Run optimization and return error if it fails.
  if (!SolveMultiObjectiveOptimization())
    return false;

  // Extract thrust force from aerodynamic forces given in world frame considering current UAV orientation.
  double thrust = (odometry_.orientation_W_B.inverse() * x_.segment<3>(robot_dof_)).tail<1>()(0);
  Vector4d torque_thrust;
  torque_thrust << x_.segment<3>(robot_dof_+3), thrust;

  // Convert aerodynamic thrust and torques to rotors speed references.
  *rotor_velocities = torque_to_rotor_velocities_ * torque_thrust;
  *rotor_velocities = rotor_velocities->cwiseMax(VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();

  *torques = x_.tail(arm_dof_);

  return true;
}


void MultiObjectiveController::SetOdometry(const mav_msgs::EigenOdometry& odometry) {
  odometry_ = odometry;
}


void MultiObjectiveController::SetArmJointsState(const manipulator_msgs::EigenJointsState& joints_state) {
  static Vector3d dangerous_range = (1-controller_parameters_.safe_range_joints_)/2 * (controller_parameters_.arm_joints_angle_max_ - controller_parameters_.arm_joints_angle_min_);
  static Vector3d arm_joints_angle_min = controller_parameters_.arm_joints_angle_min_ + dangerous_range;
  static Vector3d arm_joints_angle_max = controller_parameters_.arm_joints_angle_max_ - dangerous_range;
  joints_state_ = joints_state;

  // This occurs only once after objective weights update or initialization.
  if (set_frozen_joints_angle_des_) {
    frozen_joints_angle_des_ = joints_state.angles;
    // Clip between joints angle limits
    frozen_joints_angle_des_ = frozen_joints_angle_des_.cwiseMax(arm_joints_angle_min);
    frozen_joints_angle_des_ = frozen_joints_angle_des_.cwiseMin(arm_joints_angle_max);
    set_frozen_joints_angle_des_ = false;
  }
}


void MultiObjectiveController::SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  ROS_INFO_ONCE("MultiObjectiveController got first UAV trajectory point.");

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


void MultiObjectiveController::SetDesiredFrozenJointsAngle() {
  set_frozen_joints_angle_des_ = true;
}


void MultiObjectiveController::SetExternalForces(const Vector3d& forces) {
  //Todo : ignore small forces (just noise)
  ext_forces_ = forces;
}

/************************************************************/
/******  OPTIMIZATION STEP  *********************************/
/************************************************************/

bool MultiObjectiveController::SolveMultiObjectiveOptimization() {
  std::clock_t startcputime = std::clock();

  // Update aerial manipulator Lagrangian first derivatives needed in set-point objective.
  UpdateRobotVelocities();

  // Here is where dynamic model is used.
  UpdateLinearConstraints();

  // Check for inequality constraints consistency.
  for (unsigned int i = 0; i<6; i++) {
    if (f_(i)<d_(i)) {
      ROS_WARN_THROTTLE(1, "[multi_objective_controller] Constraints on %i-th inequality are inconsistent.", i);
      return false;
    }
  }

  // For computational time evaluation
  std::clock_t after_update_cpu_time = std::clock();

  // Compute quadratic cost function according to current active objectives and reference inputs.
  ComputeCostFunction();

  // For computational time evaluation
  std::clock_t after_cost_cpu_time = std::clock();

  // Reset minimizer
  x_.setZero();

  // Call OOQP solver to obtain optimal solution, if exists.
  if (!ooqpei::OoqpEigenInterface::solve(Q_, c_, A_, b_, C_, d_, f_, x_)) {
    ROS_WARN_THROTTLE(1,"[multi_objective_controller] Optimization failed.");
    return false;
  }

  // Time performance evaluation
//  double cpu_duration = (std::clock() - startcputime) / (double)CLOCKS_PER_SEC;
//  double cpu_duration_update = (after_update_cpu_time - startcputime) / (double)CLOCKS_PER_SEC;
//  double cpu_duration_cost = (after_cost_cpu_time - after_update_cpu_time) / (double)CLOCKS_PER_SEC;
//  ROS_INFO_STREAM_THROTTLE(1, "Finished in " << cpu_duration << " seconds [CPU Clock] ");
//  std::cout << cpu_duration_update << "\t" << cpu_duration_cost << "\t" << cpu_duration << std::endl;

  return true;
}


void MultiObjectiveController::UpdateLinearConstraints() {
  static Vector2d rpy_max = controller_parameters_.safe_range_rpy_*
                            (controller_parameters_.rpy_max_ - controller_parameters_.rpy_min_) +
                            controller_parameters_.rpy_min_;
  static Vector2d rpy_min = controller_parameters_.safe_range_rpy_*
                            (controller_parameters_.rpy_min_ - controller_parameters_.rpy_max_) +
                            controller_parameters_.rpy_max_;
  static VectorXd arm_joints_angle_max = controller_parameters_.safe_range_joints_*
                            (controller_parameters_.arm_joints_angle_max_ - controller_parameters_.arm_joints_angle_min_) +
                            controller_parameters_.arm_joints_angle_min_;
  static VectorXd arm_joints_angle_min = controller_parameters_.safe_range_joints_*
                            (controller_parameters_.arm_joints_angle_min_ - controller_parameters_.arm_joints_angle_max_) +
                            controller_parameters_.arm_joints_angle_max_;
  static size_t rotors_sz = vehicle_parameters_.rotor_configuration_.rotors.size();

  Matrix3d Rot_w2v = odometry_.orientation_W_B.toRotationMatrix().transpose();

  // Here is where we use matlab-generated libraries.
  UpdateDynamicModelTerms();

  /* Recalling addressed QP problem:
   *
   *           min    1/2 x' Q x + c' x ,
   *           s.t.       A x = b ,
   *                  d <= Cx <= f .
   */

  // Update variable sectors of equality constraints terms.
  MatrixXd A_temp(robot_dof_+2,minimizer_sz_);
  A_temp.setZero();
  A_temp.topLeftCorner(robot_dof_,robot_dof_) = dyn_mdl_terms_.inertia_matrix;
  A_temp.topRightCorner(robot_dof_,robot_dof_) = -MatrixXd::Identity(robot_dof_,robot_dof_);
  A_temp.block<2,3>(robot_dof_,robot_dof_) = Rot_w2v.topRows(2);
  A_ = A_temp.sparseView();

  b_.head(robot_dof_) = -(dyn_mdl_terms_.coriolis_matrix + dyn_mdl_terms_.damping_matrix)*robot_vel_
                        - dyn_mdl_terms_.gravity_vector;

  // Update variable sectors of inequality constraints terms.
  MatrixXd C_rows_temp(rotors_sz,minimizer_sz_);
  C_rows_temp.setZero();
  C_rows_temp.block(0,robot_dof_,rotors_sz,3) = torque_to_rotor_velocities_.rightCols<1>() * Rot_w2v.bottomRows<1>();
  C_rows_temp.block(0,robot_dof_+3,rotors_sz,3) = torque_to_rotor_velocities_.leftCols<3>();
  C_.middleRows(2+arm_dof_,rotors_sz) = C_rows_temp.sparseView();

  d_.head<2>() = controller_parameters_.mu_attitude_*(rpy_min - odometry_.getRPY().head<2>() );
  d_.segment(2,arm_dof_) = controller_parameters_.mu_arm_*(arm_joints_angle_min - joints_state_.angles );

  f_.head<2>() = controller_parameters_.mu_attitude_*(rpy_max - odometry_.getRPY().head<2>() );
  f_.segment(2,arm_dof_) = controller_parameters_.mu_arm_*(arm_joints_angle_max - joints_state_.angles );
}


// Specific to 9-DoF Aerial Delta-Manipulator
void MultiObjectiveController::UpdateDynamicModelTerms() {
  // Compute static indexes vector for fast slicing of vector at run time (using igl lib).
  static VectorXi G_idx = [] {
      VectorXi tmp(7);
      tmp << 3, 4, 6, 7, 8, 9, 10; // roll, pitch, q0, q1, q2, q3, q4
      return tmp;
  }();
  static VectorXi C_idx = [] {
      VectorXi tmp(17);
      tmp << 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19; // roll, pitch, yaw, q0, q1, q2, q3, q4, dq[9]
      return tmp;
  }();

  // Evaluate current Lagrangian coordinates vector q.
  q_eig_.setZero();
  q_eig_ << odometry_.position_W, odometry_.getRPY(), joints_state_.angles;

  // Evaluate current dependent DM joints angle (q3,q4).
  double q_in[2];
  double q34_12[2];
  Vector2d::Map(q_in) = q_eig_.tail<2>(); // q1,q2
  q34_12_fun(q_in,q34_12);
  Map<Vector2d> q34_12_map(q34_12);

  // Stack q_ext = [q q3 q4] and dq together for faster input elements access.
  X_eig_.setZero();
  X_eig_ << q_eig_, q34_12_map, robot_vel_;

  // Evaluate current Coriolis matrix C.
  double C_[robot_dof_*robot_dof_];
  double q_in_C[C_idx.rows()];
  VectorXd q_in_eig(C_idx.rows());
  igl::slice(X_eig_, C_idx, q_in_eig);
  VectorXd::Map(q_in_C, q_in_eig.rows()) = q_in_eig;
  C_fun(q_in_C,C_);
  Map<MatrixXd> C_eig(C_, robot_dof_, robot_dof_);

  // Evaluate current inertia matrix M.
  double M_triu_[robot_dof_*robot_dof_];
  double q_in_M[5+arm_dof_];
  VectorXd::Map(q_in_M, 5+arm_dof_) = q_in_eig.head(5+arm_dof_);
  M_triu_fun(q_in_M,M_triu_);
  Map<MatrixXd> M_eig(M_triu_, robot_dof_, robot_dof_);
  M_eig.triangularView<StrictlyLower>() = M_eig.transpose();

  // Evaluate current gravity vector G.
  double G_[robot_dof_];
  double q_in_G[G_idx.size()];
  q_in_eig.resize(G_idx.size());
  igl::slice(X_eig_, G_idx, q_in_eig);
  VectorXd::Map(q_in_G, q_in_eig.rows()) = q_in_eig;
  G_fun(q_in_G,G_);
  Map<VectorXd> G_eig(G_, robot_dof_);

  // Evaluate current damping matrix D (actually constant).
  MatrixXd D_eig = MatrixXd::Identity(robot_dof_, robot_dof_) * 0.001;
  D_eig.bottomRightCorner(arm_dof_, arm_dof_) = Matrix3d::Identity() * 0.1;

  // Update dynamic terms structure.
  dyn_mdl_terms_.setAll(M_eig,C_eig,D_eig,G_eig);
}


// Specific to 9-DoF Aerial Delta-Manipulator
void MultiObjectiveController::UpdateEndEffectorState() {
  // Compute static indexes vector for fast slicing of vector at run time (using igl lib).
  static VectorXi p_ee_idx = [] {
      VectorXi tmp(9);
      tmp << 0, 1, 2, 3, 4, 5, 6, 7, 9; // x, y, z, roll, pitch, yaw, q0, q1, q3
      return tmp;
  }();
  static VectorXi J_e_dot_idx = [] {
      VectorXi tmp(14);
      tmp << 3, 4, 5, 6, 7, 8, 9, 10, 14, 15, 16, 17, 18, 19; // roll, pitch, yaw, q0, q1, q2, q3, q4,
      return tmp;
  }();

//  // Evaluate current Lagrangian coordinates vector q.
//  VectorXd q_eig(robot_dof_);
//  q_eig << odometry_.position_W, odometry_.getRPY(), joints_state_.angles;
//
//  // Evaluate current dependent DM joints angle (q3,q4).
//  double q_in[2];
//  double q34_12[2];
//  Vector2d::Map(q_in) = q_eig.tail<2>(); // q1,q2
//  q34_12_fun(q_in,q34_12);
//  Map<Vector2d> q34_12_map(q34_12);
//
//  // Stack q_ext = [q q3 q4] and dq together for faster input elements access.
//  VectorXd X_eig(2*robot_dof_+2);
//  X_eig << q_eig, q34_12_map, robot_vel_;

  // Evaluate end-effector Jacobian.
  double J_e[6*robot_dof_];
  double q_in_J_e[8];
  VectorXd::Map(q_in_J_e, 8) = X_eig_.segment<8>(3);
  J_e_fun(q_in_J_e,J_e);
  Map<MatrixXd> J_e_eig(J_e, 6, robot_dof_);

  // Evaluate end-effector Jacobian first time derivative.
  double J_e_dot[6*robot_dof_];
  double q_in_J_e_dot[J_e_dot_idx.size()];
  VectorXd q_in_eig(J_e_dot_idx.size());
  igl::slice(X_eig_, J_e_dot_idx, q_in_eig);
  VectorXd::Map(q_in_J_e_dot, J_e_dot_idx.size()) = q_in_eig;
  J_e_dot_fun(q_in_J_e_dot,J_e_dot);
  Map<MatrixXd> J_e_dot_eig(J_e_dot, 6, robot_dof_);

  // Evaluate end-effector global 3D position.
  double p_ee[3];
  double q_in_p_ee[p_ee_idx.size()];
  q_in_eig.resize(p_ee_idx.size());
  igl::slice(X_eig_, p_ee_idx, q_in_eig);
  VectorXd::Map(q_in_p_ee, p_ee_idx.size()) = q_in_eig;
  p_ee_fun(q_in_p_ee,p_ee);
  Map<VectorXd> p_ee_eig(p_ee, 3);

  // Update end-effector related structure.
  end_effector_.setPosJac(p_ee_eig, J_e_eig, J_e_dot_eig);
}


void MultiObjectiveController::ComputeCostFunction() {

  // Initialize cost function terms.
  MatrixXd Q(minimizer_sz_,minimizer_sz_);
  VectorXd c(minimizer_sz_);
  Q_.setZero();
  c_.setZero();

  // Normalize weighting factors vector and check for consistency (and possibly fix it).
  VectorXd current_weights = controller_parameters_.objectives_weight_;
  if (!mav_trajectory_received_) {
    // In case no reference for UAV pose is given, set "zero velocity" mode.
    current_weights << 0, 0, 0, 1, controller_parameters_.objectives_weight_.tail<3>();
  }
  if ((!arm_trajectory_received_ and controller_parameters_.objectives_weight_(4) > 0) or
     (!ee_trajectory_received_ and controller_parameters_.objectives_weight_(5) > 0)) {
    // In case no reference for DM configuration is given, set "frozen arm" mode.
    current_weights << controller_parameters_.objectives_weight_.head<4>(), 0, 0, 1;
  }
  current_weights.normalize();

  /* Iterate over weights vector, compute objective function for active elements and add
   * them together according to their relative importance factor.
   */
  for (unsigned int i=0; i<current_weights.size(); i++) {
    if (current_weights(i) == 0)
      continue;

    // Evaluate quadratic cost function terms for active objective.
    switch (i) {
      case (0):
        GetFlatStateSetPtObjective(&Q,&c);
        break;

      case (1):
        GetPoseSetPtObjective(&Q,&c);
        break;

      case (2):
        GetYawSetPtObjective(&Q,&c);
        break;

      case (3):
        GetZeroVelSetPtObjective(&Q,&c);
        break;

      case (4):
        GetManipulatorSetPtObjective(&Q,&c);
        break;

      case (5):
        GetEndEffectorSetPtObjective(&Q,&c);
        break;

      case (6):
        GetFrozenArmSetPtObjective(&Q,&c);
        break;

      default:
        Q.setZero();
        c.setZero();
    }

    // Add them in weighted sum.
    SpMatrixXd Q_temp = Q.sparseView();
    Q_ += current_weights(i) * Q_temp;
    c_ += current_weights(i) * c;
  }
}

/*
 **************************************************************************
 * Template to compute quadratic terms for a set-point objective:
 * servo the task 'g' around a given reference value 'g_ref'.
 **************************************************************************
*/
void MultiObjectiveController::GetSetPointObjective(const VectorXd& g, const VectorXd& g_ref,
                                                    const VectorXd& kp, const VectorXd& kv,
                                                    const MatrixXd& Jg, const MatrixXd& Jg_dot,
                                                    const VectorXd& q_dot, MatrixXd* Q,
                                                    VectorXd* c) const {
  // Quadratic term.
  *Q = MatrixXd::Zero(minimizer_sz_,minimizer_sz_);
  (*Q).topLeftCorner(robot_dof_,robot_dof_) = Jg.transpose() * Jg;

  // Linear term.
  *c = VectorXd::Zero(minimizer_sz_);
  (*c).head(robot_dof_) = -Jg.transpose()*(kp.cwiseProduct(g_ref-g) - kv.cwiseProduct(Jg*q_dot) - Jg_dot*q_dot);
}

/*************************************************************************/
/******  OBJECTIVE FUNCTIONs  ********************************************/
/*************************************************************************/

void MultiObjectiveController::GetFlatStateSetPtObjective(MatrixXd* _Q, VectorXd* _c) const {
// Provides control on both UAV CoG global 3D position and heading in terms of yaw angle.
  assert(mav_trajectory_received_);

  MatrixXd Q_pos;
  MatrixXd Q_orient;
  VectorXd c_pos;
  VectorXd c_orient;

  GetPositionSetPtObjective(command_trajectory_.position_W, &Q_pos, &c_pos);

  // Roll and Pitch angle automatically set to zero to force hovering mode.
  Vector3d rpy_des(0,0,command_trajectory_.getYaw());

  GetOrientationSetPtObjective(rpy_des, &Q_orient, &c_orient);

  *_Q = Q_pos + Q_orient;
  *_c = c_pos + c_orient;
}


void MultiObjectiveController::GetPoseSetPtObjective(MatrixXd* _Q, VectorXd* _c) const {
/* Provides control on UAV CoG global 3D position only. Heading is maintained
 * stable by velocity regulation.
 */
  assert(mav_trajectory_received_);

  MatrixXd Q_pos;
  MatrixXd Q_orient;
  VectorXd c_pos;
  VectorXd c_orient;

  GetPositionSetPtObjective(command_trajectory_.position_W, &Q_pos, &c_pos);

  /* Roll and Pitch angle automatically set to zero to force hovering mode.
   * Desired yaw equal to current yaw value (zero error).
   */
  Vector3d rpy_des(0,0,odometry_.getYaw());

  /* In absence of a target heading, reference value for the yaw angle is automatically
   * computed such that the UAV turns towards the target position of the end-effector
   * to facilitate its reachability, if e.e. objective is active.
   */
  if (ee_trajectory_received_ and (controller_parameters_.objectives_weight_(5)>0) )
    rpy_des.z() = GetYawFromEndEffector();

  GetOrientationSetPtObjective(rpy_des, &Q_orient, &c_orient);

  *_Q = Q_pos + Q_orient;
  *_c = c_pos + c_orient;
}


void MultiObjectiveController::GetYawSetPtObjective(MatrixXd* _Q, VectorXd* _c) const {
/* Provides heading control in terms of yaw angle only. Position is maintained stable
 * by velocity regulation.
 */
  assert(mav_trajectory_received_);

  MatrixXd Q_pos;
  MatrixXd Q_orient;
  VectorXd c_pos;
  VectorXd c_orient;

  // Desired position equal to current position (zero error).
  GetPositionSetPtObjective(odometry_.position_W, &Q_pos, &c_pos);

  // Roll and Pitch angle automatically set to zero to force hovering mode.
  Vector3d rpy_des(0,0,command_trajectory_.getYaw());

  GetOrientationSetPtObjective(rpy_des, &Q_orient, &c_orient);

  *_Q = Q_pos + Q_orient;
  *_c = c_pos + c_orient;
}


void MultiObjectiveController::GetZeroVelSetPtObjective(MatrixXd* _Q, VectorXd* _c) const {
// Velocity regulation on whole UAV flat-state.

  MatrixXd Q_pos;
  MatrixXd Q_orient;
  VectorXd c_pos;
  VectorXd c_orient;

  // Desired position equal to current position (zero error).
  GetPositionSetPtObjective(odometry_.position_W, &Q_pos, &c_pos);

  /* Roll and Pitch angle automatically set to zero to force hovering mode.
   * Desired yaw equal to current yaw value (zero error).
   */
  Vector3d rpy_des(0,0,odometry_.getYaw());

  /* In absence of a target heading, reference value for the yaw angle is automatically
   * computed such that the UAV turns towards the target position of the end-effector
   * to facilitate its reachability, if e.e. objective is active.
   */
  if (ee_trajectory_received_ and (controller_parameters_.objectives_weight_(5)>0) )
    rpy_des.z() = GetYawFromEndEffector();

  GetOrientationSetPtObjective(rpy_des, &Q_orient, &c_orient);

  *_Q = Q_pos + Q_orient;
  *_c = c_pos + c_orient;
}


void MultiObjectiveController::GetManipulatorSetPtObjective(MatrixXd* _Q, VectorXd* _c) const {
// Provides direct control on DM joints angle q_r = ( q0, q1, q2 )'.
  assert(arm_trajectory_received_);

  static MatrixXd Jac_ = [&] {
      MatrixXd tmp(arm_dof_,robot_dof_);
      tmp.setZero();
      tmp.topRightCorner(arm_dof_,arm_dof_) = MatrixXd::Identity(arm_dof_,arm_dof_);
      return tmp;
  }();
  static MatrixXd Jac_dot_ = MatrixXd::Zero(arm_dof_,robot_dof_);

  GetSetPointObjective(joints_state_.angles, joints_angle_des_, controller_parameters_.arm_joints_angle_gain_,
                       controller_parameters_.arm_joints_ang_rate_gain_, Jac_, Jac_dot_, robot_vel_, _Q, _c);
}


void MultiObjectiveController::GetEndEffectorSetPtObjective(MatrixXd* _Q, VectorXd* _c) {
// Provides control on the end-effector global 3D position.
  assert(ee_trajectory_received_);

  // We update end-effector position and jacobian only when objective is active here.
  UpdateEndEffectorState();

  GetSetPointObjective(end_effector_.odometry.position, command_trajectory_ee_.position_W, controller_parameters_.ee_position_gain_,
                       controller_parameters_.ee_velocity_gain_, end_effector_.jacobian_W.topRows(3), end_effector_.jacobian_dot_W.topRows(3), robot_vel_, _Q, _c);
}


void MultiObjectiveController::GetFrozenArmSetPtObjective(MatrixXd* _Q, VectorXd* _c) const {
// Maintain latest manipulator configuration.

  static MatrixXd Jac_ = [&] {
      MatrixXd tmp(arm_dof_,robot_dof_);
      tmp.setZero();
      tmp.topRightCorner(arm_dof_,arm_dof_) = MatrixXd::Identity(arm_dof_,arm_dof_);
      return tmp;
  }();
  static MatrixXd Jac_dot_ = MatrixXd::Zero(arm_dof_,robot_dof_);

  GetSetPointObjective(joints_state_.angles, frozen_joints_angle_des_, controller_parameters_.arm_joints_angle_gain_,
                       controller_parameters_.arm_joints_ang_rate_gain_, Jac_, Jac_dot_, robot_vel_, _Q, _c);
}

/*************************************************************************/
/******  UAV RELATED SUB-OBJECTIVE FUNCTIONs  ****************************/
/*************************************************************************/

void MultiObjectiveController::GetPositionSetPtObjective(const Vector3d& _position_ref, MatrixXd* _Q, VectorXd* _c) const {
  static Matrix3Xd Jac_ = [&] {
      Matrix3Xd tmp(3,robot_dof_);
      tmp.setZero();
      tmp.topLeftCorner(3,3) = Matrix3d::Identity();
      return tmp;
  }();
  static Matrix3Xd Jac_dot_ = Matrix3Xd::Zero(3,robot_dof_);

  GetSetPointObjective(odometry_.position_W, _position_ref, controller_parameters_.mav_position_gain_,
                       controller_parameters_.mav_velocity_gain_, Jac_, Jac_dot_, robot_vel_, _Q, _c);
}


void MultiObjectiveController::GetOrientationSetPtObjective(const Vector3d& _orientation_ref, MatrixXd* _Q, VectorXd* _c) const {
  static Matrix3Xd Jac_ = [&] {
      Matrix3Xd tmp(3,robot_dof_);
      tmp.setZero();
      tmp.block<3,3>(0,3) = Matrix3d::Identity();
      return tmp;
  }();
  static Matrix3Xd Jac_dot_ = Matrix3Xd::Zero(3,robot_dof_);

  Vector3d rpy = odometry_.getRPY();

  GetSetPointObjective(rpy, _orientation_ref, controller_parameters_.mav_attitude_gain_,
                       controller_parameters_.mav_angular_rate_gain_, Jac_, Jac_dot_, robot_vel_, _Q, _c);
}

/*************************************************************************/
/******  ANCILLARY METHODs  ****************************/
/*************************************************************************/

void MultiObjectiveController::UpdateRobotVelocities() {
  // Transform velocities to world frame.
  Eigen::Vector3d velocity_W =  odometry_.getVelocityWorld();
  Eigen::Vector3d ang_velocity_W =  AngVelBody2World(odometry_.angular_velocity_B);

  robot_vel_.head(3) = velocity_W;
  robot_vel_.segment<3>(3) = ang_velocity_W;
  robot_vel_.tail(arm_dof_) = joints_state_.angular_rates;
}


double MultiObjectiveController::GetYawFromEndEffector() const {
/* Compute yaw angle such that the UAV turns towards the target position of the
 * end-effector to facilitate its reachability.
 */
  assert(ee_trajectory_received_);

  Vector3d p_ee_V = odometry_.orientation_W_B.inverse() * (command_trajectory_ee_.position_W - odometry_.position_W);
  double ro = p_ee_V.norm();

  return asin(p_ee_V.y()/(ro*sqrt(1-pow((p_ee_V.z()/ro),2)))) + odometry_.getYaw();
}


Vector3d MultiObjectiveController::AngVelBody2World(const Vector3d& angular_rates) const {
  Vector3d rpy = odometry_.getRPY();
  Matrix3d Jv_inv;
  Jv_inv << 1, sin(rpy.x())*tan(rpy.y()), cos(rpy.x())*tan(rpy.y()),
            0, cos(rpy.x()) , -sin(rpy.x()),
            0, sin(rpy.x())/cos(rpy.y()), cos(rpy.x())/cos(rpy.y());

  return Jv_inv*angular_rates;
}

}

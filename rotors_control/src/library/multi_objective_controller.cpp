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


#define PRECISION 7 //StreamPrecision FullPrecision

using namespace Eigen;

IOFormat MatlabMatrixFmt(PRECISION, 0, ", ", ";\n", "", "", "[", "]");
IOFormat MatlabVectorFmt(PRECISION, DontAlignCols, ", ", "; ", "", "", "[ ", " ]");
IOFormat MatlabVector2Fmt(PRECISION, DontAlignCols, ", ", "; ", "", "", " ", " ");

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
  size_t rotors_sz = vehicle_parameters_.rotor_configuration_.rotors.size();
  size_t ineq_no = 3+arm_dof_*2+rotors_sz;
  torque_to_rotor_velocities_.resize(rotors_sz, 4);
  torque_to_rotor_velocities_ = pseudoInv(controller_parameters_.allocation_matrix_);

  // Initialize linear constraints
  // Solve min 1/2 x' Q x + c' x, such that A x = b, d <= Cx <= f.
  Q_.resize(minimizer_sz_,minimizer_sz_);
  Q_.setZero();
  c_ = VectorXd::Zero(minimizer_sz_);
  A_.resize(robot_dof_+2,minimizer_sz_);
  A_.setZero();
  b_ = VectorXd::Zero(robot_dof_+2);
  C_.resize(ineq_no, minimizer_sz_);
  C_.setZero();
  MatrixXd C_temp = MatrixXd::Zero(ineq_no,minimizer_sz_);
  C_temp.block(0,3,3+arm_dof_,3+arm_dof_) = MatrixXd::Identity(3+arm_dof_,3+arm_dof_);
  C_temp.bottomRightCorner(arm_dof_,arm_dof_) = MatrixXd::Identity(arm_dof_,arm_dof_);
  C_ = C_temp.sparseView();
  d_.resize(ineq_no);
  d_.setZero();
  d_.segment(3+arm_dof_,rotors_sz) = VectorXd::Zero(rotors_sz);
  d_.tail(arm_dof_) = VectorXd::Constant(arm_dof_,controller_parameters_.arm_joint_torque_lim_.x());
  f_.resize(ineq_no);
  f_.setZero();
  f_.segment(3+arm_dof_,rotors_sz) = VectorXd::Constant(rotors_sz,pow(controller_parameters_.max_rot_velocity_,2));
  f_.tail(arm_dof_) = VectorXd::Constant(arm_dof_,controller_parameters_.arm_joint_torque_lim_.y());
  x_ = VectorXd::Zero(minimizer_sz_);

  initialized_params_ = true;

  //debug
//  std::cout << "to_rotor_vel_ = " << torque_to_rotor_velocities_.format(MatlabMatrixFmt) << std::endl << std::endl;
//  std::cout << robot_dof_ << std::endl;
//  std::cout << "A_ : " << A_.size() << " = " << A_.rows() << "x" << A_.cols() << "\n" << A_ << std::endl;
//  std::cout << "b_ : " << b_.size() << " = " << b_.rows() << "x" << b_.cols() << std::endl;
//  std::cout << "C_ : " << C_.size() << " = " << C_.rows() << "x" << C_.cols() << std::endl;
//  std::cout << "d_ : " << d_.size() << " = " << d_.rows() << "x" << d_.cols() << std::endl;
//  std::cout << "f_ : " << f_.size() << " = " << f_.rows() << "x" << f_.cols() << std::endl;
//  std::cout << "A_ = " << A_.toDense().format(MatlabMatrixFmt) << std::endl;
//  std::cout << "b_ = " << b_.format(MatlabVectorFmt) << std::endl;
//  std::cout << "C_ = " << C_.toDense().format(MatlabMatrixFmt) << std::endl;
//  std::cout << "d_ = " << d_.format(MatlabVectorFmt) << std::endl;
//  std::cout << "f_ = " << f_.format(MatlabVectorFmt) << std::endl;
}


void MultiObjectiveController::CalculateControlInputs(VectorXd* rotor_velocities, Vector3d* torques) {
  assert(rotor_velocities);
  assert(torques);
  assert(initialized_params_);

  rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());
  // Return 0 velocities on all rotors, until the first command is received.
  if (!controller_active_) {
    *rotor_velocities = VectorXd::Zero(rotor_velocities->rows());
    return;
  }

  SolveMultiObjectiveOptimization();
//  ROS_INFO("CHECK POINT after SolveMultiObjectiveOptimization");  //debug

  // Extract thrust force from aerodynamic forces given in world frame and current UAV orientation
  double thrust = (odometry_.orientation_W_B.inverse() * x_.segment<3>(robot_dof_)).tail<1>()(0);
  Vector4d torque_thrust;
  torque_thrust << x_.segment<3>(robot_dof_+3), thrust;

  *rotor_velocities = torque_to_rotor_velocities_ * torque_thrust;
  *rotor_velocities = rotor_velocities->cwiseMax(VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();

  *torques = x_.tail(arm_dof_);
  //  torques->setZero();   //dummy ouput

  //debug
//  std::cout << "torques to rotor velocities :\n" << torque_to_rotor_velocities_ << std::endl;
//  std::cout << "Rotors velocities after saturation :\t" << rotor_velocities->transpose() << std::endl;
//  std::cout << "Global force vector :\t" << x_.segment<3>(robot_dof_).transpose() << std::endl;
//  std::cout << "Aero torques & thrust:\t" << torque_thrust.transpose() << std::endl;
//  std::cout << "Joint torques :\t" << torques->transpose() << std::endl;
}


void MultiObjectiveController::SetOdometry(const mav_msgs::EigenOdometry& odometry) {
  odometry_ = odometry;

  //debug
//  std::cout << "UAV position :\t" << odometry_.position_W.transpose() << std::endl;
//  std::cout << "UAV orientation (quaternion) :\t" << odometry_.orientation_W_B.coeffs().transpose() << std::endl;
//  std::cout << "UAV orientation (RPY) [degrees] :\t" << odometry_.getRPY().transpose() * 180/M_PI << std::endl;
//  std::cout << "UAV orientation (rotation matrix) :\n" << odometry_.orientation_W_B.toRotationMatrix() << std::endl;
}


void MultiObjectiveController::SetArmJointsState(const manipulator_msgs::EigenJointsState& joints_state) {
  joints_state_ = joints_state;

  //debug
//  std::cout << "Joints state position vector [degrees] :\t" << joints_state_.angles.transpose() * 180/M_PI << std::endl;
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


void MultiObjectiveController::SetExternalForces(const Vector3d& forces) {
  //Todo : ignore small forces (just noise)
  ext_forces_ = forces;
}

/////////////////////// PRIVATE METHODs //////////////////////

void MultiObjectiveController::SolveMultiObjectiveOptimization() {
  MatrixXd Q(minimizer_sz_,minimizer_sz_);
  VectorXd c(minimizer_sz_);

  // Reset
  Q_.setZero();
  c_.setZero();
  x_.setZero();

  UpdateLinearConstraints();

  VectorXd current_weights = controller_parameters_.objectives_weight_;
  if (!mav_trajectory_received_) {
    current_weights << 0, 0, 0, 2, controller_parameters_.objectives_weight_.tail<3>();
  }
  if (!(arm_trajectory_received_ or ee_trajectory_received_)) {
    current_weights << controller_parameters_.objectives_weight_.head<4>(), 0, 0, 1;
  }
  current_weights.normalize();

  // debug
//  std::cout << "current_weights = " << current_weights.format(MatlabVectorFmt) << std::endl;

  for (unsigned int i=0; i<current_weights.size(); i++) {
    if (current_weights(i) == 0)
      continue;

    switch (i) {
      case (0):
        GetAttitudeSetPtObjective(&Q,&c);
//        ROS_INFO_THROTTLE(1, "CHECK POINT after GetAttitudeSetPtObjective");  //debug
        break;

      case (1):
        GetPoseSetPtObjective(&Q,&c);
//        ROS_INFO_THROTTLE(1, "CHECK POINT after GetPoseSetPtObjective");  //debug
        break;

      case (2):
        GetYawSetPtObjective(&Q,&c);
//        ROS_INFO_THROTTLE(1, "CHECK POINT after GetYawSetPtObjective");  //debug
        break;

      case (3):
        GetFreeHoverSetPtObjective(&Q,&c);
//        ROS_INFO_THROTTLE(1, "CHECK POINT after GetFreeHoverSetPtObjective");  //debug
        break;

      case (4):
        if (!arm_trajectory_received_) {
          current_weights << controller_parameters_.objectives_weight_.head<4>(), 0, 0, controller_parameters_.objectives_weight_.tail<3>().maxCoeff();
          continue;
        }
        GetManipulatorSetPtObjective(&Q,&c);
//        ROS_INFO_THROTTLE(1, "CHECK POINT after GetManipulatorSetPtObjective");  //debug
        break;

      case (5):
        if (!ee_trajectory_received_) {
          current_weights << controller_parameters_.objectives_weight_.head<4>(), 0, 0, controller_parameters_.objectives_weight_.tail<3>().maxCoeff();
          continue;
        }
        GetEndEffectorSetPtObjective(&Q,&c);
//        ROS_INFO_THROTTLE(1, "CHECK POINT after GetEndEffectorSetPtObjective");  //debug
        break;

      case (6):
        GetDeadArmSetPtObjective(&Q,&c);
//        ROS_INFO_THROTTLE(1, "CHECK POINT after GetDeadArmSetPtObjective");  //debug
        break;

      default:
        Q.setZero();
        c.setZero();
    }

    SpMatrixXd Q_temp = Q.sparseView();
    Q_ += current_weights(i) * Q_temp;
    c_ += current_weights(i) * c;
  }

  for (unsigned int i = 0; i<6; i++) {
    if (f_(i)<d_(i)) {
      ROS_WARN_THROTTLE(1, "[multi_objective_controller] Constraints on %i-th inequality are inconsistent.", i);
    }
  }

  if (!ooqpei::OoqpEigenInterface::solve(Q_, c_, A_, b_, C_, d_, f_, x_)) {
    ROS_WARN_THROTTLE(1,"[multi_objective_controller] Optimization failed.");
//    return;
  }

  //debug
//  std::cout << "weights = " << current_weights.format(MatlabVectorFmt) << std::endl;
//  std::cout << "gq_ref = [ " << command_trajectory_.position_W.format(MatlabVector2Fmt) << "; 0 ; 0 ;" << command_trajectory_.getYaw() << ";"
//            << joints_angle_des_.format(MatlabVector2Fmt) << " ]" << std::endl;
//  std::cout << "Q_ = " << Q_.toDense().format(MatlabMatrixFmt) << std::endl;
//  std::cout << "c_ = " << c_.format(MatlabVectorFmt) << std::endl;
//  std::cout << "x_ = " << x_.format(MatlabVectorFmt) << std::endl;
}


/*
 * Template to compute quadratic terms for a set-point objective:
 * servo the task 'g' around a given reference value 'g_ref'.
*/
void MultiObjectiveController::GetSetPointObjective(const VectorXd& g, const VectorXd& g_ref,
                                                    const VectorXd& kp, const VectorXd& kv,
                                                    const MatrixXd& Jg, const MatrixXd& Jg_dot,
                                                    const VectorXd& q_dot, MatrixXd* Q,
                                                    VectorXd* c) const {

  *Q = MatrixXd::Zero(minimizer_sz_,minimizer_sz_);
  (*Q).topLeftCorner(robot_dof_,robot_dof_) = Jg.transpose() * Jg;

  *c = VectorXd::Zero(minimizer_sz_);
  (*c).head(robot_dof_) = -Jg.transpose()*(kp.cwiseProduct(g_ref-g) - kv.cwiseProduct(Jg*q_dot) - Jg_dot*q_dot);
}


void MultiObjectiveController::GetAttitudeSetPtObjective(MatrixXd* _Q, VectorXd* _c) const {

  assert(mav_trajectory_received_);

  MatrixXd Q_pos;
  MatrixXd Q_orient;
  VectorXd c_pos;
  VectorXd c_orient;

  GetPositionSetPtObjective(command_trajectory_.position_W, &Q_pos, &c_pos);

  Vector3d rpy_des(0,0,command_trajectory_.getYaw());
  GetOrientationSetPtObjective(rpy_des, &Q_orient, &c_orient);

  *_Q = Q_pos + Q_orient;
  *_c = c_pos + c_orient;
}


void MultiObjectiveController::GetPoseSetPtObjective(MatrixXd* _Q, VectorXd* _c) const {

  assert(mav_trajectory_received_);

  MatrixXd Q_pos;
  MatrixXd Q_orient;
  VectorXd c_pos;
  VectorXd c_orient;

  GetPositionSetPtObjective(command_trajectory_.position_W, &Q_pos, &c_pos);

  Vector3d rpy_des(0,0,odometry_.getYaw());
  if (ee_trajectory_received_ and (controller_parameters_.objectives_weight_(5)>0) )
    rpy_des.z() = GetYawFromEndEffector();
  GetOrientationSetPtObjective(rpy_des, &Q_orient, &c_orient);

  *_Q = Q_pos + Q_orient;
  *_c = c_pos + c_orient;
}


void MultiObjectiveController::GetYawSetPtObjective(MatrixXd* _Q, VectorXd* _c) const {

  assert(mav_trajectory_received_);

  MatrixXd Q_pos;
  MatrixXd Q_orient;
  VectorXd c_pos;
  VectorXd c_orient;

  GetPositionSetPtObjective(odometry_.position_W, &Q_pos, &c_pos);

  Vector3d rpy_des(0,0,command_trajectory_.getYaw());
  GetOrientationSetPtObjective(rpy_des, &Q_orient, &c_orient);

  *_Q = Q_pos + Q_orient;
  *_c = c_pos + c_orient;
}


void MultiObjectiveController::GetFreeHoverSetPtObjective(MatrixXd* _Q, VectorXd* _c) const {
  MatrixXd Q_pos;
  MatrixXd Q_orient;
  VectorXd c_pos;
  VectorXd c_orient;

  GetPositionSetPtObjective(odometry_.position_W, &Q_pos, &c_pos);

  Vector3d rpy_des(0,0,odometry_.getYaw());
  if (ee_trajectory_received_ and (controller_parameters_.objectives_weight_(5)>0) )
    rpy_des.z() = GetYawFromEndEffector();
  GetOrientationSetPtObjective(rpy_des, &Q_orient, &c_orient);

  *_Q = Q_pos + Q_orient;
  *_c = c_pos + c_orient;
}


void MultiObjectiveController::GetPositionSetPtObjective(const Vector3d& _position_ref, MatrixXd* _Q, VectorXd* _c) const {
  static Matrix3Xd Jac_ = [&] {
      Matrix3Xd tmp(3,robot_dof_);
      tmp.setZero();
      tmp.topLeftCorner(3,3) = Matrix3d::Identity();
      return tmp;
  }();
  static Matrix3Xd Jac_dot_ = Matrix3Xd::Zero(3,robot_dof_);

  VectorXd robot_vel = GetRobotVelocities();

  GetSetPointObjective(odometry_.position_W, _position_ref, controller_parameters_.mav_position_gain_,
                       controller_parameters_.mav_velocity_gain_, Jac_, Jac_dot_, robot_vel, _Q, _c);
}


void MultiObjectiveController::GetOrientationSetPtObjective(const Vector3d& _orientation_ref, MatrixXd* _Q, VectorXd* _c) const {
  static Matrix3Xd Jac_ = [&] {
      Matrix3Xd tmp(3,robot_dof_);
      tmp.setZero();
      tmp.block<3,3>(0,3) = Matrix3d::Identity();
      return tmp;
  }();
  static Matrix3Xd Jac_dot_ = Matrix3Xd::Zero(3,robot_dof_);

  VectorXd robot_vel = GetRobotVelocities();

  Vector3d rpy = odometry_.getRPY();

  GetSetPointObjective(rpy, _orientation_ref, controller_parameters_.mav_attitude_gain_,
                       controller_parameters_.mav_angular_rate_gain_, Jac_, Jac_dot_, robot_vel, _Q, _c);
}


void MultiObjectiveController::GetManipulatorSetPtObjective(MatrixXd* _Q, VectorXd* _c) const {
  static MatrixXd Jac_ = [&] {
      MatrixXd tmp(arm_dof_,robot_dof_);
      tmp.setZero();
      tmp.topRightCorner(arm_dof_,arm_dof_) = MatrixXd::Identity(arm_dof_,arm_dof_);
      return tmp;
  }();
  static MatrixXd Jac_dot_ = MatrixXd::Zero(arm_dof_,robot_dof_);

  assert(arm_trajectory_received_);

  Jac_.topRightCorner(arm_dof_,arm_dof_) = MatrixXd::Identity(arm_dof_,arm_dof_);

  VectorXd robot_vel = GetRobotVelocities();

  GetSetPointObjective(joints_state_.angles, joints_angle_des_, controller_parameters_.arm_joints_angle_gain_,
                       controller_parameters_.arm_joints_ang_rate_gain_, Jac_, Jac_dot_, robot_vel, _Q, _c);
}


void MultiObjectiveController::GetDeadArmSetPtObjective(MatrixXd* _Q, VectorXd* _c) const {
  static MatrixXd Jac_ = [&] {
      MatrixXd tmp(arm_dof_,robot_dof_);
      tmp.setZero();
      tmp.topRightCorner(arm_dof_,arm_dof_) = MatrixXd::Identity(arm_dof_,arm_dof_);
      return tmp;
  }();
  static MatrixXd Jac_dot_ = MatrixXd::Zero(arm_dof_,robot_dof_);

  VectorXd robot_vel = GetRobotVelocities();

  GetSetPointObjective(joints_state_.angles, joints_state_.angles, controller_parameters_.arm_joints_angle_gain_,
                       controller_parameters_.arm_joints_ang_rate_gain_, Jac_, Jac_dot_, robot_vel, _Q, _c);
}


void MultiObjectiveController::GetEndEffectorSetPtObjective(MatrixXd* _Q, VectorXd* _c) {

  assert(ee_trajectory_received_);

  UpdateEndEffectorState();

  VectorXd robot_vel = GetRobotVelocities();

  GetSetPointObjective(end_effector_.odometry.position, command_trajectory_ee_.position_W, controller_parameters_.ee_position_gain_,
                       controller_parameters_.ee_velocity_gain_, end_effector_.jacobian_W.topRows(3), end_effector_.jacobian_dot_W.topRows(3), robot_vel, _Q, _c);
}


VectorXd MultiObjectiveController::GetRobotVelocities() const {
  // Transform velocities to world frame.
  Eigen::Vector3d velocity_W =  odometry_.getVelocityWorld();
  Eigen::Vector3d ang_velocity_W =  AngVelBody2World(odometry_.angular_velocity_B);

  VectorXd robot_vel(robot_dof_);

  robot_vel.head(3) = velocity_W;
  robot_vel.segment<3>(3) = ang_velocity_W;
  robot_vel.tail(arm_dof_) = joints_state_.angular_rates;

  return robot_vel;
}


double MultiObjectiveController::GetYawFromEndEffector() const {

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


void MultiObjectiveController::UpdateLinearConstraints() {
  static Vector3d rpy_max = controller_parameters_.safe_range_rpy_*
                            (controller_parameters_.rpy_max_ - controller_parameters_.rpy_min_) +
                            controller_parameters_.rpy_min_;
  static Vector3d rpy_min = controller_parameters_.safe_range_rpy_*
                            (controller_parameters_.rpy_min_ - controller_parameters_.rpy_max_) +
                            controller_parameters_.rpy_max_;
  static Vector3d arm_joints_angle_max = controller_parameters_.safe_range_joints_*
                            (controller_parameters_.arm_joints_angle_max_ - controller_parameters_.arm_joints_angle_min_) +
                            controller_parameters_.arm_joints_angle_min_;
  static Vector3d arm_joints_angle_min = controller_parameters_.safe_range_joints_*
                            (controller_parameters_.arm_joints_angle_min_ - controller_parameters_.arm_joints_angle_max_) +
                            controller_parameters_.arm_joints_angle_max_;
  static size_t rotors_sz = vehicle_parameters_.rotor_configuration_.rotors.size();

  Matrix3d Rot_w2v = odometry_.orientation_W_B.toRotationMatrix().transpose();

  UpdateDynamicModelTerms();

  MatrixXd A_temp(robot_dof_+2,minimizer_sz_);
  A_temp.setZero();
  A_temp.topLeftCorner(robot_dof_,robot_dof_) = dyn_mdl_terms_.inertia_matrix;
  A_temp.topRightCorner(robot_dof_,robot_dof_) = -MatrixXd::Identity(robot_dof_,robot_dof_);
  A_temp.block<2,3>(robot_dof_,robot_dof_) = Rot_w2v.topRows(2);
  A_ = A_temp.sparseView();

  b_.head(robot_dof_) = -(dyn_mdl_terms_.coriolis_matrix + dyn_mdl_terms_.damping_matrix)*GetRobotVelocities()
                          - dyn_mdl_terms_.gravity_vector;

  MatrixXd C_temp(rotors_sz,minimizer_sz_);
  C_temp.setZero();
  C_temp.block(0,robot_dof_,rotors_sz,3) = torque_to_rotor_velocities_.rightCols<1>() * Rot_w2v.bottomRows<1>();
  C_temp.block(0,robot_dof_+3,rotors_sz,3) = torque_to_rotor_velocities_.leftCols<3>();
  C_.middleRows(3+arm_dof_,rotors_sz) = C_temp.sparseView();

  d_.head<3>() = controller_parameters_.mu_attitude_*(rpy_min - odometry_.getRPY() );
  d_.segment(3,arm_dof_) = controller_parameters_.mu_arm_*(arm_joints_angle_min - joints_state_.angles );

  f_.head<3>() = controller_parameters_.mu_attitude_*(rpy_max - odometry_.getRPY() );
  f_.segment(3,arm_dof_) = controller_parameters_.mu_arm_*(arm_joints_angle_max - joints_state_.angles );

  //debug
//  std::cout << "rpy_max = " << rpy_max.format(MatlabVectorFmt) << std::endl;
//  std::cout << "rpy_min = " << rpy_min.format(MatlabVectorFmt) << std::endl;
//  std::cout << "arm_joints_angle_max = " << arm_joints_angle_max.format(MatlabVectorFmt) << std::endl;
//  std::cout << "arm_joints_angle_min = " << arm_joints_angle_min.format(MatlabVectorFmt) << std::endl;
//  std::cout << "A_ = " << A_.toDense().format(MatlabMatrixFmt) << std::endl;
//  std::cout << "b_ = " << b_.format(MatlabVectorFmt) << std::endl;
//  std::cout << "C_ = " << C_.toDense().format(MatlabMatrixFmt) << std::endl;
//  std::cout << "d_ = " << d_.format(MatlabVectorFmt) << std::endl;
//  std::cout << "f_ = " << f_.format(MatlabVectorFmt) << std::endl;
}


// Specific to Aerial Delta Manipulator
void MultiObjectiveController::UpdateEndEffectorState() {

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

  VectorXd q_eig(robot_dof_);
  q_eig << odometry_.position_W, odometry_.getRPY(), joints_state_.angles;

  double q_in[2];
  double q34_12[2];
  Vector2d::Map(q_in) = q_eig.tail<2>(); // q1,q2
  q34_12_fun(q_in,q34_12);
  Map<Vector2d> q34_12_map(q34_12);

  VectorXd X_eig(2*robot_dof_+2);
  X_eig << q_eig, q34_12_map, GetRobotVelocities();

  //debug
//  std::cout << "X_eig = " << X_eig.format(MatlabVectorFmt) << std::endl;
//  std::cout << "X = [" << X_eig.head(robot_dof_).format(MatlabVector2Fmt) << ";" << X_eig.tail(robot_dof_).format(MatlabVector2Fmt) << "]" << std::endl;

  double J_e[6*robot_dof_];
  double q_in_J_e[8];
  VectorXd::Map(q_in_J_e, 8) = X_eig.segment<8>(3);
  J_e_fun(q_in_J_e,J_e);
  Map<MatrixXd> J_e_eig(J_e, 6, robot_dof_);

  double J_e_dot[6*robot_dof_];
  double q_in_J_e_dot[J_e_dot_idx.size()];
  VectorXd q_in_eig(J_e_dot_idx.size());
  igl::slice(X_eig, J_e_dot_idx, q_in_eig);
  VectorXd::Map(q_in_J_e_dot, J_e_dot_idx.size()) = q_in_eig;
  J_e_dot_fun(q_in_J_e_dot,J_e_dot);
  Map<MatrixXd> J_e_dot_eig(J_e_dot, 6, robot_dof_);

  double p_ee[3];
  double q_in_p_ee[p_ee_idx.size()];
  q_in_eig.resize(p_ee_idx.size());
  igl::slice(X_eig, p_ee_idx, q_in_eig);
  VectorXd::Map(q_in_p_ee, p_ee_idx.size()) = q_in_eig;
  p_ee_fun(q_in_p_ee,p_ee);
  Map<VectorXd> p_ee_eig(p_ee, 3);

  //debug
//  std::cout << "J_e_ = " << J_e_eig.format(MatlabMatrixFmt) << std::endl;
//  std::cout << "J_e_dot_ = " << J_e_dot_eig.format(MatlabMatrixFmt) << std::endl;
//  std::cout << "p_ee_ = " << p_ee_eig.format(MatlabVectorFmt) << std::endl;

  end_effector_.setPosJac(p_ee_eig, J_e_eig, J_e_dot_eig);
}


// Specific to Aerial Delta Manipulator
void MultiObjectiveController::UpdateDynamicModelTerms() {

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

  VectorXd q_eig(robot_dof_);
  q_eig << odometry_.position_W, odometry_.getRPY(), joints_state_.angles;

  double q_in[2];
  double q34_12[2];
  Vector2d::Map(q_in) = q_eig.tail<2>(); // q1,q2
  q34_12_fun(q_in,q34_12);
  Map<Vector2d> q34_12_map(q34_12);

  VectorXd X_eig(2*robot_dof_+2);
  X_eig << q_eig, q34_12_map, GetRobotVelocities();

  //debug
//  std::cout << "X :\t" << X_eig.format(MatlabVectorFmt) << std::endl;
//  std::cout << "X = [" << X_eig.head(robot_dof_).format(MatlabVector2Fmt) << ";" << X_eig.tail(robot_dof_).format(MatlabVector2Fmt) << "]" << std::endl;

  double C_[robot_dof_*robot_dof_];
  double q_in_C[C_idx.rows()];
  VectorXd q_in_eig(C_idx.rows());
  igl::slice(X_eig, C_idx, q_in_eig);
  VectorXd::Map(q_in_C, q_in_eig.rows()) = q_in_eig;
  C_fun(q_in_C,C_);
  Map<MatrixXd> C_eig(C_, robot_dof_, robot_dof_);

  double M_triu_[robot_dof_*robot_dof_];
  double q_in_M[5+arm_dof_];
  VectorXd::Map(q_in_M, 5+arm_dof_) = q_in_eig.head(5+arm_dof_);
  M_triu_fun(q_in_M,M_triu_);
  Map<MatrixXd> M_eig(M_triu_, robot_dof_, robot_dof_);
  M_eig.triangularView<StrictlyLower>() = M_eig.transpose();

  double G_[robot_dof_];
  double q_in_G[G_idx.size()];
  q_in_eig.resize(G_idx.size());
  igl::slice(X_eig, G_idx, q_in_eig);
  VectorXd::Map(q_in_G, q_in_eig.rows()) = q_in_eig;
  G_fun(q_in_G,G_);
  Map<VectorXd> G_eig(G_, robot_dof_);

  MatrixXd D_eig = MatrixXd::Identity(robot_dof_, robot_dof_) * 0.001;
  D_eig.bottomRightCorner(arm_dof_, arm_dof_) = Matrix3d::Identity() * 0.1;

  dyn_mdl_terms_.setAll(M_eig,C_eig,D_eig,G_eig);
}


}

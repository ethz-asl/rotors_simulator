/*
 * SteadyStateCalculation.cpp
 *
 *  Created on: Dec 4, 2014
 *      Author: mina
 */

#include <rotors_control/SteadyStateCalculation.h>

namespace rotors_control{
SteadyStateCalculation::SteadyStateCalculation() {

}

SteadyStateCalculation::~SteadyStateCalculation() {
}

void SteadyStateCalculation::Initialize() {

  printf("Start initializing the SS calc\n");

  std::string prefix = "MPC_position_controller/controller/";

  std::vector<double> temporary_A, temporary_B, temporary_Bd, temporary_C;

  if (!ros::param::get(prefix + "n_state", state_size_))
    ROS_ERROR("n_state in target state calc is not loaded from ros parameter server");
  if (!ros::param::get(prefix + "n_input", input_size_))
    ROS_ERROR("n_input in target state calc is not loaded from ros parameter server");
  if (!ros::param::get(prefix + "n_measurement", reference_size_))
    ROS_ERROR("n_measurement in target state calc is not loaded from ros parameter server");
  if (!ros::param::get(prefix + "n_disturbance", disturbance_size_))
    ROS_ERROR("n_disturbance in target state calc is not loaded from ros parameter server");
  if (!ros::param::get(prefix + "A", temporary_A))
    ROS_ERROR("A in target state calc is not loaded from ros parameter server");
  if (!ros::param::get(prefix + "B", temporary_B))
    ROS_ERROR("B in target state calc is not loaded from ros parameter server");
  if (!ros::param::get(prefix + "Bd", temporary_Bd))
    ROS_ERROR("Bd in target state calc is not loaded from ros parameter server");
  if (!ros::param::get(prefix + "C", temporary_C))
    ROS_ERROR("C in target state calc is not loaded from ros parameter server");


  Eigen::MatrixXd left_hand_side;
  left_hand_side.resize(state_size_ + reference_size_, state_size_ + input_size_);

  Eigen::Map<Eigen::MatrixXd> A_map(temporary_A.data(), state_size_, state_size_);
  Eigen::Map<Eigen::MatrixXd> B_map(temporary_B.data(), state_size_, input_size_);
  Eigen::Map<Eigen::MatrixXd> Bd_map(temporary_Bd.data(), state_size_, disturbance_size_);
  Eigen::Map<Eigen::MatrixXd> C_map(temporary_C.data(), reference_size_, state_size_);

  Eigen::MatrixXd A;
  A = A_map;
  Eigen::MatrixXd B;
  B = B_map;
  Bd_ = Bd_map;
  Eigen::MatrixXd C;
  C = C_map;

  left_hand_side << A - Eigen::MatrixXd::Identity(state_size_, state_size_), B, C, Eigen::MatrixXd::Zero(
      reference_size_, input_size_);

  //TODO(mina) use a more robust way
  pseudo_inverse_left_hand_side_ = (left_hand_side.transpose() * left_hand_side).inverse()
      * left_hand_side.transpose();

  initialized_params_ = true;
  ROS_INFO("initializing the SS calc done");
}

void SteadyStateCalculation::ComputeSteadyState(Eigen::VectorXd &estimated_disturbance,
                                                Eigen::VectorXd &reference,
                                                Eigen::VectorXd* steady_state_state,
                                                Eigen::VectorXd* steady_state_input) {

  assert(steady_state_state);
  assert(steady_state_input);
  assert(initialized_params_);

  /*
   Eigen::VectorXd c1(state_size_ + output_size_);


   c1 << reference,  -Bd_*estimated_disturbance;
   Eigen::Map<Eigen::VectorXd>(const_cast<double*>(PARAMS.c1), state_size_ + output_size_,1)= c1;
   SteadyStateOptimization_output output;
   SteadyStateOptimization_info info;

   SteadyStateOptimization_solve(const_cast<SteadyStateOptimization_params*>(&PARAMS),&output,&info);
   */

  Eigen::VectorXd target_state_and_input(state_size_ + input_size_);
  Eigen::VectorXd right_hand_side(state_size_ + reference_size_);

  right_hand_side << -Bd_ * estimated_disturbance, reference;

  target_state_and_input = pseudo_inverse_left_hand_side_ * right_hand_side;
  // std::cout << "lhs = \n" << left_hand_side_ << std::endl;
  // std::cout << "rhs= \n" <<   right_hand_side << std::endl;
  // std::cout << "sol = \n" << target_state_and_input << std::endl;
  // ros::Duration(20.0).sleep();
  steady_state_state->resize(state_size_);
  steady_state_input->resize(input_size_);

  *steady_state_state = target_state_and_input.segment(0, state_size_);
  *steady_state_input = target_state_and_input.segment(state_size_, input_size_);

  /*Eigen::VectorXd output_eig(state_size_ + input_size_);
   output_eig <<  output.output_1[0], output.output_1[1],output.output_1[2],output.output_1[3],output.output_1[4],output.output_1[5],output.output_1[6],output.output_1[7] ,output.output_1[8], output.output_1[9], output.output_1[10];
   *steady_state_state << output_eig.segment(0, state_size_);
   *steady_state_input << output_eig.segment(state_size_, input_size_);
   */
}

}

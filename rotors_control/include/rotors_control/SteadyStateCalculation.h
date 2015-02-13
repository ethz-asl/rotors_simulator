/*
 * SteadyStateCalculation.h

 *
 *  Created on: Dec 4, 2014
 *      Author: mina
 */

#ifndef INCLUDE_MAV_CONTROL_STEADYSTATECALCULATION_H_
#define INCLUDE_MAV_CONTROL_STEADYSTATECALCULATION_H_
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
//#include "SteadyStateOptimization.h"

#include <ros/ros.h>

namespace rotors_control{

class SteadyStateCalculation
{
 public:
  SteadyStateCalculation();
  ~SteadyStateCalculation();

  void Initialize();


  void ComputeSteadyState(Eigen::VectorXd &estimated_state, Eigen::VectorXd &reference,
                          Eigen::VectorXd* steadystate_state, Eigen::VectorXd* steady_state_input);







private:


   int state_size_;
   int disturbance_size_;
   int reference_size_;
   int input_size_;



   Eigen::MatrixXd Bd_;
   Eigen::MatrixXd pseudo_inverse_left_hand_side_;


//   SteadyStateOptimization_params PARAMS;
   bool initialized_params_;

};






}

#endif /* INCLUDE_MAV_CONTROL_STEADYSTATECALCULATION_H_ */

#ifndef MPC_POSITION_CONTROLLER_NODE_H
#define MPC_POSITION_CONTROLLER_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>


#include <mav_msgs/CommandRollPitchYawrateThrust.h>
#include <mav_msgs/CommandTrajectory.h>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "rotors_control/common.h"
#include "rotors_control/MPC_position_controller.h"



namespace rotors_control{

class MPCPositionControllerNode{
 public:
  MPCPositionControllerNode();
  ~MPCPositionControllerNode();
  
  void InitializeParams();


  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:

  MPCPositionController MPC_position_controller_;

  bool initialized_params_;

  bool got_first_trajectory_command_;


  std::string namespace_;
  std::string odometry_sub_topic_;
  std::string command_trajectory_sub_topic_;

  std::string command_roll_pitch_yawrate_thrust_topic_;



  ros::Subscriber odometry_sub_;
  ros::Subscriber cmd_trajectory_sub_;
  


  ros::Publisher command_roll_pitch_yawrate_thrust_pub_;



  void CommandTrajectoryCallback(
      const mav_msgs::CommandTrajectoryConstPtr& trajectory_reference_msg);

  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);


};

}
#endif

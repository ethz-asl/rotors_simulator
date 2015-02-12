#include <rotors_control/PID_attitude_controller.h>
#include "rotors_control/common.h"
#include <stdio.h>
#include <memory.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <mav_msgs/CommandRollPitchYawrateThrust.h>
#include <mav_msgs/MotorSpeed.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>


namespace rotors_control{


class PIDAttitudeControllerNode{
 public:
  PIDAttitudeControllerNode();
  ~PIDAttitudeControllerNode();
  
  void InitializeParams();


  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:

  PIDAttitudeController PID_attitude_controller_;

  std::string namespace_;
  //topic names
  std::string motor_velocity_reference_pub_topic_;
  std::string odometry_sub_topic_;
  std::string command_roll_pitch_yawrate_thrust_topic_;


  //publishers
  ros::Publisher motor_velocity_reference_pub_;


  // subscribers
  ros::Subscriber command_roll_pitch_yawrate_thrust_sub_;
  ros::Subscriber odometry_sub_;


  bool initialized_params_;

  bool got_first_attitude_command_;



  void CommandRollPitchYawRateThrustCallback(const mav_msgs::CommandRollPitchYawrateThrustConstPtr& roll_pitch_yawrate_thrust_reference);
  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);


};

}

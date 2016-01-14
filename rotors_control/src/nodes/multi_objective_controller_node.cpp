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


#include "multi_objective_controller_node.h"

#include <manipulator_msgs/default_topics_manipulator.h>
#include <mav_msgs/default_topics.h>
#include "rotors_control/parameters_ros.h"


namespace rotors_control {

MultiObjectiveControllerNode::MultiObjectiveControllerNode() {

  InitializeParams();

  // UAV commands subscribers
  cmd_pose_sub_ = nh_.subscribe(mav_msgs::default_topics::COMMAND_POSE, 1, &MultiObjectiveControllerNode::CommandPoseCallback, this);
  cmd_multi_dof_joint_trajectory_sub_ = nh_.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1, &MultiObjectiveControllerNode::MultiDofJointTrajectoryCallback, this);

  /************************************/
  /* Manipulator commands subscribers */
  /************************************/
  // DM commands for direct joints angle control
  cmd_joints_trajectory_sub_ = nh_.subscribe(manipulator_msgs::default_topics::COMMAND_JOINT_TRAJECTORY, 1, &MultiObjectiveControllerNode::JointTrajectoryCallback, this);
  cmd_joints_angle_sub_ = nh_.subscribe(manipulator_msgs::default_topics::COMMAND_JOINT_ANGLES, 1, &MultiObjectiveControllerNode::JointCommandAngleCallback, this);
  // DM commands for joints angle control via end-effector reference position
  cmd_ee_multi_dof_joint_trajectory_sub_ = nh_.subscribe(manipulator_msgs::default_topics::COMMAND_EE_TRAJECTORY, 1, &MultiObjectiveControllerNode::EndEffMultiDofJointTrajectoryCallback, this);
  cmd_ee_pose_sub_ = nh_.subscribe(manipulator_msgs::default_topics::COMMAND_EE_POSE, 1, &MultiObjectiveControllerNode::EndEffCommandPoseCallback, this);

  // Force sensor subscriber
  force_sensor_sub_ = nh_.subscribe(manipulator_msgs::default_topics::FORCE_SENSOR_LINEAR, 1, &MultiObjectiveControllerNode::ForceSensorCallback, this);

  /****************************************************************************/
  /* Robot state subscribers (synchronized via message_filter implementation) */
  /****************************************************************************/
  // UAV odometry
  odometry_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,mav_msgs::default_topics::ODOMETRY, 10);
  // DM joints state (3D vector)
  joint_state_sub_.clear();
  joint_state_sub_.push_back(new message_filters::Subscriber<sensor_msgs::JointState>(nh_, manipulator_msgs::default_topics::MOTOR_PITCHING_JOINT_STATE, 10));
  joint_state_sub_.push_back(new message_filters::Subscriber<sensor_msgs::JointState>(nh_, manipulator_msgs::default_topics::MOTOR_LEFT_JOINT_STATE, 10));
  joint_state_sub_.push_back(new message_filters::Subscriber<sensor_msgs::JointState>(nh_, manipulator_msgs::default_topics::MOTOR_RIGHT_JOINT_STATE, 10));

  // Synchronizer
  sync_ = new message_filters::Synchronizer<RobotSyncPolicy>(RobotSyncPolicy(10),*odometry_sub_, *(joint_state_sub_[0]),*(joint_state_sub_[1]),*(joint_state_sub_[2]));
  sync_->registerCallback(boost::bind(&MultiObjectiveControllerNode::AerialManipulatorStateCallback, this, _1, _2, _3, _4));

  // Rotors velocity publisher
  motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::COMMAND_ACTUATORS, 10);

  // Manipulator actuators publishers (one for each DM motor)
  pitch_motor_torque_ref_pub_ = nh_.advertise<manipulator_msgs::CommandTorqueServoMotor>(manipulator_msgs::default_topics::COMMAND_MOTOR_PITCHING_TORQUE, 10);
  left_motor_torque_ref_pub_ = nh_.advertise<manipulator_msgs::CommandTorqueServoMotor>(manipulator_msgs::default_topics::COMMAND_MOTOR_LEFT_TORQUE, 10);
  right_motor_torque_ref_pub_ = nh_.advertise<manipulator_msgs::CommandTorqueServoMotor>(manipulator_msgs::default_topics::COMMAND_MOTOR_RIGHT_TORQUE, 10);

  // Timers
  command_timer_ = nh_.createTimer(ros::Duration(0), &MultiObjectiveControllerNode::TimedCommandCallback, this, true, false);
  command_arm_timer_ = nh_.createTimer(ros::Duration(0), &MultiObjectiveControllerNode::TimedCommandArmCallback, this, true, false);
}


MultiObjectiveControllerNode::~MultiObjectiveControllerNode() { }


void MultiObjectiveControllerNode::InitializeParams() {
/* Here we load parameters from 'multi_objective_controller_*.yaml' file, where * contains information about
 * robot model and parameters set defined either by the user or selected by default. An example of this file
 * and editable parameters can be found in 'rotors_gazebo/resource' folder. Possibly updated parameters are
 * then used to initialize 'multi_objective_controller_' object.
 */

  ros::NodeHandle pnh("~");

  namespace_ = pnh.getNamespace();

  // Parameters server subscriber
  params_server_.setCallback(boost::bind(&MultiObjectiveControllerNode::ParamsDynReconfigureCallback, this, _1, _2));

  int mav_objective_function_idx, manipulator_objective_function_idx;

  // Read parameters from rosparam
  GetRosParameter(pnh, "mav_position_gain_x",
                  multi_objective_controller_.controller_parameters_.mav_position_gain_.x(),
                  &multi_objective_controller_.controller_parameters_.mav_position_gain_.x());
  GetRosParameter(pnh, "mav_position_gain_y",
                  multi_objective_controller_.controller_parameters_.mav_position_gain_.y(),
                  &multi_objective_controller_.controller_parameters_.mav_position_gain_.y());
  GetRosParameter(pnh, "mav_position_gain_z",
                  multi_objective_controller_.controller_parameters_.mav_position_gain_.z(),
                  &multi_objective_controller_.controller_parameters_.mav_position_gain_.z());
  GetRosParameter(pnh, "mav_velocity_gain_x",
                  multi_objective_controller_.controller_parameters_.mav_velocity_gain_.x(),
                  &multi_objective_controller_.controller_parameters_.mav_velocity_gain_.x());
  GetRosParameter(pnh, "mav_velocity_gain_y",
                  multi_objective_controller_.controller_parameters_.mav_velocity_gain_.y(),
                  &multi_objective_controller_.controller_parameters_.mav_velocity_gain_.y());
  GetRosParameter(pnh, "mav_velocity_gain_z",
                  multi_objective_controller_.controller_parameters_.mav_velocity_gain_.z(),
                  &multi_objective_controller_.controller_parameters_.mav_velocity_gain_.z());
  GetRosParameter(pnh, "mav_attitude_gain_roll",
                  multi_objective_controller_.controller_parameters_.mav_attitude_gain_.x(),
                  &multi_objective_controller_.controller_parameters_.mav_attitude_gain_.x());
  GetRosParameter(pnh, "mav_attitude_gain_pitch",
                  multi_objective_controller_.controller_parameters_.mav_attitude_gain_.y(),
                  &multi_objective_controller_.controller_parameters_.mav_attitude_gain_.y());
  GetRosParameter(pnh, "mav_attitude_gain_yaw",
                  multi_objective_controller_.controller_parameters_.mav_attitude_gain_.z(),
                  &multi_objective_controller_.controller_parameters_.mav_attitude_gain_.z());
  GetRosParameter(pnh, "mav_angular_rate_gain_roll",
                  multi_objective_controller_.controller_parameters_.mav_angular_rate_gain_.x(),
                  &multi_objective_controller_.controller_parameters_.mav_angular_rate_gain_.x());
  GetRosParameter(pnh, "mav_angular_rate_gain_pitch",
                  multi_objective_controller_.controller_parameters_.mav_angular_rate_gain_.y(),
                  &multi_objective_controller_.controller_parameters_.mav_angular_rate_gain_.y());
  GetRosParameter(pnh, "mav_angular_rate_gain_yaw",
                  multi_objective_controller_.controller_parameters_.mav_angular_rate_gain_.z(),
                  &multi_objective_controller_.controller_parameters_.mav_angular_rate_gain_.z());
  GetRosParameter(pnh, "ee_position_gain_x",
                  multi_objective_controller_.controller_parameters_.ee_position_gain_.x(),
                  &multi_objective_controller_.controller_parameters_.ee_position_gain_.x());
  GetRosParameter(pnh, "ee_position_gain_y",
                  multi_objective_controller_.controller_parameters_.ee_position_gain_.y(),
                  &multi_objective_controller_.controller_parameters_.ee_position_gain_.y());
  GetRosParameter(pnh, "ee_position_gain_z",
                  multi_objective_controller_.controller_parameters_.ee_position_gain_.z(),
                  &multi_objective_controller_.controller_parameters_.ee_position_gain_.z());
  GetRosParameter(pnh, "ee_velocity_gain_x",
                  multi_objective_controller_.controller_parameters_.ee_velocity_gain_.x(),
                  &multi_objective_controller_.controller_parameters_.ee_velocity_gain_.x());
  GetRosParameter(pnh, "ee_velocity_gain_y",
                  multi_objective_controller_.controller_parameters_.ee_velocity_gain_.y(),
                  &multi_objective_controller_.controller_parameters_.ee_velocity_gain_.y());
  GetRosParameter(pnh, "ee_velocity_gain_z",
                  multi_objective_controller_.controller_parameters_.ee_velocity_gain_.z(),
                  &multi_objective_controller_.controller_parameters_.ee_velocity_gain_.z());
  GetRosParameter(pnh, "arm_joints_angle_gain_pitch",
                  multi_objective_controller_.controller_parameters_.arm_joints_angle_gain_.x(),
                  &multi_objective_controller_.controller_parameters_.arm_joints_angle_gain_.x());
  GetRosParameter(pnh, "arm_joints_angle_gain_left",
                  multi_objective_controller_.controller_parameters_.arm_joints_angle_gain_.y(),
                  &multi_objective_controller_.controller_parameters_.arm_joints_angle_gain_.y());
  GetRosParameter(pnh, "arm_joints_angle_gain_right",
                  multi_objective_controller_.controller_parameters_.arm_joints_angle_gain_.z(),
                  &multi_objective_controller_.controller_parameters_.arm_joints_angle_gain_.z());
  GetRosParameter(pnh, "arm_joints_ang_rate_gain_pitch",
                  multi_objective_controller_.controller_parameters_.arm_joints_ang_rate_gain_.x(),
                  &multi_objective_controller_.controller_parameters_.arm_joints_ang_rate_gain_.x());
  GetRosParameter(pnh, "arm_joints_ang_rate_gain_left",
                  multi_objective_controller_.controller_parameters_.arm_joints_ang_rate_gain_.y(),
                  &multi_objective_controller_.controller_parameters_.arm_joints_ang_rate_gain_.y());
  GetRosParameter(pnh, "arm_joints_ang_rate_gain_right",
                  multi_objective_controller_.controller_parameters_.arm_joints_ang_rate_gain_.z(),
                  &multi_objective_controller_.controller_parameters_.arm_joints_ang_rate_gain_.z());
  GetRosParameter(pnh, "rpy_max/roll",
                  multi_objective_controller_.controller_parameters_.rpy_max_.x(),
                  &multi_objective_controller_.controller_parameters_.rpy_max_.x());
  GetRosParameter(pnh, "rpy_max/pitch",
                  multi_objective_controller_.controller_parameters_.rpy_max_.y(),
                  &multi_objective_controller_.controller_parameters_.rpy_max_.y());
  GetRosParameter(pnh, "rpy_min/roll",
                  multi_objective_controller_.controller_parameters_.rpy_min_.x(),
                  &multi_objective_controller_.controller_parameters_.rpy_min_.x());
  GetRosParameter(pnh, "rpy_min/pitch",
                  multi_objective_controller_.controller_parameters_.rpy_min_.y(),
                  &multi_objective_controller_.controller_parameters_.rpy_min_.y());
  GetRosParameter(pnh, "arm_joint_angle_max/pitch",
                  multi_objective_controller_.controller_parameters_.arm_joints_angle_max_.x(),
                  &multi_objective_controller_.controller_parameters_.arm_joints_angle_max_.x());
  GetRosParameter(pnh, "arm_joint_angle_max/left",
                  multi_objective_controller_.controller_parameters_.arm_joints_angle_max_.y(),
                  &multi_objective_controller_.controller_parameters_.arm_joints_angle_max_.y());
  GetRosParameter(pnh, "arm_joint_angle_max/right",
                  multi_objective_controller_.controller_parameters_.arm_joints_angle_max_.z(),
                  &multi_objective_controller_.controller_parameters_.arm_joints_angle_max_.z());
  GetRosParameter(pnh, "arm_joint_angle_min/pitch",
                  multi_objective_controller_.controller_parameters_.arm_joints_angle_min_.x(),
                  &multi_objective_controller_.controller_parameters_.arm_joints_angle_min_.x());
  GetRosParameter(pnh, "arm_joint_angle_min/left",
                  multi_objective_controller_.controller_parameters_.arm_joints_angle_min_.y(),
                  &multi_objective_controller_.controller_parameters_.arm_joints_angle_min_.y());
  GetRosParameter(pnh, "arm_joint_angle_min/right",
                  multi_objective_controller_.controller_parameters_.arm_joints_angle_min_.z(),
                  &multi_objective_controller_.controller_parameters_.arm_joints_angle_min_.z());
  GetRosParameter(pnh, "mav_objective_function", 0,
                  &mav_objective_function_idx);
  GetRosParameter(pnh, "manipulator_objective_function", 4,
                  &manipulator_objective_function_idx);
  GetRosParameter(pnh, "value_mav", multi_objective_controller_.controller_parameters_.objectives_weight_.head<4>().maxCoeff(),
                  &multi_objective_controller_.controller_parameters_.objectives_weight_(mav_objective_function_idx));
  GetRosParameter(pnh, "value_arm", multi_objective_controller_.controller_parameters_.objectives_weight_.tail<3>().maxCoeff(),
                  &multi_objective_controller_.controller_parameters_.objectives_weight_(manipulator_objective_function_idx));
  GetRosParameter(pnh, "arm_joint_torque_lim/min",
                  multi_objective_controller_.controller_parameters_.arm_joint_torque_lim_.x(),
                  &multi_objective_controller_.controller_parameters_.arm_joint_torque_lim_.x());
  GetRosParameter(pnh, "arm_joint_torque_lim/max",
                  multi_objective_controller_.controller_parameters_.arm_joint_torque_lim_.y(),
                  &multi_objective_controller_.controller_parameters_.arm_joint_torque_lim_.y());
  GetRosParameter(pnh, "max_rot_velocity",
                  multi_objective_controller_.controller_parameters_.max_rot_velocity_,
                  &multi_objective_controller_.controller_parameters_.max_rot_velocity_);
  GetRosParameter(pnh, "mu_attitude",
                  multi_objective_controller_.controller_parameters_.mu_attitude_,
                  &multi_objective_controller_.controller_parameters_.mu_attitude_);
  GetRosParameter(pnh, "mu_arm",
                  multi_objective_controller_.controller_parameters_.mu_arm_,
                  &multi_objective_controller_.controller_parameters_.mu_arm_);
  GetRosParameter(pnh, "safe_range_rpy",
                  multi_objective_controller_.controller_parameters_.safe_range_rpy_,
                  &multi_objective_controller_.controller_parameters_.safe_range_rpy_);
  GetRosParameter(pnh, "safe_range_joints",
                  multi_objective_controller_.controller_parameters_.safe_range_joints_,
                  &multi_objective_controller_.controller_parameters_.safe_range_joints_);

  GetVehicleParameters(pnh, &multi_objective_controller_.vehicle_parameters_);

  multi_objective_controller_.InitializeParameters();

  multi_objective_controller_.SetDesiredFrozenJointsAngle();

  //Todo: Workaround to update objective functions weight in rqt_reconfigure from yaml default
//  rotors_control::MultiObjectiveControllerConfig config;
//  params_server_.getConfigDefault(config);
//  config.value_arm = multi_objective_controller_.controller_parameters_.objectives_weight_.tail<3>().maxCoeff();
//  params_server_.updateConfig(config);
}


void MultiObjectiveControllerNode::ParamsDynReconfigureCallback(rotors_control::MultiObjectiveControllerConfig& config, uint32_t level) {
/* Configuration file for dynamic reconfigure tool can be found in 'rotors_control/cfg' folder. For more details about
 * how to edit or create a new .CFG file, visit http://wiki.ros.org/dynamic_reconfigure/Tutorials/HowToWriteYourFirstCfgFile.
 */

  multi_objective_controller_.controller_parameters_.mav_position_gain_.x() = config.mav_position_gain_x;
  multi_objective_controller_.controller_parameters_.mav_position_gain_.y() = config.mav_position_gain_y;
  multi_objective_controller_.controller_parameters_.mav_position_gain_.z() = config.mav_position_gain_z;

  multi_objective_controller_.controller_parameters_.mav_velocity_gain_.x() = config.mav_velocity_gain_x;
  multi_objective_controller_.controller_parameters_.mav_velocity_gain_.y() = config.mav_velocity_gain_y;
  multi_objective_controller_.controller_parameters_.mav_velocity_gain_.z() = config.mav_velocity_gain_z;

  multi_objective_controller_.controller_parameters_.mav_attitude_gain_.x() = config.mav_attitude_gain_roll;
  multi_objective_controller_.controller_parameters_.mav_attitude_gain_.y() = config.mav_attitude_gain_pitch;
  multi_objective_controller_.controller_parameters_.mav_attitude_gain_.z() = config.mav_attitude_gain_yaw;

  multi_objective_controller_.controller_parameters_.mav_angular_rate_gain_.x() = config.mav_angular_rate_gain_roll;
  multi_objective_controller_.controller_parameters_.mav_angular_rate_gain_.y() = config.mav_angular_rate_gain_pitch;
  multi_objective_controller_.controller_parameters_.mav_angular_rate_gain_.z() = config.mav_angular_rate_gain_yaw;

  multi_objective_controller_.controller_parameters_.arm_joints_angle_gain_.x() = config.arm_joints_angle_gain_pitch;
  multi_objective_controller_.controller_parameters_.arm_joints_angle_gain_.y() = config.arm_joints_angle_gain_left;
  multi_objective_controller_.controller_parameters_.arm_joints_angle_gain_.z() = config.arm_joints_angle_gain_right;

  multi_objective_controller_.controller_parameters_.arm_joints_ang_rate_gain_.x() = config.arm_joints_ang_rate_gain_pitch;
  multi_objective_controller_.controller_parameters_.arm_joints_ang_rate_gain_.y() = config.arm_joints_ang_rate_gain_left;
  multi_objective_controller_.controller_parameters_.arm_joints_ang_rate_gain_.z() = config.arm_joints_ang_rate_gain_right;

  multi_objective_controller_.controller_parameters_.ee_position_gain_.x() = config.ee_position_gain_x;
  multi_objective_controller_.controller_parameters_.ee_position_gain_.y() = config.ee_position_gain_y;
  multi_objective_controller_.controller_parameters_.ee_position_gain_.z() = config.ee_position_gain_z;

  multi_objective_controller_.controller_parameters_.ee_velocity_gain_.x() = config.ee_velocity_gain_x;
  multi_objective_controller_.controller_parameters_.ee_velocity_gain_.y() = config.ee_velocity_gain_y;
  multi_objective_controller_.controller_parameters_.ee_velocity_gain_.z() = config.ee_velocity_gain_z;

  multi_objective_controller_.controller_parameters_.mu_attitude_ = config.mu_attitude;
  multi_objective_controller_.controller_parameters_.mu_arm_ = config.mu_arm;

  multi_objective_controller_.controller_parameters_.safe_range_rpy_ = config.safe_range_rpy;
  multi_objective_controller_.controller_parameters_.safe_range_joints_ = config.safe_range_joints;

  multi_objective_controller_.controller_parameters_.objectives_weight_.setZero();
  multi_objective_controller_.controller_parameters_.objectives_weight_(config.mav_objective_function) = config.value_mav;
  multi_objective_controller_.controller_parameters_.objectives_weight_(config.manipulator_objective_function) = config.value_arm;
  multi_objective_controller_.SetDesiredFrozenJointsAngle();


  ROS_INFO("[multi_objective_controller_node] Controller parameters reconfigured.");
  std::cout << "SUMMARY\n========\n\nPARAMETERS" << std::endl;
  PrintParam("mav_position_gain/x", config.mav_position_gain_x);
  PrintParam("mav_position_gain/y", config.mav_position_gain_y);
  PrintParam("mav_position_gain/z", config.mav_position_gain_z);
  PrintParam("mav_velocity_gain/x", config.mav_velocity_gain_x);
  PrintParam("mav_velocity_gain/y", config.mav_velocity_gain_y);
  PrintParam("mav_velocity_gain/z", config.mav_velocity_gain_z);
  PrintParam("mav_attitude_gain/roll", config.mav_attitude_gain_roll);
  PrintParam("mav_attitude_gain/pitch", config.mav_attitude_gain_pitch);
  PrintParam("mav_attitude_gain/yaw", config.mav_attitude_gain_yaw);
  PrintParam("mav_angular_rate_gain/roll", config.mav_angular_rate_gain_roll);
  PrintParam("mav_angular_rate_gain/pitch", config.mav_angular_rate_gain_pitch);
  PrintParam("mav_angular_rate_gain/yaw", config.mav_angular_rate_gain_yaw);
  PrintParam("arm_joints_angle_gain/pitch", config.arm_joints_angle_gain_pitch);
  PrintParam("arm_joints_angle_gain/left", config.arm_joints_angle_gain_left);
  PrintParam("arm_joints_angle_gain/right", config.arm_joints_angle_gain_right);
  PrintParam("arm_joints_ang_rate_gain/pitch", config.arm_joints_ang_rate_gain_pitch);
  PrintParam("arm_joints_ang_rate_gain/left", config.arm_joints_ang_rate_gain_left);
  PrintParam("arm_joints_ang_rate_gain/right", config.arm_joints_ang_rate_gain_right);
  PrintParam("ee_position_gain/x", config.ee_position_gain_x);
  PrintParam("ee_position_gain/y", config.ee_position_gain_y);
  PrintParam("ee_position_gain/z", config.ee_position_gain_z);
  PrintParam("ee_velocity_gain/x", config.ee_velocity_gain_x);
  PrintParam("ee_velocity_gain/y", config.ee_velocity_gain_y);
  PrintParam("ee_velocity_gain/z", config.ee_velocity_gain_z);
  PrintParam("mu_attitude", config.mu_attitude);
  PrintParam("mu_arm", config.mu_arm);
  PrintParam("safe_range_rpy", config.safe_range_rpy);
  PrintParam("safe_range_joints", config.safe_range_joints);
  PrintParam("objectives_weight/att", multi_objective_controller_.controller_parameters_.objectives_weight_(0));
  PrintParam("objectives_weight/pos", multi_objective_controller_.controller_parameters_.objectives_weight_(1));
  PrintParam("objectives_weight/yaw", multi_objective_controller_.controller_parameters_.objectives_weight_(2));
  PrintParam("objectives_weight/vel", multi_objective_controller_.controller_parameters_.objectives_weight_(3));
  PrintParam("objectives_weight/arm", multi_objective_controller_.controller_parameters_.objectives_weight_(4));
  PrintParam("objectives_weight/ee", multi_objective_controller_.controller_parameters_.objectives_weight_(5));
  PrintParam("objectives_weight/frozen", multi_objective_controller_.controller_parameters_.objectives_weight_(6));
  std::cout << std::endl;
}

/***********************************************************************************************************************/
/******  UAV CALLSBACK  ******/
/***********************************************************************************************************************/

void MultiObjectiveControllerNode::CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
  commands_.push_front(eigen_reference);

  multi_objective_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
}


void MultiObjectiveControllerNode::MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference);

  for (size_t i = 1; i < n_commands; ++i) {
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before = msg->points[i-1];
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference = msg->points[i];

    mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);

    commands_.push_back(eigen_reference);
    command_waiting_times_.push_back(current_reference.time_from_start - reference_before.time_from_start);
  }

  // We can trigger the first command immediately.
  multi_objective_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();

  if (n_commands > 1) {
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

/***********************************************************************************************************************/
/******  MANIPULATOR CALLSBACK  ******/
/***********************************************************************************************************************/

void MultiObjectiveControllerNode::EndEffCommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  /* Clear all pending commands. Notice that only one kind of trajectory for DM can be parsed at a time (joints angle or e.e.).
   * Hence, whenever a new manipulator related set of commands is received, all pending trajectory points from previous trajectory
   * (if any) are cleared, no matter if they were the same type or not.
   */
  command_timer_.stop();
  commands_ee_.clear();
  commands_joints_.clear();
  command_waiting_times_.clear();

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
  commands_ee_.push_front(eigen_reference);

  multi_objective_controller_.SetEndEffTrajectoryPoint(commands_ee_.front());
  commands_ee_.pop_front();
}


void MultiObjectiveControllerNode::EndEffMultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  /* Clear all pending commands. Notice that only one kind of trajectory for DM can be parsed at a time (joints angle or e.e.).
   * Hence, whenever a new manipulator related set of commands is received, all pending trajectory points from previous trajectory
   * (if any) are cleared, no matter if they were the same type or not.
   */
  command_arm_timer_.stop();
  commands_ee_.clear();
  commands_joints_.clear();
  command_arm_waiting_times_.clear();

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got end-effector MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_ee_.push_front(eigen_reference);

  for (size_t i = 1; i < n_commands; ++i) {
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before = msg->points[i-1];
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference = msg->points[i];

    mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);

    commands_ee_.push_back(eigen_reference);
    command_arm_waiting_times_.push_back(current_reference.time_from_start - reference_before.time_from_start);
  }

  // We can trigger the first command immediately.
  multi_objective_controller_.SetEndEffTrajectoryPoint(commands_ee_.front());
  commands_ee_.pop_front();

  if (n_commands > 1) {
    command_arm_timer_.setPeriod(command_arm_waiting_times_.front());
    command_arm_waiting_times_.pop_front();
    command_arm_timer_.start();
  }
}


void MultiObjectiveControllerNode::JointTrajectoryCallback(const trajectory_msgs::JointTrajectoryConstPtr& trajectory_msg) {
  /* Clear all pending commands. Notice that only one kind of trajectory for DM can be parsed at a time (joints angle or e.e.).
   * Hence, whenever a new manipulator related set of commands is received, all pending trajectory points from previous trajectory
   * (if any) are cleared, no matter if they were the same type or not.
   */
  command_arm_timer_.stop();
  commands_ee_.clear();
  commands_joints_.clear();
  command_arm_waiting_times_.clear();

  const size_t n_commands = trajectory_msg->points.size();
  if(n_commands < 1){
    ROS_WARN_STREAM("Got JointTrajectory message, but message has no points.");
    return;
  }

  manipulator_msgs::EigenJointTrajectoryPoint eigen_reference;
  manipulator_msgs::eigenJointTrajectoryPointFromMsg(trajectory_msg->points.front(), &eigen_reference);
  commands_joints_.push_front(eigen_reference);

  for (size_t i = 1; i < n_commands; ++i) {
    const trajectory_msgs::JointTrajectoryPoint& reference_before = trajectory_msg->points[i-1];
    const trajectory_msgs::JointTrajectoryPoint& current_reference = trajectory_msg->points[i];

    manipulator_msgs::eigenJointTrajectoryPointFromMsg(current_reference, &eigen_reference);

    commands_joints_.push_back(eigen_reference);
    command_arm_waiting_times_.push_back(current_reference.time_from_start - reference_before.time_from_start);
  }

  // We can trigger the first command immediately.
  multi_objective_controller_.SetDesiredJointsAngle(commands_joints_.front());
  commands_joints_.pop_front();
  if (n_commands > 1) {
    command_arm_timer_.setPeriod(command_arm_waiting_times_.front());
    command_arm_waiting_times_.pop_front();
    command_arm_timer_.start();
  }
}


void MultiObjectiveControllerNode::JointCommandAngleCallback(const geometry_msgs::Vector3StampedConstPtr& vector3_msg) {
  /* Clear all pending commands. Notice that only one kind of trajectory for DM can be parsed at a time (joints angle or e.e.).
   * Hence, whenever a new manipulator related set of commands is received, all pending trajectory points from previous trajectory
   * (if any) are cleared, no matter if they were the same type or not.
   */
  command_arm_timer_.stop();
  commands_ee_.clear();
  commands_joints_.clear();
  command_arm_waiting_times_.clear();

  manipulator_msgs::EigenJointTrajectoryPoint eigen_reference;
  manipulator_msgs::eigenJointTrajectoryPointFromVector3(vector3_msg->vector, &eigen_reference);
  commands_joints_.push_front(eigen_reference);

  multi_objective_controller_.SetDesiredJointsAngle(commands_joints_.front());
  commands_joints_.pop_front();
}

/***********************************************************************************************************************/
/******  TIMERS CALLSBACK  ******/
/***********************************************************************************************************************/

void MultiObjectiveControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {

  if(commands_.empty()){
    ROS_WARN("Commands empty, this should not happen here");
    return;
  }

  multi_objective_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();

  command_timer_.stop();
  if(!command_waiting_times_.empty()){
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}


void MultiObjectiveControllerNode::TimedCommandArmCallback(const ros::TimerEvent& e) {

  if (commands_ee_.empty() and commands_joints_.empty()) {
    ROS_WARN("Manipulator commands empty, this should not happen here");
    return;
  }

  if (commands_ee_.empty()) {
    multi_objective_controller_.SetDesiredJointsAngle(commands_joints_.front());
    commands_joints_.pop_front();
  } else {
    multi_objective_controller_.SetEndEffTrajectoryPoint(commands_ee_.front());
    commands_ee_.pop_front();
  }

  command_arm_timer_.stop();
  if(!command_arm_waiting_times_.empty()){
    command_arm_timer_.setPeriod(command_arm_waiting_times_.front());
    command_arm_waiting_times_.pop_front();
    command_arm_timer_.start();
  }
}

/***********************************************************************************************************************/
/******  STATE UPDATE CALLSBACK  ******/
/***********************************************************************************************************************/

void MultiObjectiveControllerNode::AerialManipulatorStateCallback(const nav_msgs::OdometryConstPtr& odometry_msg,
                                                                  const sensor_msgs::JointStateConstPtr& joint_state_msg0,
                                                                  const sensor_msgs::JointStateConstPtr& joint_state_msg1,
                                                                  const sensor_msgs::JointStateConstPtr& joint_state_msg2) {

  ROS_INFO_ONCE("MultiObjectiveController got first synchronized odometry+joint states messages.");

  // Update robot state (first UAV, then DM)
  mav_msgs::EigenOdometry odometry;
  mav_msgs::eigenOdometryFromMsg(*(odometry_msg.get()), &odometry);
  multi_objective_controller_.SetOdometry(odometry);
  manipulator_msgs::EigenJointsState joints_state;
  std::vector<sensor_msgs::JointState> joint_state_msgs;
  joint_state_msgs.push_back(*(joint_state_msg0.get()));
  joint_state_msgs.push_back(*(joint_state_msg1.get()));
  joint_state_msgs.push_back(*(joint_state_msg2.get()));
  eigenJointsStateFromMsg(joint_state_msgs, &joints_state);
  multi_objective_controller_.SetArmJointsState(joints_state);

  // Run optimization and compute control inputs. Publish nothing if optimization fails.
  Eigen::VectorXd ref_rotor_velocities;
  Eigen::Vector3d ref_torques;
  if (!multi_objective_controller_.CalculateControlInputs(&ref_rotor_velocities, &ref_torques))
    return;

  // Publish rotors speed messages.
  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  actuator_msg->header.stamp = odometry_msg->header.stamp;
  motor_velocity_reference_pub_.publish(actuator_msg);

  // Publish torque messages.
  manipulator_msgs::CommandTorqueServoMotorPtr torque_msg(new manipulator_msgs::CommandTorqueServoMotor);
  torque_msg->header.stamp = odometry_msg->header.stamp;
  torque_msg->torque = ref_torques.x();
  pitch_motor_torque_ref_pub_.publish(torque_msg);
  torque_msg->torque = ref_torques.y();
  left_motor_torque_ref_pub_.publish(torque_msg);
  torque_msg->torque = ref_torques.z();
  right_motor_torque_ref_pub_.publish(torque_msg);

}

////// FORCE SENSOR CALLBACK //////

void MultiObjectiveControllerNode::ForceSensorCallback(const geometry_msgs::Vector3StampedConstPtr& force_msg) {
  Eigen::Vector3d eigen_forces = mav_msgs::vector3FromMsg(force_msg->vector);
  multi_objective_controller_.SetExternalForces(eigen_forces);
}

}

/****************************************************************************/
//////**************************** MAIN ********************************//////
/****************************************************************************/

int main(int argc, char** argv) {
  ros::init(argc, argv, "multi_objective_controller_node");

  rotors_control::MultiObjectiveControllerNode multi_objective_controller_node;

  ros::spin();

  return 0;
}

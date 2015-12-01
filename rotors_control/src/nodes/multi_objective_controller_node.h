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

#ifndef ROTORS_CONTROL_MULTI_OBJECTIVE_CONTROLLER_NODE_H
#define ROTORS_CONTROL_MULTI_OBJECTIVE_CONTROLLER_NODE_H

#include <stdio.h>
#include <Eigen/Eigen>
#include <boost/bind.hpp>
#include <ros/callback_queue.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <manipulator_msgs/CommandTorqueServoMotor.h>

#include "rotors_control/multi_objective_controller.h"

#include <dynamic_reconfigure/server.h>
#include <rotors_control/MultiObjectiveControllerConfig.h>


typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, sensor_msgs::JointState, sensor_msgs::JointState, sensor_msgs::JointState> RobotSyncPolicy;


namespace rotors_control {

class MultiObjectiveControllerNode {

 public:

  MultiObjectiveControllerNode();
  ~MultiObjectiveControllerNode();

  void InitializeParams();
  void Publish() {};


 private:

  MultiObjectiveController multi_objective_controller_;

  std::string namespace_;

  ros::NodeHandle nh_;

  // dynamic_reconfigure
  dynamic_reconfigure::Server<rotors_control::MultiObjectiveControllerConfig> params_server_;

  // subscribers
  ros::Subscriber cmd_multi_dof_joint_trajectory_sub_;
  ros::Subscriber cmd_pose_sub_;
  ros::Subscriber cmd_ee_multi_dof_joint_trajectory_sub_;
  ros::Subscriber cmd_ee_pose_sub_;
  ros::Subscriber cmd_joints_trajectory_sub_;
  ros::Subscriber cmd_joints_angle_sub_;
  ros::Subscriber force_sensor_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> *odometry_sub_;
  std::vector<message_filters::Subscriber<sensor_msgs::JointState>*> joint_state_sub_;

  // synchronizer
  message_filters::Synchronizer<RobotSyncPolicy> *sync_;

  // publishers
  ros::Publisher motor_velocity_reference_pub_;
  ros::Publisher pitch_motor_torque_ref_pub_;
  ros::Publisher left_motor_torque_ref_pub_;
  ros::Publisher right_motor_torque_ref_pub_;

  // commands
  mav_msgs::EigenTrajectoryPointDeque commands_;
  mav_msgs::EigenTrajectoryPointDeque commands_ee_;
  manipulator_msgs::EigenJointTrajectoryPointDeque commands_joints_;

  // timers
  std::deque<ros::Duration> command_waiting_times_;
  ros::Timer command_timer_;
  std::deque<ros::Duration> command_arm_waiting_times_;
  ros::Timer command_arm_timer_;

  // params dynamic reconfigure callback
  void ParamsDynReconfigureCallback(rotors_control::MultiObjectiveControllerConfig& config, uint32_t level);

  // timers callsback
  void TimedCommandCallback(const ros::TimerEvent& e);
  void TimedCommandArmCallback(const ros::TimerEvent& e);

  // UAV trajectories callbacks
  void MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);
  void CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg);

  // Manipulator trajectories callbacks
  void EndEffMultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);
  void EndEffCommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg);
  void JointTrajectoryCallback(const trajectory_msgs::JointTrajectoryConstPtr& trajectory_reference_msg);
  void JointCommandAngleCallback(const geometry_msgs::Vector3StampedConstPtr& vector3_msg);

  // Aerial manipulator state update callback
  void AerialManipulatorStateCallback(const nav_msgs::OdometryConstPtr& odometry_msg,
                                      const sensor_msgs::JointStateConstPtr& joint_state_msg0,
                                      const sensor_msgs::JointStateConstPtr& joint_state_msg1,
                                      const sensor_msgs::JointStateConstPtr& joint_state_msg2);

  // Force sensor callback
  void ForceSensorCallback(const geometry_msgs::Vector3StampedConstPtr& force_msg);

  // miscellaneous
  template<class T>
  inline void PrintParam(const std::string& param_str, const T& value) {
    std::cout << " * " << namespace_.c_str() << "/" << param_str.c_str() << ": " << value << std::endl;
  }
};
}

#endif // ROTORS_CONTROL_MULTI_OBJECTIVE_CONTROLLER_NODE_H

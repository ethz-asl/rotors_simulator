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

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "multi_objective_controller_node.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control {

MultiObjectiveControllerNode::MultiObjectiveControllerNode() {

      InitializeParams();

      cmd_pose_sub_ = nh_.subscribe(
          mav_msgs::default_topics::COMMAND_POSE, 1,
          &MultiObjectiveControllerNode::CommandPoseCallback, this);

      cmd_multi_dof_joint_trajectory_sub_ = nh_.subscribe(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
          &MultiObjectiveControllerNode::MultiDofJointTrajectoryCallback, this);

      odometry_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,mav_msgs::default_topics::ODOMETRY, 10);
//      joint_state_sub_ = new message_filters::Subscriber<sensor_msgs::JointState>(nh_,"joint_State", 10);
//      sync_ = new message_filters::Synchronizer<RobotSyncPolicy>(RobotSyncPolicy(10),*odometry_sub_,*joint_state_sub_);

      joint_state_sub_.clear();
      joint_state_sub_.push_back(new message_filters::Subscriber<sensor_msgs::JointState>(nh_, "pitching", 10));
      joint_state_sub_.push_back(new message_filters::Subscriber<sensor_msgs::JointState>(nh_, "left", 10));
      joint_state_sub_.push_back(new message_filters::Subscriber<sensor_msgs::JointState>(nh_, "right", 10));
      sync_ = new message_filters::Synchronizer<RobotSyncPolicy>(RobotSyncPolicy(10),*odometry_sub_,
                                                                 *(joint_state_sub_[0]),*(joint_state_sub_[1]),*(joint_state_sub_[2]));

      sync_->registerCallback(boost::bind(&MultiObjectiveControllerNode::AerialManipulatorStateCallback, this, _1, _2, _3, _4));

      motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::COMMAND_ACTUATORS, 10);

      std::string suffix = "command/servo_position";
      pitch_motor_torque_ref_pub_ = nh_.advertise<manipulator_msgs::CommandTorqueServoMotor>("pitching"+suffix, 10);
      left_motor_torque_ref_pub_ = nh_.advertise<manipulator_msgs::CommandTorqueServoMotor>("left"+suffix, 10);
      right_motor_torque_ref_pub_ = nh_.advertise<manipulator_msgs::CommandTorqueServoMotor>("right"+suffix, 10);

      command_timer_ = nh_.createTimer(ros::Duration(0), &MultiObjectiveControllerNode::TimedCommandCallback, this,
                                       true, false);
}


MultiObjectiveControllerNode::~MultiObjectiveControllerNode() { }


void MultiObjectiveControllerNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  // Read parameters from rosparam.
  GetRosParameter(pnh, "mav_position_gain/x",
                  multi_objective_controller_.controller_parameters_.mav_position_gain_.x(),
                  &multi_objective_controller_.controller_parameters_.mav_position_gain_.x());
  GetRosParameter(pnh, "mav_position_gain/y",
                  multi_objective_controller_.controller_parameters_.mav_position_gain_.y(),
                  &multi_objective_controller_.controller_parameters_.mav_position_gain_.y());
  GetRosParameter(pnh, "mav_position_gain/z",
                  multi_objective_controller_.controller_parameters_.mav_position_gain_.z(),
                  &multi_objective_controller_.controller_parameters_.mav_position_gain_.z());
  GetRosParameter(pnh, "mav_velocity_gain/x",
                  multi_objective_controller_.controller_parameters_.mav_velocity_gain_.x(),
                  &multi_objective_controller_.controller_parameters_.mav_velocity_gain_.x());
  GetRosParameter(pnh, "mav_velocity_gain/y",
                  multi_objective_controller_.controller_parameters_.mav_velocity_gain_.y(),
                  &multi_objective_controller_.controller_parameters_.mav_velocity_gain_.y());
  GetRosParameter(pnh, "mav_velocity_gain/z",
                  multi_objective_controller_.controller_parameters_.mav_velocity_gain_.z(),
                  &multi_objective_controller_.controller_parameters_.mav_velocity_gain_.z());
  GetRosParameter(pnh, "mav_attitude_gain/x",
                  multi_objective_controller_.controller_parameters_.mav_attitude_gain_.x(),
                  &multi_objective_controller_.controller_parameters_.mav_attitude_gain_.x());
  GetRosParameter(pnh, "mav_attitude_gain/y",
                  multi_objective_controller_.controller_parameters_.mav_attitude_gain_.y(),
                  &multi_objective_controller_.controller_parameters_.mav_attitude_gain_.y());
  GetRosParameter(pnh, "mav_attitude_gain/z",
                  multi_objective_controller_.controller_parameters_.mav_attitude_gain_.z(),
                  &multi_objective_controller_.controller_parameters_.mav_attitude_gain_.z());
  GetRosParameter(pnh, "mav_angular_rate_gain/x",
                  multi_objective_controller_.controller_parameters_.mav_angular_rate_gain_.x(),
                  &multi_objective_controller_.controller_parameters_.mav_angular_rate_gain_.x());
  GetRosParameter(pnh, "mav_angular_rate_gain/y",
                  multi_objective_controller_.controller_parameters_.mav_angular_rate_gain_.y(),
                  &multi_objective_controller_.controller_parameters_.mav_angular_rate_gain_.y());
  GetRosParameter(pnh, "mav_angular_rate_gain/z",
                  multi_objective_controller_.controller_parameters_.mav_angular_rate_gain_.z(),
                  &multi_objective_controller_.controller_parameters_.mav_angular_rate_gain_.z());
  GetRosParameter(pnh, "ee_position_gain/x",
                  multi_objective_controller_.controller_parameters_.ee_position_gain_.x(),
                  &multi_objective_controller_.controller_parameters_.ee_position_gain_.x());
  GetRosParameter(pnh, "ee_position_gain/y",
                  multi_objective_controller_.controller_parameters_.ee_position_gain_.y(),
                  &multi_objective_controller_.controller_parameters_.ee_position_gain_.y());
  GetRosParameter(pnh, "ee_position_gain/z",
                  multi_objective_controller_.controller_parameters_.ee_position_gain_.z(),
                  &multi_objective_controller_.controller_parameters_.ee_position_gain_.z());
  GetRosParameter(pnh, "ee_velocity_gain/x",
                  multi_objective_controller_.controller_parameters_.ee_velocity_gain_.x(),
                  &multi_objective_controller_.controller_parameters_.ee_velocity_gain_.x());
  GetRosParameter(pnh, "ee_velocity_gain/y",
                  multi_objective_controller_.controller_parameters_.ee_velocity_gain_.y(),
                  &multi_objective_controller_.controller_parameters_.ee_velocity_gain_.y());
  GetRosParameter(pnh, "ee_velocity_gain/z",
                  multi_objective_controller_.controller_parameters_.ee_velocity_gain_.z(),
                  &multi_objective_controller_.controller_parameters_.ee_velocity_gain_.z());
  GetRosParameter(pnh, "arm_joint_angle_gain/pitch",
                  multi_objective_controller_.controller_parameters_.arm_joints_angle_gain_.x(),
                  &multi_objective_controller_.controller_parameters_.arm_joints_angle_gain_.x());
  GetRosParameter(pnh, "arm_joint_angle_gain/left",
                  multi_objective_controller_.controller_parameters_.arm_joints_angle_gain_.y(),
                  &multi_objective_controller_.controller_parameters_.arm_joints_angle_gain_.y());
  GetRosParameter(pnh, "arm_joint_angle_gain/right",
                  multi_objective_controller_.controller_parameters_.arm_joints_angle_gain_.z(),
                  &multi_objective_controller_.controller_parameters_.arm_joints_angle_gain_.z());
  GetRosParameter(pnh, "arm_joint_ang_rate_gain/pitch",
                  multi_objective_controller_.controller_parameters_.arm_joints_ang_rate_gain_.x(),
                  &multi_objective_controller_.controller_parameters_.arm_joints_ang_rate_gain_.x());
  GetRosParameter(pnh, "arm_joint_ang_rate_gain/left",
                  multi_objective_controller_.controller_parameters_.arm_joints_ang_rate_gain_.y(),
                  &multi_objective_controller_.controller_parameters_.arm_joints_ang_rate_gain_.y());
  GetRosParameter(pnh, "arm_joint_ang_rate_gain/right",
                  multi_objective_controller_.controller_parameters_.arm_joints_ang_rate_gain_.z(),
                  &multi_objective_controller_.controller_parameters_.arm_joints_ang_rate_gain_.z());
  GetVehicleParameters(pnh, &multi_objective_controller_.vehicle_parameters_);
  multi_objective_controller_.InitializeParameters();
}


void MultiObjectiveControllerNode::Publish() {
}


void MultiObjectiveControllerNode::CommandPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& pose_msg) {
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


void MultiObjectiveControllerNode::MultiDofJointTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
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


void MultiObjectiveControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {

  if(commands_.empty()){
    ROS_WARN("Commands empty, this should not happen here");
    return;
  }

  const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
  multi_objective_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
  command_timer_.stop();
  if(!command_waiting_times_.empty()){
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}


void MultiObjectiveControllerNode::AerialManipulatorStateCallback(const nav_msgs::OdometryConstPtr& odometry_msg,
                                                                  const sensor_msgs::JointStateConstPtr& joint_state_msg0,
                                                                  const sensor_msgs::JointStateConstPtr& joint_state_msg1,
                                                                  const sensor_msgs::JointStateConstPtr& joint_state_msg2) {

  ROS_INFO_ONCE("MultiObjectiveController got first odometry message.");

  // Update robot states
  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  multi_objective_controller_.SetOdometry(odometry);
  manipulator_msgs::EigenJointsState joints_state;
  std::vector<sensor_msgs::JointState> joint_state_msgs;
  joint_state_msgs.push_back(*(joint_state_msg0.get()));
  joint_state_msgs.push_back(*(joint_state_msg1.get()));
  joint_state_msgs.push_back(*(joint_state_msg2.get()));
  eigenJointsStateFromMsg(joint_state_msgs, &joints_state);
  multi_objective_controller_.SetArmJointsState(joints_state);

  // Run optimization and compute control inputs
  Eigen::VectorXd ref_rotor_velocities;
  Eigen::Vector3d ref_torques;
  multi_objective_controller_.CalculateControlInputs(&ref_rotor_velocities, &ref_torques);

  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  actuator_msg->header.stamp = odometry_msg->header.stamp;

  // Publish commands
  motor_velocity_reference_pub_.publish(actuator_msg);
  pitch_motor_torque_ref_pub_.publish(ref_torques.x());
  left_motor_torque_ref_pub_.publish(ref_torques.y());
  right_motor_torque_ref_pub_.publish(ref_torques.z());
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "multi_objective_controller_node");

  rotors_control::MultiObjectiveControllerNode multi_objective_controller_node;

  ros::spin();

  return 0;
}

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

#include "rotors_control/lee_position_controller_node.h"

namespace rotors_control {

LeePositionControllerNode::LeePositionControllerNode() {

  InitializeParams();

  ros::NodeHandle node_handle(namespace_);

  odometry_sub_ = node_handle.subscribe(odometry_sub_topic_, 10,
                                          &LeePositionControllerNode::OdometryCallback, this);

  motor_velocity_reference_pub_ = node_handle.advertise<mav_msgs::MotorSpeed>(
                                          motor_velocity_reference_pub_topic_, 10);
}

LeePositionControllerNode::~LeePositionControllerNode() { }

void LeePositionControllerNode::InitializeParams() {
  //TODO(burrimi): Read parameters from yaml.

  ros::NodeHandle pnh("~");
  pnh.param<std::string>("robotNamespace", namespace_, kDefaultNamespace);
  pnh.param<std::string>("commandTrajectorySubTopic", command_trajectory_sub_topic_, kDefaultCommandTrajectoryTopic);
  pnh.param<std::string>("odometrySubTopic", odometry_sub_topic_, kDefaultOdometrySubTopic);
  pnh.param<std::string>("motorVelocityCommandPubTopic", motor_velocity_reference_pub_topic_, kDefaultMotorVelocityReferencePubTopic);

}
void LeePositionControllerNode::Publish() {
}

void LeePositionControllerNode::CommandTrajectoryCallback(
    const mav_msgs::CommandTrajectoryConstPtr& trajectory_reference_msg) {
  mav_msgs::EigenCommandTrajectory trajectory;
  mav_msgs::eigenCommandTrajectoryFromMsg(trajectory_reference_msg, &trajectory);
  lee_position_controller_.SetCommandTrajectory(trajectory);
}


void LeePositionControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  ROS_INFO_ONCE("LeePositionController got first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  lee_position_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
  lee_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::MotorSpeed turning_velocities_msg;

  turning_velocities_msg.motor_speed.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    turning_velocities_msg.motor_speed.push_back(ref_rotor_velocities[i]);
  turning_velocities_msg.header.stamp = odometry_msg->header.stamp;

  motor_velocity_reference_pub_.publish(turning_velocities_msg);
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lee_position_controller_node");

  rotors_control::LeePositionControllerNode lee_position_controller_node;

  ros::spin();

  return 0;
}

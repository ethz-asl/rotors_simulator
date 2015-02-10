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

LeePositionControllerNode::LeePositionControllerNode()
    : node_handle_(0){

  InitializeParams();

  node_handle_ = new ros::NodeHandle(namespace_);


  imu_sub_ = node_handle_->subscribe(imu_sub_topic_, 10,
                                     &LeePositionControllerNode::ImuCallback, this);

  odometry_sub_ = node_handle_->subscribe(odometry_sub_topic_, 10,
                                          &LeePositionControllerNode::OdometryCallback, this);

  motor_velocity_reference_pub_ = node_handle_->advertise<mav_msgs::MotorSpeed>(
                                          motor_velocity_reference_pub_topic_, 10);
}

LeePositionControllerNode::~LeePositionControllerNode() {
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void LeePositionControllerNode::InitializeParams() {
  //TODO(burrimi): Read parameters from yaml.

  ros::NodeHandle pnh("~");
  pnh.param<std::string>("robotNamespace", namespace_, kDefaultNamespace);
  pnh.param<std::string>("commandTrajectorySubTopic", command_trajectory_sub_topic_, kDefaultCommandTrajectoryTopic);
  pnh.param<std::string>("imuSubTopic", imu_sub_topic_, kDefaultImuSubTopic);
  pnh.param<std::string>("odometrySubTopic", odometry_sub_topic_, kDefaultOdometrySubTopic);
  pnh.param<std::string>("motorVelocityCommandPubTopic", motor_velocity_reference_pub_topic_, kDefaultMotorVelocityReferencePubTopic);

}
void LeePositionControllerNode::Publish() {
}

void LeePositionControllerNode::CommandTrajectoryCallback(
    const mav_msgs::CommandTrajectoryConstPtr& trajectory_reference_msg) {

  Eigen::Vector3d position_reference(trajectory_reference_msg->position[0],
                                     trajectory_reference_msg->position[1],
                                     trajectory_reference_msg->position[2]);

  lee_position_controller_.SetPositionReference(position_reference);

  Eigen::Vector3d velocity_reference(trajectory_reference_msg->velocity[0],
                                     trajectory_reference_msg->velocity[1],
                                     trajectory_reference_msg->velocity[2]);
  lee_position_controller_.SetVelocityReference(velocity_reference);

  Eigen::Vector3d acceleration_reference(trajectory_reference_msg->acceleration[0],
                                         trajectory_reference_msg->acceleration[1],
                                         trajectory_reference_msg->acceleration[2]);
  lee_position_controller_.SetAccelerationReference(acceleration_reference);

  Eigen::Vector3d jerk_reference(trajectory_reference_msg->jerk[0],
                                 trajectory_reference_msg->jerk[1],
                                 trajectory_reference_msg->jerk[2]);
  lee_position_controller_.SetJerkReference(jerk_reference);

  lee_position_controller_.SetYawReference(trajectory_reference_msg->yaw);
  lee_position_controller_.SetYawRateReference(trajectory_reference_msg->yaw_rate);
}


void LeePositionControllerNode::OdometryCallback(
    const nav_msgs::OdometryConstPtr odometry_msg) {

  ROS_INFO_ONCE("got first odometry message.");

  Eigen::Vector3d position(odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y,
                           odometry_msg->pose.pose.position.z);
  lee_position_controller_.SetPosition(position);

  Eigen::Quaternion<double> q_W_I(odometry_msg->pose.pose.orientation.w, odometry_msg->pose.pose.orientation.x,
                                        odometry_msg->pose.pose.orientation.y, odometry_msg->pose.pose.orientation.z);
  lee_position_controller_.SetAttitude(q_W_I);

  // Convert body linear velocity from odometry message into world frame.
  Eigen::Vector3d velocity_I(odometry_msg->twist.twist.linear.x,
                             odometry_msg->twist.twist.linear.y,
                             odometry_msg->twist.twist.linear.z);
  Eigen::Vector3d velocity_W;
  velocity_W = q_W_I.toRotationMatrix() * velocity_I;
  lee_position_controller_.SetVelocity(velocity_W);

  // Get angulare rates from odometry message.
  Eigen::Vector3d angular_rate_I(odometry_msg->twist.twist.angular.x, odometry_msg->twist.twist.angular.y,
                           odometry_msg->twist.twist.angular.z);

  // We set the body angular rates.
  lee_position_controller_.SetAngularRate(angular_rate_I);

  Eigen::VectorXd ref_rotor_velocities;
  lee_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  mav_msgs::MotorSpeed turning_velocities_msg;

  turning_velocities_msg.motor_speed.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    turning_velocities_msg.motor_speed.push_back(ref_rotor_velocities[i]);
  turning_velocities_msg.header.stamp = odometry_msg->header.stamp;

  motor_velocity_reference_pub_.publish(turning_velocities_msg);
}


void LeePositionControllerNode::ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {

  ROS_INFO_ONCE("Received first IMU message.");


  Eigen::Vector3d angular_rate(imu_msg->angular_velocity.x,
                               imu_msg->angular_velocity.y,
                               imu_msg->angular_velocity.z);
  lee_position_controller_.SetAngularRate(angular_rate);
  // imu->linear_acceleration;
}


}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lee_position_controller_node");

  rotors_control::LeePositionControllerNode lee_position_controller_node;

  ros::spin();

  return 0;
}

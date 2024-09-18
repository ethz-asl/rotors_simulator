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


#ifndef ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CONTROLLER_NODE_HPP
#define ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CONTROLLER_NODE_HPP

#include <eigen3/Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mav_msgs/msg/roll_pitch_yawrate_thrust.hpp>
#include <mav_msgs/msg/actuators.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <actuator_msgs/msg/actuators.hpp>
//#include <ros/callback_queue.h>

#include "rotors_control/common.hpp"
#include "rotors_control/roll_pitch_yawrate_thrust_controller.hpp"

namespace rotors_control {

class RollPitchYawrateThrustControllerNode : public rclcpp::Node 
{
 public:
  RollPitchYawrateThrustControllerNode();
  ~RollPitchYawrateThrustControllerNode();

  void InitializeParams();
  void Publish();

 private:

  RollPitchYawrateThrustController roll_pitch_yawrate_thrust_controller_;
  rclcpp::Node::SharedPtr node_handle;

  std::string namespace_;

  // subscribers
  rclcpp::Subscription<mav_msgs::msg::RollPitchYawrateThrust>::ConstSharedPtr cmd_roll_pitch_yawrate_thrust_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr odometry_sub_;
  
  //publishers
  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr motor_velocity_reference_pub_;

  void RollPitchYawrateThrustCallback(const mav_msgs::msg::RollPitchYawrateThrust::ConstSharedPtr& roll_pitch_yawrate_thrust_reference_msg);

  void OdometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odometry_msg);
};
}

#endif // ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CONTROLLER_NODE_HPP

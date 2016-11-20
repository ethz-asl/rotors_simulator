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


#include "rotors_joy_interface/joy.h"

#include <mav_msgs/default_topics.h>

Joy::Joy() {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ctrl_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust> (
    mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 10);

  control_msg_.roll = 0;
  control_msg_.pitch = 0;
  control_msg_.yaw_rate = 0;
  control_msg_.thrust.x = 0;
  control_msg_.thrust.y = 0;
  control_msg_.thrust.z = 0;
  current_yaw_vel_ = 0;

  pnh.param("axis_roll_", axes_.roll, 0);
  pnh.param("axis_pitch_", axes_.pitch, 1);
  pnh.param("axis_thrust_", axes_.thrust, 2);

  pnh.param("axis_direction_roll", axes_.roll_direction, -1);
  pnh.param("axis_direction_pitch", axes_.pitch_direction, 1);
  pnh.param("axis_direction_thrust", axes_.thrust_direction, 1);

  pnh.param("max_v_xy", max_.v_xy, 1.0);  // [m/s]
  pnh.param("max_roll", max_.roll, 10.0 * M_PI / 180.0);  // [rad]
  pnh.param("max_pitch", max_.pitch, 10.0 * M_PI / 180.0);  // [rad]
  pnh.param("max_yaw_rate", max_.rate_yaw, 45.0 * M_PI / 180.0);  // [rad/s]
  pnh.param("max_thrust", max_.thrust, 30.0);  // [N]

  pnh.param("v_yaw_step", v_yaw_step_, 0.05);  // [rad/s]

  pnh.param("is_fixed_wing", is_fixed_wing_, false);

  pnh.param("button_yaw_left_", buttons_.yaw_left, 3);
  pnh.param("button_yaw_right_", buttons_.yaw_right, 4);
  pnh.param("button_ctrl_enable_", buttons_.ctrl_enable, 5);
  pnh.param("button_ctrl_mode_", buttons_.ctrl_mode, 10);
  pnh.param("button_takeoff_", buttons_.takeoff, 7);
  pnh.param("button_land_", buttons_.land, 8);

  namespace_ = nh_.getNamespace();
  joy_sub_ = nh_.subscribe("joy", 10, &Joy::JoyCallback, this);
}

void Joy::StopMav() {
  control_msg_.roll = 0;
  control_msg_.pitch = 0;
  control_msg_.yaw_rate = 0;
  control_msg_.thrust.x = 0;
  control_msg_.thrust.y = 0;
  control_msg_.thrust.z = 0;
}

void Joy::JoyCallback(const sensor_msgs::JoyConstPtr& msg) {
  current_joy_ = *msg;
  control_msg_.roll = msg->axes[axes_.roll] * max_.roll * axes_.roll_direction;
  control_msg_.pitch = msg->axes[axes_.pitch] * max_.pitch * axes_.pitch_direction;

  if (msg->buttons[buttons_.yaw_left]) {
    current_yaw_vel_ = max_.rate_yaw;
  }
  else if (msg->buttons[buttons_.yaw_right]) {
    current_yaw_vel_ = -max_.rate_yaw;
  }
  else {
    current_yaw_vel_ = 0;
  }
  control_msg_.yaw_rate = current_yaw_vel_;

  if (is_fixed_wing_) {
    double thrust = msg->axes[axes_.thrust] * axes_.thrust_direction;
    control_msg_.thrust.x = (thrust >= 0.0) ? thrust : 0.0;
  }
  else {
    control_msg_.thrust.z = (msg->axes[axes_.thrust] + 1) * max_.thrust / 2.0 * axes_.thrust_direction;
  }

  ros::Time update_time = ros::Time::now();
  control_msg_.header.stamp = update_time;
  control_msg_.header.frame_id = "rotors_joy_frame";
  Publish();
}

void Joy::Publish() {
  ctrl_pub_.publish(control_msg_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_joy_interface");
  Joy joy;

  ros::spin();

  return 0;
}

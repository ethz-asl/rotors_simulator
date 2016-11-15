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


#ifndef ROTORS_JOY_INTERFACE_JOY_H_
#define ROTORS_JOY_INTERFACE_JOY_H_

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

struct Axes {
  int roll;
  int pitch;
  int thrust;
  int roll_direction;
  int pitch_direction;
  int thrust_direction;
};

struct Buttons {
  int takeoff;
  int land;
  int ctrl_enable;
  int ctrl_mode;
  int yaw_left;
  int yaw_right;
};

struct Max {
  double v_xy;
  double roll;
  double pitch;
  double rate_yaw;
  double thrust;
};

class Joy {
  typedef sensor_msgs::Joy::_buttons_type ButtonType;

 private:
  ros::NodeHandle nh_;
  ros::Publisher ctrl_pub_;
  ros::Subscriber joy_sub_;

  std::string namespace_;

  Axes axes_;
  Buttons buttons_;

  mav_msgs::RollPitchYawrateThrust control_msg_;
  geometry_msgs::PoseStamped pose_;
  sensor_msgs::Joy current_joy_;

  Max max_;

  double current_yaw_vel_;
  double v_yaw_step_;

  bool is_fixed_wing_;

  void StopMav();

  void JoyCallback(const sensor_msgs::JoyConstPtr& msg);
  void Publish();

 public:
  Joy();
};

#endif // ROTORS_JOY_INTERFACE_JOY_H_

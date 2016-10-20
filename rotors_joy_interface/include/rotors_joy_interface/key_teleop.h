/*
 * Copyright 2016 Pavel Vechersky, ASL, ETH Zurich, Switzerland
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

#ifndef ROTORS_JOY_INTERFACE_KEY_TELEOP_H_
#define ROTORS_JOY_INTERFACE_KEY_TELEOP_H_

#include <ros/ros.h>
#include <termios.h>

#include <mav_msgs/default_topics.h>
#include <mav_msgs/RollPitchYawrateThrust.h>

namespace rotors_teleop {
// File descriptor constant
static constexpr int STDIN_FD = 0;

// Key assignment constants
static constexpr int KEYCODE_THRUST_UP = 0x77;      // 'w'
static constexpr int KEYCODE_THRUST_DOWN = 0x73;    // 's'
static constexpr int KEYCODE_YAW_LEFT = 0x61;       // 'a'
static constexpr int KEYCODE_YAW_RIGHT = 0x64;      // 'd'
static constexpr int KEYCODE_PITCH_UP = 0x42;       // up arrow
static constexpr int KEYCODE_PITCH_DOWN = 0x41;     // down arrow
static constexpr int KEYCODE_ROLL_LEFT = 0x44;      // left arrow
static constexpr int KEYCODE_ROLL_RIGHT = 0x43;     // right arrow
static constexpr int KEYCODE_RESET = 0x52;          // 'R'
static constexpr int KEYCODE_QUIT = 0x51;           // 'Q'

// Default values
static constexpr bool kDefaultIsFixedWing = true;
static constexpr double kDefaultControlTimeout = 0.5;
static constexpr double kDefaultSensitivity = 0.05;

class KeyTeleop {
 public:
  KeyTeleop();
  virtual ~KeyTeleop();

  void ConfigureTerminalInput();
  void KeyInputLoop();
  void Shutdown();

 private:
  // ROS interface
  ros::NodeHandle nh_;
  ros::Publisher ctrl_pub_;

  // Type of vehicle
  bool is_fixed_wing_;

  // Control values
  double roll_;
  double pitch_;
  double yaw_;
  double thrust_;

  // Controls sensitivity
  double sensitivity_;

  // Control timers
  double last_roll_time_;
  double last_pitch_time_;
  double last_yaw_time_;
  double control_timeout_;

  // Control message
  mav_msgs::RollPitchYawrateThrust control_msg_;

  // Object to store original input terminal attributes
  termios attributes_old_;
};
}

#endif // ROTORS_JOY_INTERFACE_KEY_TELEOP_H_

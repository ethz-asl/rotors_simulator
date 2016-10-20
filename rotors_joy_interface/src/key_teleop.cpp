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

#include "rotors_joy_interface/key_teleop.h"

namespace rotors_teleop {

KeyTeleop::KeyTeleop() {
  ConfigureTerminalInput();

  ros::NodeHandle pnh("~");

  std::string control_pub_topic;

  pnh.param("control_pub_topic", control_pub_topic,
    std::string(mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST));
  pnh.param("control_timeout", control_timeout_, kDefaultControlTimeout);
  pnh.param("is_fixed_wing", is_fixed_wing_, kDefaultIsFixedWing);
  pnh.param("controls_sensitivity", sensitivity_, kDefaultSensitivity);

  ctrl_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust>
    (control_pub_topic, 1);
}

KeyTeleop::~KeyTeleop() {
  Shutdown();
}

void KeyTeleop::ConfigureTerminalInput() {
  // Get the console attributes
  termios attributes_new;
  tcgetattr(STDIN_FD, &attributes_old_);
  memcpy(&attributes_new, &attributes_old_, sizeof(termios));

  // Disable canonical mode and echoing of characters
  attributes_new.c_lflag &= ~(ICANON | ECHO);

  // Minimum number of characters
  attributes_new.c_cc[VMIN] = 1;
  attributes_new.c_cc[VEOL] = 1;
  attributes_new.c_cc[VEOF] = 2;

  // Set the new attributes
  tcsetattr(STDIN_FD, TCSANOW, &attributes_new);

  ROS_INFO("Terminal configured for keyboard teleop");
}

void KeyTeleop::KeyInputLoop() {
  char c;

  while (ros::ok()) {
    if (read(STDIN_FD, &c, 1) < 0) {
      ROS_ERROR("Error reading input from the terminal");
      Shutdown();
    }

    double curr_time = ros::Time::now().toSec();

    switch(c) {
      case KEYCODE_ROLL_LEFT:
        roll_ -= sensitivity_;
        roll_ = (roll_ < -1.0) ? -1.0 : roll_;
        last_roll_time_ = curr_time;
        break;
      case KEYCODE_ROLL_RIGHT:
        roll_ += sensitivity_;
        roll_ = (roll_ > 1.0) ? 1.0 : roll_;
        last_roll_time_ = curr_time;
        break;
      case KEYCODE_PITCH_UP:
        pitch_ -= sensitivity_;
        pitch_ = (pitch_ < -1.0) ? -1.0 : pitch_;
        last_pitch_time_ = curr_time;
        break;
      case KEYCODE_PITCH_DOWN:
        pitch_ += sensitivity_;
        pitch_ = (pitch_ > 1.0) ? 1.0 : pitch_;
        last_pitch_time_ = curr_time;
        break;
      case KEYCODE_YAW_LEFT:
        yaw_ -= sensitivity_;
        yaw_ = (yaw_ < -1.0) ? -1.0 : yaw_;
        last_yaw_time_ = curr_time;
        break;
      case KEYCODE_YAW_RIGHT:
        yaw_ += sensitivity_;
        yaw_ = (yaw_ > 1.0) ? 1.0 : yaw_;
        last_yaw_time_ = curr_time;
        break;
      case KEYCODE_THRUST_UP:
        thrust_ += sensitivity_;
        thrust_ = (thrust_ > 1.0) ? 1.0 : thrust_;
        break;
      case KEYCODE_THRUST_DOWN:
        thrust_ -= sensitivity_;
        thrust_ = (thrust_ < 0.0) ? 0.0 : thrust_;
        break;
      case KEYCODE_RESET:
        ROS_INFO("Keyboard teleop - resetting controls");
        roll_ = 0.0;
        pitch_ = 0.0;
        yaw_ = 0.0;
        thrust_ = 0.0;
      case KEYCODE_QUIT:
        ROS_INFO("Keyboard teleop - shuttind down");
        Shutdown();
        break;
      default:
        break;
    }

    // If it has been too long since we received certain commands we set those controls to zero
    roll_ = ((curr_time - last_roll_time_) > control_timeout_) ? 0.0 : roll_;
    pitch_ = ((curr_time - last_pitch_time_) > control_timeout_) ? 0.0 : pitch_;
    yaw_ = ((curr_time - last_yaw_time_) > control_timeout_) ? 0.0 : yaw_;

    ros::Time current_time = ros::Time::now();

    control_msg_.roll = roll_;
    control_msg_.pitch = pitch_;
    control_msg_.yaw_rate = yaw_;

    if (is_fixed_wing_)
      control_msg_.thrust.x = thrust_;
    else
      control_msg_.thrust.z = thrust_;

    control_msg_.header.stamp.sec = current_time.sec;
    control_msg_.header.stamp.nsec = current_time.nsec;

    ctrl_pub_.publish(control_msg_);

    ros::spinOnce();
  }
}

void KeyTeleop::Shutdown() {
  // Restore the old attributes
  tcsetattr(STDIN_FD, TCSANOW, &attributes_old_);
  ros::shutdown();
}
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_key_teleop");

  rotors_teleop::KeyTeleop key_teleop;
  key_teleop.KeyInputLoop();

  return 0;
}

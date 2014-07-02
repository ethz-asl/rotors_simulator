//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>
// All rights reserved.
//
// TODO(ff): Enter some license
//==============================================================================

#ifndef MAV_JOY_INTERFACE_JOY_H_
#define MAV_JOY_INTERFACE_JOY_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <mav_msgs/CommandAttitudeThrust.h>
#include <geometry_msgs/PoseStamped.h>

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

  mav_msgs::CommandAttitudeThrust control_msg_;
  geometry_msgs::PoseStamped pose_;
  sensor_msgs::Joy current_joy_;

  Max max_;

  double current_yaw_vel_;
  double v_yaw_step_;

  void StopMav();

  void JoyCallback(const sensor_msgs::JoyConstPtr& msg);
  void Publish();

 public:
  Joy();
};

#endif /* MAV_JOY_INTERFACE_JOY_H_ */

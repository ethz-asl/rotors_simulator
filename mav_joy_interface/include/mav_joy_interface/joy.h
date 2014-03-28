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
#include <mav_comm/mav_ctrl.h>
#include <geometry_msgs/PoseStamped.h>


struct Axes
{
  int x;
  int y;
  int z;
  int yaw;
};

struct ButtonAssignment
{
  int takeoff;
  int ctrl_enable;
  int ctrl_mode;
  int yaw_left;
  int yaw_right;
};

struct VMax
{
  double xy;
  double z;
  double yaw;
};

class Joy
{
  typedef sensor_msgs::Joy::_buttons_type ButtonType;

private:
  ros::NodeHandle nh_;
  ros::Publisher ctrl_pub_;
  ros::Subscriber joy_sub_;

  std::string namespace_;

  Axes axes_;
  ButtonAssignment buttons_;

  geometry_msgs::PoseStamped pose_;
  sensor_msgs::Joy current_joy_;

  VMax v_max_;


  bool setDynParam(const std::string & param_string);

  bool sendMavCommand(const sensor_msgs::JoyConstPtr & msg);
  void stopMav();

  void joyCallback(const sensor_msgs::JoyConstPtr & msg);
  void publish();


public:
  Joy();
};

#endif /* MAV_JOY_INTERFACE_JOY_H_ */

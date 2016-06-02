#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>

#include <eigen_conversions/eigen_msg.h>

// Use the structure definitions from the rotors_joy_interface 
// #include "rotors_joy_interface/joy.h"

#define DEG2RAD(x) ((x) / 180.0 * M_PI)

ros::Publisher trajectory_pub;
ros::Subscriber odom_sub, joy_sub;
nav_msgs::Odometry odom_msg;
sensor_msgs::Joy joy_msg;

bool joy_msg_ready = false;

int axis_roll  , axis_pitch, 
    axis_thrust, axis_yaw;
int axis_direction_roll, 
    axis_direction_pitch, 
    axis_direction_thrust, 
    axis_direction_yaw;
 
double max_vel,
       max_yawrate;


void joy_callback(const sensor_msgs::JoyConstPtr& msg);
void odom_callback(const nav_msgs::OdometryConstPtr& msg);

int main(int argc, char** argv) {

  ros::init(argc, argv, "velocity_control_with_joy");
  ros::NodeHandle nh("~");

  // Continuously publish waypoints.
  trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
                    mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  // Subscribe to joystick messages
  joy_sub = nh.subscribe("joy", 10, &joy_callback);

  // Subscribe to gt odometry messages
  odom_sub = nh.subscribe("odom", 10, &odom_callback);

  ROS_INFO("Started velocity_control_with_joy.");

  nh.param("axis_roll_"  , axis_roll, 3);
  nh.param("axis_pitch_" , axis_pitch, 4);
  nh.param("axis_yaw_"   , axis_yaw, 0);
  nh.param("axis_thrust_", axis_thrust, 1);

  nh.param("axis_direction_roll_"  , axis_direction_roll, -1);
  nh.param("axis_direction_pitch_" , axis_direction_pitch, 1);
  nh.param("axis_direction_yaw_"   , axis_direction_yaw, 1);
  nh.param("axis_direction_thrust_", axis_direction_thrust, -1);
 
  nh.param("max_vel", max_vel, 1.0);
  nh.param("max_yawrate", max_yawrate, DEG2RAD(45));
  
  ros::spin();
}

void joy_callback(const sensor_msgs::JoyConstPtr& msg){
  joy_msg = *msg;
  joy_msg_ready = true;
}

void odom_callback(const nav_msgs::OdometryConstPtr& msg){
  
  if(joy_msg_ready == false)
    return;
 
  odom_msg = *msg;
 
  static bool init_pose_set = false;
  static ros::Time prev_time = ros::Time::now();
  static Eigen::Vector3d init_position;
  static double init_yaw;

  if(init_pose_set == false){
    Eigen::Affine3d eigen_affine;
    tf::poseMsgToEigen(odom_msg.pose.pose, eigen_affine);
    init_position = eigen_affine.matrix().topRightCorner<3, 1>();
    init_yaw = eigen_affine.matrix().topLeftCorner<3, 3>().eulerAngles(0, 1, 2)(2);
    init_pose_set = true;
  }

  double dt = (ros::Time::now() - prev_time).toSec();
  prev_time = ros::Time::now();

  double yaw    = joy_msg.axes[axis_yaw]    * axis_direction_yaw;
  double roll   = joy_msg.axes[axis_roll]   * axis_direction_roll;
  double pitch  = joy_msg.axes[axis_pitch]  * axis_direction_pitch;
  double thrust = joy_msg.axes[axis_thrust] * axis_direction_thrust;

  Eigen::Vector3d desired_dposition( cos(init_yaw) * pitch - sin(init_yaw) * roll, 
                                     sin(init_yaw) * pitch + cos(init_yaw) * roll, 
                                     thrust);
  init_position += desired_dposition * max_vel * dt;

  init_yaw = init_yaw + max_yawrate * yaw * dt;

  static trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();
  trajectory_msg.header.seq++;

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(init_position,
      init_yaw, &trajectory_msg);

  trajectory_msg.points[0].time_from_start = ros::Duration(1.0);

  trajectory_pub.publish(trajectory_msg);
}

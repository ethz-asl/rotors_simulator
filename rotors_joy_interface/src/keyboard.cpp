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
#include <mav_msgs/conversions.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <thread>
#include <mutex>

#include <cstdio>
#include <unistd.h>
#include <termios.h>

ros::Publisher ctrl_pub;
bool has_new_kbd_input = false;
char c;

void kbd_listener() {
  while (ros::ok()) {
    c = std::getchar();
    printf("%c \n",c);
    has_new_kbd_input = true;
  }
}

bool got_odom = false;
nav_msgs::Odometry odom;
void odom_callback(const nav_msgs::OdometryConstPtr& odometry_msg) {
  odom = *odometry_msg;
  got_odom = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_keyboard_interface");
  ros::NodeHandle nh;

  ros::Subscriber odometry_sub = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1, odom_callback);
  ctrl_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
             mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  ros::Rate rate(30);
  while (ros::ok() && !got_odom) {
    ros::spinOnce();
    rate.sleep();
  }

  geometry_msgs::Pose ref_pose = odom.pose.pose;
  
  struct termios old_tio, new_tio;
  
  // get the terminal settings for stdin 
  tcgetattr(STDIN_FILENO, &old_tio);
  
  // we want to keep the old setting to restore them a the end 
  new_tio = old_tio;

  // disable canonical mode (buffered i/o) and local echo 
  new_tio.c_lflag &=(~ICANON & ~ECHO);

  // set the new settings immediately 
  tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

  std::thread kbd_thread(kbd_listener);
  
  while (ros::ok()) {
    
    if (has_new_kbd_input) {
      
      trajectory_msgs::MultiDOFJointTrajectory control_msg;
      control_msg.header.stamp = ros::Time::now();
      
      Eigen::Vector3d target_position; 
      target_position << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z; 
      // current position
      
      double target_yaw = tf::getYaw(odom.pose.pose.orientation); 
      // current yaw, counter-clockwise positive 
      
      if (c == 'W') {
        target_position.x() += 0.5;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(target_position, target_yaw, &control_msg);
        
      } else if (c == 'E') {
        target_position.x() -= 0.5;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(target_position, target_yaw, &control_msg);
      
      } else if (c == 'A') {
        target_position.y() += 0.5;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(target_position, target_yaw, &control_msg);
      
      } else if (c == 'F') {
        target_position.y() -= 0.5;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(target_position, target_yaw, &control_msg);
      
      } else if (c == 'Q') {
        target_position.z() += 0.5;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(target_position, target_yaw, &control_msg);
      
      } else if (c == 'R') {
        target_position.z() -= 0.5;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(target_position, target_yaw, &control_msg);
      
      } else if (c == 'S') {
        target_yaw += 30*M_PI/180;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(target_position, target_yaw, &control_msg);
        
      } else if (c == 'D') {
        target_yaw -= 30*M_PI/180;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(target_position, target_yaw, &control_msg);
      }
  
      has_new_kbd_input = false;
      
      ctrl_pub.publish(control_msg);
    }
    
    ros::spinOnce();
    rate.sleep();
  }
  
  
  // restore the former settings 
  tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
  
  kbd_thread.join();
   
  return 0;
}

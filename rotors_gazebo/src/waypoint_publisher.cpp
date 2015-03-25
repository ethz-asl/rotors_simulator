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

#include <fstream>
#include <iostream>

#include <mav_msgs/CommandTrajectoryPositionYaw.h>
#include <ros/ros.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "waypoint_publisher");
  ros::NodeHandle nh("");
  ros::Publisher trajectory_pub = nh.advertise<mav_msgs::CommandTrajectoryPositionYaw>(
      "command/trajectory_position_yaw", 10);

  ROS_INFO("Started waypoint_publisher.");

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  if (args.size() != 5) {
    ROS_ERROR(
        "Usage: waypoint_publisher <x> <y> <z> <yaw> \n");
    return -1;
  }

  const float DEG_2_RAD = M_PI / 180.0;

  mav_msgs::CommandTrajectoryPositionYaw trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();
  trajectory_msg.position.x = std::stof(args.at(1));
  trajectory_msg.position.y = std::stof(args.at(2));
  trajectory_msg.position.z = std::stof(args.at(3));
  trajectory_msg.yaw = std::stof(args.at(4)) * DEG_2_RAD;

  // Wait for some time to create the ros publisher.
  ros::Duration(1.0).sleep();

  ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
           nh.getNamespace().c_str(),
           trajectory_msg.position.x,
           trajectory_msg.position.y,
           trajectory_msg.position.z);
  trajectory_pub.publish(trajectory_msg);

  return 0;
}

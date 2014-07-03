/*
 * Copyright (C) 2014 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Pascal Gohl, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Sammy Omari, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * This software is released to the Contestants of the european 
 * robotics challenges (EuRoC) for the use in stage 1. (Re)-distribution, whether 
 * in parts or entirely, is NOT PERMITTED. 
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 */

#include "ros/ros.h"
#include "gazebo_msgs/GetModelState.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char** argv) {

  ros::init(argc, argv, "gazebo_model_publisher");
  if (argc != 3) {
    ROS_INFO("usage: gazebo_pose_publisher <robot_name> <publish_topic>");
    return 1;
  }

  ros::NodeHandle nh;
  ros::Rate loop_rate(100);

  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>(std::string(argv[2]), 10);
  ros::ServiceClient gazebo_model_state = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  gazebo_msgs::GetModelState model_state;
  model_state.request.model_name = std::string(argv[1]);
  geometry_msgs::PoseStamped pose_msg;

  while (ros::ok()) {
    gazebo_model_state.call(model_state);
    pose_msg.pose = model_state.response.pose;
    ros::Time update_time = ros::Time::now();
    pose_msg.header.stamp = update_time;
    pose_pub.publish(pose_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

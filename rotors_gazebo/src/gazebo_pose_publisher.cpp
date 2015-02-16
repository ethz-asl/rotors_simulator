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

#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

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

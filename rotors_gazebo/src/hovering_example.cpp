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

#include <thread>
#include <chrono>

#include <mav_msgs/CommandTrajectoryPositionYaw.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

bool waypoint_changed(const mav_msgs::CommandTrajectoryPositionYaw& trajectory_prev,
                      const mav_msgs::CommandTrajectoryPositionYaw& trajectory) {
  if(trajectory_prev.position.x != trajectory.position.x ||
      trajectory_prev.position.y != trajectory.position.y ||
      trajectory_prev.position.z != trajectory.position.z) {
    return true;
  }
  return false;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "hovering_example");
  ros::NodeHandle nh;
  ros::Publisher trajectory_pub = nh.advertise<mav_msgs::CommandTrajectoryPositionYaw>(
      "command/trajectory_position_yaw", 10);
  ROS_INFO("Started hovering example.");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  }
  else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(5.0).sleep();

  mav_msgs::CommandTrajectoryPositionYaw trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();
  trajectory_msg.position.x = 0.0;
  trajectory_msg.position.y = 0.0;
  trajectory_msg.position.z = 1.0;

  ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
           nh.getNamespace().c_str(),
           trajectory_msg.position.x,
           trajectory_msg.position.y,
           trajectory_msg.position.z);
  trajectory_pub.publish(trajectory_msg);

  ros::spin();
}

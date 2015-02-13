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

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <mav_msgs/CommandTrajectory.h>
#include <rotors_control/calibrateKF.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "hovering_example");
  ros::NodeHandle nh;
  ros::Publisher trajectory_pub = nh.advertise<mav_msgs::CommandTrajectory>("command/trajectory", 10);

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

  ROS_INFO("Fly to position [0 0 1].");
  mav_msgs::CommandTrajectory trajectory_msg;
  trajectory_msg.position.x = 0;
  trajectory_msg.position.y = 0;
  trajectory_msg.position.z = 1;
  trajectory_msg.yaw = 0;
  trajectory_pub.publish(trajectory_msg);

  rotors_control::calibrateKFRequest calibrate_req;
  rotors_control::calibrateKFResponse calibrate_res;
  calibrate_req.calibration_time = 5.0;
  ros::Duration(10.0).sleep();


 if(!ros::service::call("/firefly/StartCalibrateKF", calibrate_req, calibrate_res)){
   ROS_WARN("Failed to calibrate");
 }else{
   ROS_INFO("Calibrating Disturbances Observer... ");
 }

 ros::Duration(calibrate_req.calibration_time + 0.5).sleep();

 ROS_INFO("Fly to position [1 1 1].");
 trajectory_msg.position.x = 1;
 trajectory_msg.position.y = 1;
 trajectory_msg.position.z = 1;
 trajectory_msg.yaw = M_PI_2;
 trajectory_pub.publish(trajectory_msg);



  ros::spin();
}

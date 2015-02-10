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

#include <mav_msgs/CommandTrajectory.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>


bool sim_running = false;

void callback(const sensor_msgs::ImuPtr& /*msg*/) {
  sim_running = true;
}

class WaypointWithTime {
 public:
  WaypointWithTime()
      : waiting_time(0) {
  }

  WaypointWithTime(double t, float x, float y, float z, float yaw)
      : waiting_time(t) {
    wp.position.x = x;
    wp.position.y = y;
    wp.position.z = z;
    wp.yaw = yaw;
  }

  mav_msgs::CommandTrajectory wp;
  double waiting_time;
};

int main(int argc, char** argv) {

  ros::init(argc, argv, "waypoint_publisher");
  ros::NodeHandle nh;

  ROS_INFO("Started waypoint_publisher.");

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  if (args.size() != 2) {
    ROS_ERROR(
        "Usage: waypoint_publisher <waypoint_file> (one per line, space separated: wait_time [s] x[m] y[m] z[m] yaw[deg])");
    return -1;
  }

  std::vector<WaypointWithTime> waypoints;
  const float DEG_2_RAD = M_PI / 180.0;

  std::ifstream wp_file(args.at(1).c_str());

  if (wp_file.is_open()) {
    double t, x, y, z, yaw;
    // Only read complete waypoints.
    while (wp_file >> t >> x >> y >> z >> yaw) {
      waypoints.push_back(WaypointWithTime(t, x, y, z, yaw * DEG_2_RAD));
    }
    wp_file.close();
    ROS_INFO("Read %d waypoints.", (int )waypoints.size());
  }

  else {
    ROS_ERROR_STREAM("Unable to open poses file: " << args.at(1));
    return -1;
  }

//  waypoints.push_back(WaypointWithTime(20, 10, 10, 5, 22.5 * DEG_2_RAD));
//  waypoints.push_back(WaypointWithTime(15, 10,  0, 5, 45.0 * DEG_2_RAD));
//  waypoints.push_back(WaypointWithTime(15,  0, 10, 1, 67.5 * DEG_2_RAD));
//  waypoints.push_back(WaypointWithTime(15,  0,  0, 1, 90.0 * DEG_2_RAD));

//  waypoints.push_back(WaypointWithTime(15, 2, 2, 1.5, 0 * DEG_2_RAD));
//  waypoints.push_back(WaypointWithTime(15, 2, 0, 1.5, 0 * DEG_2_RAD));
//  waypoints.push_back(WaypointWithTime(15, 0, 2, 1.0,   0 * DEG_2_RAD));
//  waypoints.push_back(WaypointWithTime(15, 0, 0, 1.0,   0 * DEG_2_RAD));

  ros::Subscriber sub = nh.subscribe("imu", 10, &callback);

  ros::Publisher wp_pub = nh.advertise<mav_msgs::CommandTrajectory>("waypoint", 10);
  mav_msgs::CommandTrajectory wp_msg;

  ROS_INFO("Wait for simulation to become ready...");

  while (!sim_running && ros::ok()) {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  ROS_INFO("...ok");

  // Wait for 30s such that everything can settle and the helicopter flies to initial position.
  ros::Duration(30).sleep();

  ROS_INFO("Start publishing waypoints.");
  for (size_t i = 0; i < waypoints.size(); ++i) {
    const WaypointWithTime& wp = waypoints[i];
    ROS_INFO("Publishing x=%f y=%f z=%f yaw=%f, and wait for %fs.", wp.wp.position.x, wp.wp.position.y,
             wp.wp.position.z, wp.wp.yaw, wp.waiting_time);
    wp_pub.publish(wp.wp);
    ros::Duration(wp.waiting_time).sleep();
  }

  return 0;
}

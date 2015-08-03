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

#include <Eigen/Geometry>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

bool sim_running = false;

void callback(const sensor_msgs::ImuPtr& /*msg*/) {
  sim_running = true;
}

class WaypointWithTime {
 public:
  WaypointWithTime()
      : waiting_time(0) {
  }

  WaypointWithTime(double t, float x, float y, float z, float _yaw)
      : position(x, y, z), yaw(_yaw), waiting_time(t) {
  }

  Eigen::Vector3d position;
  double yaw;
  double waiting_time;
};

int main(int argc, char** argv) {

  ros::init(argc, argv, "waypoint_publisher");
  ros::NodeHandle nh;

  ROS_INFO("Started waypoint_publisher.");

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  if (args.size() != 2 && args.size() != 3) {
    ROS_ERROR("Usage: waypoint_publisher <waypoint_file>"
        "\nThe waypoint file should be structured as: space separated: wait_time [s] x[m] y[m] z[m] yaw[deg])");
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

  // The IMU is used, to determine if the simulator is running or not.
  ros::Subscriber sub = nh.subscribe("imu", 10, &callback);

  ros::Publisher wp_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  ROS_INFO("Wait for simulation to become ready...");

  while (!sim_running && ros::ok()) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  ROS_INFO("...ok");

  // Wait for 30s such that everything can settle and the mav flies to the initial position.
  ros::Duration(30).sleep();

  ROS_INFO("Start publishing waypoints.");

  trajectory_msgs::MultiDOFJointTrajectoryPtr msg(new trajectory_msgs::MultiDOFJointTrajectory);
  msg->header.stamp = ros::Time::now();
  msg->points.resize(waypoints.size());
  ros::Duration time_from_start(0);
  for (size_t i = 0; i < waypoints.size(); ++i) {
    WaypointWithTime& wp = waypoints[i];

    msg->points[i].transforms.resize(1);
    msg->points[i].velocities.resize(1);
    msg->points[i].accelerations.resize(1);

    msg->points[i].transforms[0].translation.x = wp.position.x();
    msg->points[i].transforms[0].translation.y = wp.position.y();
    msg->points[i].transforms[0].translation.z = wp.position.z();
    msg->points[i].transforms[0].rotation.w = cos(wp.yaw * 0.5);
    msg->points[i].transforms[0].rotation.x = 0;
    msg->points[i].transforms[0].rotation.y = 0;
    msg->points[i].transforms[0].rotation.z = sin(wp.yaw * 0.5);
    // We don't need velocities and accelerations here. Their constructors initialize them to zero.

    msg->points[i].time_from_start = time_from_start;
    time_from_start += ros::Duration(wp.waiting_time);
  }
  wp_pub.publish(msg);

  return 0;
}

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
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

bool sim_running = false;

static const int64_t kNanoSecondsInSecond = 1000000000;

ros::Publisher wp_pub;

class WaypointWithTime {
 public:
  WaypointWithTime()
      : waiting_time(0) {
  }

  WaypointWithTime(double t, double x, double y, double z, double roll,
                   double pitch, double yaw, double x_vel, double y_vel,
                   double z_vel, double p, double q, double r)
      : position(x, y, z),
        orientation(roll, pitch, yaw),
        velocity(x_vel, y_vel, z_vel),
        angular_velocity(p, q, r),
        waiting_time(t) {
  }

  Eigen::Vector3d position;
  Eigen::Vector3d orientation;
  Eigen::Vector3d velocity;
  Eigen::Vector3d angular_velocity;
  double waiting_time;
};

std::vector<std::string> buttonNames = { "A", "B", "X", "Y", "LB", "RB", "Back", "Start", "LStick", "RStick" };

std::map<int, std::vector<WaypointWithTime > > waypoints;
std::map<int, std::string> filenames;

void publishTrajectory(int index)
{
	trajectory_msgs::MultiDOFJointTrajectoryPtr msg(new trajectory_msgs::MultiDOFJointTrajectory);
	msg->header.stamp = ros::Time::now();
	msg->points.resize(waypoints[index].size());
	msg->joint_names.push_back("base_link");
	  int64_t time_from_start_ns = 0;
	  for (size_t i = 0; i < waypoints[index].size(); ++i) {
	    WaypointWithTime& wp = waypoints[index][i];

	    time_from_start_ns += static_cast<int64_t>(wp.waiting_time * kNanoSecondsInSecond);

	    mav_msgs::EigenTrajectoryPoint trajectory_point;
	    trajectory_point.position_W = wp.position;
	    trajectory_point.setFromRPY(wp.orientation.x(), wp.orientation.y(), wp.orientation.z());
	    trajectory_point.velocity_W = wp.velocity;
	    trajectory_point.angular_velocity_W = wp.angular_velocity;
	    trajectory_point.time_from_start_ns = time_from_start_ns;

	    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &msg->points[i]);
	  }
	wp_pub.publish(msg);
}

bool parseFile(const std::string& filename, int index)
{
	const float DEG_2_RAD = M_PI / 180.0;

	waypoints[index].clear();

    std::ifstream wp_file(filename);

    if (wp_file.is_open()) {
	  double t, x, y, z, roll, pitch, yaw, x_vel, y_vel, z_vel, p, q, r;
	  // Only read complete waypoints.
	  while (wp_file >> t >> x >> y >> z >> roll >> pitch >> yaw >> x_vel >> y_vel >> z_vel >> p >> q >> r) {
	    waypoints[index].push_back(WaypointWithTime(t, x, y, z, roll * DEG_2_RAD, pitch * DEG_2_RAD, yaw * DEG_2_RAD, x_vel, y_vel, z_vel, p, q, r));
	  }
	  wp_file.close();
	  ROS_INFO("Read %d waypoints from file %s.", (int)waypoints[index].size(), filename.c_str());
    } else {
		ROS_ERROR_STREAM("Unable to open poses file: " << filename);
	  return false;
	}

    return true;
}

void callback(const sensor_msgs::ImuPtr& /*msg*/) {
  sim_running = true;
}

void readAllFiles()
{
	const int nButtons = 10;

	for (int i=0; i<nButtons; i++)
	{
		  std::string filename;
		  if(!ros::param::get("~/file_"+buttonNames[i],filename))
		  {
			  ROS_WARN("No file assigned to button %s", buttonNames[i].c_str());
			  continue;
		  }

		  filenames[i] = filename;

		  ROS_INFO("Button %s is assigned to file %s", buttonNames[i].c_str(), filename.c_str());

		  if (!parseFile(filename, i))
		  {
			  ROS_ERROR("Could not parse file %s", filename.c_str());
		  }
	}
}

void joyCallback(const sensor_msgs::JoyPtr& msg) {
  static ros::Time lastCallback(0);

  double timePassed = (msg->header.stamp - lastCallback).toSec();

  if (msg->axes[2] < -0.5)
  {
	  readAllFiles();
	  return;
  }

  if (timePassed > 0.1)
  {
	  for (int i=0; i<msg->buttons.size(); i++)
	  {
		  if (msg->buttons[i] > 0)
		  {
			  if (waypoints.find(i) == waypoints.end())
			  {
				  ROS_WARN("Button %s pressed but no trajectory assigned.", buttonNames[i].c_str());
				  return;
			  }

			  ROS_INFO("Publishing trajectory %s", filenames[i].c_str());
			  publishTrajectory(i);
			  return;
		  }
	  }
	  lastCallback = msg->header.stamp;
  }
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "waypoint_publisher");
  ros::NodeHandle nh;

  ROS_INFO("Started waypoint_publisher.");

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  readAllFiles();

  // The IMU is used, to determine if the simulator is running or not.
  ros::Subscriber sub = nh.subscribe("imu", 10, &callback);
  ros::Subscriber joy = nh.subscribe("joy", 10, &joyCallback);

  wp_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  ROS_INFO("Wait for simulation to become ready...");

  while (!sim_running && ros::ok()) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  ROS_INFO("...ok");

  ROS_INFO("Waiting until everything is settled ...");

  // Wait for 30s such that everything can settle and the mav flies to the initial position.
  ros::Duration(2).sleep();

  ROS_INFO("Ready.");

  ros::spin();

  return 0;
}

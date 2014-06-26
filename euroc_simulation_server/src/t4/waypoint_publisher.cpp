#include <ros/ros.h>
#include <mav_msgs/ControlTrajectory.h>
#include <sensor_msgs/Imu.h>

#include <iostream>
#include <fstream>

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
    wp.position[0] = x;
    wp.position[1] = y;
    wp.position[2] = z;
    wp.yaw = yaw;
  }

  mav_msgs::ControlTrajectory wp;
  double waiting_time;
};

int main(int argc, char** argv) {

  ros::init(argc, argv, "euroc_c3_t4_waypoint_publisher");
  ros::NodeHandle nh;

  ROS_INFO("running c3_t4_waypoint_publisher");

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  if(args.size() != 2){
    ROS_ERROR("Usage: waypoint_publisher <waypoint_file> (one per line, space separated: wait_time [s] x[m] y[m] z[m] yaw[deg])");
    return -1;
  }

  std::vector<WaypointWithTime> waypoints;
  const float DEG_2_RAD = M_PI / 180.0;

  std::ifstream wp_file(args.at(1).c_str());

  if (wp_file.is_open()) {
    double t, x, y, z, yaw;
    // very safe ;), but actually only reads complete poses
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

  ros::Publisher wp_pub = nh.advertise<mav_msgs::ControlTrajectory>("waypoint", 10);
  mav_msgs::ControlTrajectory wp_msg;

  ROS_INFO("Wait for simulation to become ready...");

  while (!sim_running && ros::ok()){
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  ROS_INFO("...ok");

  // Wait for 30s such that everything can settle and the helicopter flies to initial position.
  ros::Duration(30).sleep();

  ROS_INFO("Start publishing waypoints");
  for (size_t i = 0; i < waypoints.size(); ++i) {
    const WaypointWithTime& wp = waypoints[i];
    ROS_INFO("Publishing x=%f y=%f z=%f yaw=%f, and wait for %fs", wp.wp.position[0], wp.wp.position[1],
             wp.wp.position[2], wp.wp.yaw, wp.waiting_time);
    wp_pub.publish(wp.wp);
    ros::Duration(wp.waiting_time).sleep();
  }

  return 0;
}

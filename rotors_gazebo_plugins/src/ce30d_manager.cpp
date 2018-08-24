#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

#include <sstream>

float terabee_data[8];

void chatterCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  ROS_INFO("--------------");
  for (int y = 0; y < 20; y++) {
    std::string t = "";
    for (int x = 0; x < 320; x++) {
      t += " " + std::to_string(msg->data[320 * y + x]);
    }
    ROS_INFO(t.c_str());
  }
  ROS_INFO("--------------");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ce30d_manager");

  ros::NodeHandle n;

  ros::Subscriber sub;
  sub = n.subscribe("/niv1/lidar_0", 1000, chatterCallback);

  ros::Rate loop_rate(30);

  ros::spin();

  return 0;
}
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

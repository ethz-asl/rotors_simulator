//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>
// All rights reserved.
//
// ASL 2.0
//==============================================================================
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <euroc_comm/Task1.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>  // TODO(ff): remove me, once the only the pose is read out
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>


class DatasetCreator {
 private:
  ros::NodeHandle node_handle_;
  ros::Subscriber bag_sub_;
  ros::ServiceClient client_;
  std::string imu_topic_;
  std::string cam0_topic_;
  std::string cam1_topic_;
  std::string pose_topic_;

 public:
  DatasetCreator();
  void ProcessBagFiles(std::string in_file, std::string out_file);
};

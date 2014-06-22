//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>
// All rights reserved.
//
// ASL 2.0
//==============================================================================

#include "dataset_provider.h"

DatasetCreator::DatasetCreator() : node_handle_() {
  // while(true){
  //   dataset_provider::Task1 srv;
  //   std::vector<geometry_msgs::Imu> imu_msgs;
  //   imu_msgs.append()
  //   srv.imu_msgs =
  //   client_.call()
  // }
  ros::NodeHandle pnh("~");

  pnh.param("imu_topic", imu_topic_, std::string("/fcu/imu"));
  pnh.param("cam0_topic", cam0_topic_, std::string("/cam0/image_raw"));
  pnh.param("cam1_topic", cam1_topic_, std::string("/cam1/image_raw"));
  pnh.param("pose_topic", pose_topic_, std::string("/leica/position"));  //TODO(ff): Change the leica topic to a pose topic.

  client_ = node_handle_.serviceClient<euroc_comm::Task1>("task1");
}


void DatasetCreator::ProcessBagFiles(std::string in_file, std::string out_file) {
  rosbag::Bag bag_in;
  rosbag::Bag bag_out;
  // TODO(ff): Uncomment the following lines and delete the hardcoded bag file line
  // bag_in.open(in_file, rosbag::bagmode::Read);
  bag_in.open("/home/fadri/euroc/euroc_bags/test_merge.bag", rosbag::bagmode::Read);
  // bag_out.open(out_file, rosbag::bagmode::Write);

  std::vector<std::string> topics;
  topics.push_back(imu_topic_);
  topics.push_back(cam0_topic_);
  topics.push_back(cam1_topic_);
  topics.push_back(pose_topic_);

  rosbag::View view(bag_in, rosbag::TopicQuery(topics));
  std::vector<sensor_msgs::Imu> imu_msgs;

  // std::vector<geometry_msgs::Pose::ConstPtr> pose_msgs;
  std::vector<geometry_msgs::Point::ConstPtr> position_msgs;
//  for (rosbag::MessageInstance &m : view) {
//    sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
//    if (imu_msg != NULL) {
//      imu_msgs.push_back(*imu_msg);
//    }
//
//    sensor_msgs::Image::ConstPtr image0_msg;
//    sensor_msgs::Image::ConstPtr image1_msg;
//    sensor_msgs::Image::ConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
//    if (image_msg != NULL) {
//      if (m.getTopic() == cam0_topic_) {
//        image0_msg = image_msg;
//      }
//      if (m.getTopic() == cam1_topic_) {
//        image1_msg = image_msg;
//      }
//    }
//    // TODO(ff): Uncomment the following lines and delete the ones for the position
//    // geometry_msgs::Pose::ConstPtr pose_msg = m.instantiate<geometry_msgs::Pose>();
//    // if (pose_msg != NULL)
//    //   pose_msgs.append(pose_msg);
//    geometry_msgs::Point::ConstPtr position_msg = m.instantiate<geometry_msgs::Point>();
//    if (position_msg != NULL) {
//      position_msgs.push_back(position_msg);
//    }
//
//    // check if image_msgs have been received
//    if (image0_msg != NULL && image1_msg != NULL) {
//      dataset_provider::Task1 srv;
//      srv.request.image_msgs.push_back(*image0_msg);
//      srv.request.image_msgs.push_back(*image1_msg);
//      srv.request.imu_msgs.insert(srv.request.imu_msgs.end(), imu_msgs.begin(), imu_msgs.end());
//      if (client_.call(srv)) {
//        // no need to set image messages to zero anymore, as they get destroyed after the loop
//        imu_msgs.clear();
//      }
//    }
//  }

  bag_in.close();
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "task_1");
  if (argc != 3)
  {
    ROS_INFO("usage: task_1 <bag_in_filename> <bag_out_filename>");
    return 1;
  }
  DatasetCreator dp;
  dp.ProcessBagFiles(argv[1], argv[2]);


  return 0;
}

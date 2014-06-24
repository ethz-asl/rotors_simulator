//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>
// Copyright (c) 2014, Markus Achtelik <markus.achtelik@mavt.ethz.ch>
// All rights reserved.
//
// ASL 2.0
//==============================================================================

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <euroc_comm/Task1.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <euroc_comm/VisualInertial.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Float64.h>


void process(const rosbag::Bag& bag_in, rosbag::Bag& bag_out) {

  ros::NodeHandle nh;
  // make sure this service is persistent, otherwise we're going to time ros xmlrpc calls
  ros::ServiceClient client = nh.serviceClient<euroc_comm::Task1>("task1", true);
  std::vector<std::string> topics;
  topics.push_back(std::string("vi"));
  rosbag::View view(bag_in, rosbag::TopicQuery(topics));

  for (rosbag::MessageInstance &m : view) {
    euroc_comm::VisualInertialConstPtr vi_msg = m.instantiate<euroc_comm::VisualInertial>();
    if (vi_msg != NULL) {

      euroc_comm::Task1 srv;
      srv.request.vi_data = *vi_msg;

      geometry_msgs::PoseStamped pose;
      ros::WallTime start(ros::WallTime::now());
      std_msgs::Float64 computation_time;
      if (client.call(srv)) {
        computation_time.data = (ros::WallTime::now() - start).toSec();
        pose = srv.response.pose;
      }
      else{
        ROS_ERROR("could not contact challenger server");
        computation_time.data = -1;
      }
      bag_out.write("pose", m.getTime(), pose);
      bag_out.write("duration", m.getTime(), computation_time);
    }
   }
}

bool openBag(const std::string& filename, uint32_t mode, rosbag::Bag& bag) {
  try {
    bag.open(filename, mode);
  }
  catch (rosbag::BagException& e) {
    ROS_FATAL_STREAM("Could not open output bag: "<<filename<<" because of: "<<e.what());
    return false;
  }

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "t1_dataset_provider");
  if (argc != 3)
  {
    ROS_INFO("usage: t1_dataset_provider <bag_in_filename> <bag_out_filename>");
    return 1;
  }

  std::string bag_in_name(argv[1]);
  std::string bag_out_name(argv[2]);

  rosbag::Bag bag_in, bag_out;

  openBag(bag_in_name, rosbag::bagmode::Read, bag_in);
  openBag(bag_out_name, rosbag::bagmode::Write, bag_out);

  process(bag_in, bag_out);

  bag_in.close();
  bag_out.close();

  return 0;
}

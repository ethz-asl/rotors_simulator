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
#include <euroc_comm/Task2.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <euroc_comm/VisualInertial.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Float64.h>


void process(const rosbag::Bag& bag_in, rosbag::Bag& bag_out) {

  ros::NodeHandle nh;
  // make sure this service is persistent, otherwise we're going to time ros xmlrpc calls
  ros::ServiceClient client = nh.serviceClient<euroc_comm::Task2>("task2", true);
  std::vector<std::string> topics;
  topics.push_back(std::string("vip"));
  rosbag::View view(bag_in, rosbag::TopicQuery(topics));

  // Search the last message
  ros::Time last_stamp;
  for (rosbag::MessageInstance& m : view) {
    euroc_comm::VisualInertialWithPoseConstPtr vip_msg = m.instantiate<euroc_comm::VisualInertialWithPose>();
    if (vip_msg != NULL) {
      last_stamp = vip_msg->pose.header.stamp;
    }
  }

  int count = 0;
  bool map_received = false;
  ros::WallTime task_start(ros::WallTime::now());

  for (rosbag::MessageInstance& m : view) {

    euroc_comm::VisualInertialWithPoseConstPtr vip_msg = m.instantiate<euroc_comm::VisualInertialWithPose>();

    if (vip_msg != NULL) {
      euroc_comm::Task2 srv;
      srv.request.vip_data = *vip_msg;

      if (vip_msg->pose.header.stamp == last_stamp)
        srv.request.finished = true;
      else
        srv.request.finished = false;

      std_msgs::Float64 image_computation_time;
      std_msgs::Float64 total_computation_time;

      ros::WallTime start(ros::WallTime::now());
      if (client.call(srv)) {
        ros::WallTime now(ros::WallTime::now());
        image_computation_time.data = (now - start).toSec();
        total_computation_time.data = (now - task_start).toSec();

        if (!srv.response.map.data.empty()) {
          map_received = true;
        }
      }
      else {
        ROS_ERROR("could not contact challenger server");
        image_computation_time.data = -1;
      }
      bag_out.write("duration", m.getTime(), image_computation_time);
      bag_out.write("total_duration", m.getTime(), total_computation_time);
      ++count;

      if (map_received) {
        bag_out.write("map", m.getTime(), srv.response.map);
        ROS_INFO("Received map after %d images", count);
        break;
      }
    }
  }
  ROS_WARN_COND(!map_received, "Finished playing back the dataset, but did not receive a map");
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
  ros::init(argc, argv, "t2_dataset_provider");
  if (argc != 3)
  {
    ROS_INFO("usage: t2_dataset_provider <bag_in_filename> <bag_out_filename>");
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

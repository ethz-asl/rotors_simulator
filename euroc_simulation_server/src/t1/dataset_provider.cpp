/*
 * Copyright (C) 2014 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Pascal Gohl, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Sammy Omari, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * This software is released to the Contestants of the european 
 * robotics challenges (EuRoC) for the use in stage 1. (Re)-distribution, whether 
 * in parts or entirely, is NOT PERMITTED. 
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 */

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

#include <chrono>

void process(const rosbag::Bag& bag_in, rosbag::Bag* bag_out) {
  using namespace std::chrono;

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
      std_msgs::Float64 computation_time;
      high_resolution_clock::time_point start = high_resolution_clock::now();

      if (client.call(srv)) {
        high_resolution_clock::time_point now = high_resolution_clock::now();
        computation_time.data = duration_cast<duration<double> >(now - start).count();
        pose = srv.response.pose;
      }
      else {
        ROS_ERROR("could not contact challenger server");
        computation_time.data = -1;
      }
      if (bag_out != NULL) {
        bag_out->write("pose", m.getTime(), pose);
        bag_out->write("duration", m.getTime(), computation_time);
      }
    }
  }
}

bool openBag(const std::string& filename, uint32_t mode, rosbag::Bag& bag) {
  try {
    bag.open(filename, mode);
  }
  catch (rosbag::BagException& e) {
    ROS_FATAL_STREAM("Could not open bag: "<<filename<<" because of: "<<e.what());
    return false;
  }

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "t1_dataset_provider");

  if (argc != 2 && argc != 3) {
    ROS_INFO("usage: t1_dataset_provider <bag_in_filename> [bag_out_filename]");
    return 1;
  }

  std::string bag_in_name(argv[1]);
  rosbag::Bag bag_in;
  openBag(bag_in_name, rosbag::bagmode::Read, bag_in);

  if (argc == 3) {
    std::string bag_out_name(argv[2]);

    time_t rawtime;
    struct tm *timeinfo;
    char buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, 80, "%Y-%m-%d-%H-%M-%S", timeinfo);
    std::string date_time_str(buffer);

    std::string key(".bag");
    size_t pos = bag_out_name.rfind(key);
    if (pos != std::string::npos)
      bag_out_name.erase(pos, key.length());
    bag_out_name = bag_out_name + "_" + date_time_str + ".bag";

    rosbag::Bag bag_out;
    openBag(bag_out_name, rosbag::bagmode::Write, bag_out);

    process(bag_in, &bag_out);

    bag_out.close();
  }
  else {
    process(bag_in, NULL);
  }

  bag_in.close();

  return 0;
}

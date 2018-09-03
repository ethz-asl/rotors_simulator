/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
/*
 * Desc: Contact plugin
 * Author: Nate Koenig mod by John Hsu
 */

#include "rotors_gazebo_plugins/external/gazebo_lidar_terabee_plugin.h"
#include "gazebo/physics/physics.hh"

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"

#include <stdio.h>
#include <boost/algorithm/string.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include "std_msgs/Float32MultiArray.h"

#include "rotors_gazebo_plugins/common.h"

// 3RD PARTY
#include "mav_msgs/default_topics.h"

// USER
#include <sstream>
#include "ConnectGazeboToRosTopic.pb.h"

using namespace gazebo;
using namespace std;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboLidarTerabeePlugin)

GazeboLidarTerabeePlugin::GazeboLidarTerabeePlugin()
    : pubs_and_subs_created_(false) {}

GazeboLidarTerabeePlugin::~GazeboLidarTerabeePlugin() {
#if GAZEBO_MAJOR_VERSION >= 7
  this->parentSensor->LaserShape()->DisconnectNewLaserScans(
#else
  this->parentSensor->GetLaserShape()->DisconnectNewLaserScans(
#endif
      this->newLaserScansConnection);
  this->newLaserScansConnection.reset();

  this->parentSensor.reset();
  this->world.reset();
}

void GazeboLidarTerabeePlugin::Load(sensors::SensorPtr _parent,
                                    sdf::ElementPtr _sdf) {
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }
  // Get then name of the parent sensor
  this->parentSensor =
#if GAZEBO_MAJOR_VERSION >= 7
      std::dynamic_pointer_cast<sensors::RaySensor>(_parent);
#else
      boost::dynamic_pointer_cast<sensors::RaySensor>(_parent);
#endif

  if (!this->parentSensor)
    gzthrow("RayPlugin requires a Ray Sensor as its parent");

#if GAZEBO_MAJOR_VERSION >= 7
  this->world = physics::get_world(this->parentSensor->WorldName());
#else
  this->world = physics::get_world(this->parentSensor->GetWorldName());
#endif

  this->newLaserScansConnection =
#if GAZEBO_MAJOR_VERSION >= 7
      this->parentSensor->LaserShape()->ConnectNewLaserScans(
#else
      this->parentSensor->GetLaserShape()->ConnectNewLaserScans(
#endif
          boost::bind(&GazeboLidarTerabeePlugin::OnNewLaserScans, this));

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzwarn << "[gazebo_lidar_plugin] Please specify a robotNamespace.\n";

  boost::replace_all(namespace_, "/", "");

  getSdfParam<std::string>(_sdf, "lidarTopic", lidar_topic_, "lidar_default");
  

  node_handle_ = gazebo::transport::NodePtr(new transport::Node());
  node_handle_->Init();

#if GAZEBO_MAJOR_VERSION >= 7
  const string scopedName = _parent->ParentName();
#else
  const string scopedName = _parent->GetParentName();
#endif
}

void GazeboLidarTerabeePlugin::OnNewLaserScans() {
  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  lidar_message.set_time_msec(0);
#if GAZEBO_MAJOR_VERSION >= 7
  lidar_message.set_min_distance(parentSensor->RangeMin());
  lidar_message.set_max_distance(parentSensor->RangeMax());
  lidar_message.set_current_distance(parentSensor->Range(0));
#else
  lidar_message.set_min_distance(parentSensor->GetRangeMin());
  lidar_message.set_max_distance(parentSensor->GetRangeMax());
  lidar_message.set_current_distance(parentSensor->GetRange(0));
#endif

  std::vector<double> ranges;

  std_msgs::Float32MultiArray array_msg;
  array_msg.data.clear();
  std_msgs::MultiArrayDimension dim;
  dim.label = "ranges";
  dim.size = 2;
  dim.stride = 2;
  array_msg.layout.dim.push_back(dim);

  parentSensor->Ranges(ranges);
  int resolution = ranges.size();  // number of rays
  float min_range;

  // ss << parentSensor->GetRange(i) <<" ";
  
  min_range = parentSensor->RangeMax();
  for (int i = 0; i < resolution; i++)
    if (ranges[i] < min_range) 
      min_range = ranges[i];

  std::vector<std::string> results;
  boost::split(results, lidar_topic_, [](char c){return c=='_';});
  int index = std::stoi(results[2]);
  array_msg.data.push_back((float)index);
  array_msg.data.push_back(min_range);
  // gzdbg << "sensorName: " << parentSensor->Name() << min_range << std::endl;

  lidar_pub_.publish(array_msg);
}

void GazeboLidarTerabeePlugin::CreatePubsAndSubs() {
  gzdbg << "Lidar pub , ros init = " << ros::isInitialized() << std::endl;
  ros::NodeHandle nh;
  // rosnode_ = new ros::NodeHandle("niv1");
  // lidar_pub_ = nh.advertise<lidar_msgs::msgs::lidar>("/niv1/lidar", 10);
  lidar_pub_ = nh.advertise<std_msgs::Float32MultiArray>(
      "/" + namespace_ + "/" + lidar_topic_, 10);
}

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

#include "gazebo/physics/physics.hh"
#include "rotors_gazebo_plugins/external/gazebo_lidar_plugin.h"

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdio.h>
#include <boost/algorithm/string.hpp>

#include "rotors_gazebo_plugins/common.h"

using namespace gazebo;
using namespace std;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboLidarPlugin)


GazeboLidarPlugin::GazeboLidarPlugin()
{
}


GazeboLidarPlugin::~GazeboLidarPlugin()
{
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


void GazeboLidarPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  if(kPrintOnPluginLoad) {
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
      boost::bind(&GazeboLidarPlugin::OnNewLaserScans, this));

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzwarn << "[gazebo_lidar_plugin] Please specify a robotNamespace.\n";

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

#if GAZEBO_MAJOR_VERSION >= 7
  const string scopedName = _parent->ParentName();
#else
  const string scopedName = _parent->GetParentName();
#endif
  string topicName = "~/" + scopedName + "/lidar";
  boost::replace_all(topicName, "::", "/");

  lidar_pub_ = node_handle_->Advertise<lidar_msgs::msgs::lidar>(topicName, 10);
}


void GazeboLidarPlugin::OnNewLaserScans()
{
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

  lidar_pub_->Publish(lidar_message);
}

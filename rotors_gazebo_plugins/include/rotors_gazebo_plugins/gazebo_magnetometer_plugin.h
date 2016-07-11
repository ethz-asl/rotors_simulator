/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROTORS_GAZEBO_PLUGINS_MAGNETOMETER_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_MAGNETOMETER_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/Noise.hh>
#include <gazebo/sensors/sensors.hh>
#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>

#include "rotors_gazebo_plugins/common.h"

namespace gazebo {

class GazeboMagnetometerPlugin : public SensorPlugin {
 public:
  GazeboMagnetometerPlugin();
  virtual ~GazeboMagnetometerPlugin();

 protected:
  void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

  void OnUpdate();

 private:
  std::string namespace_;
  std::string magnetometer_topic_;
  ros::NodeHandle* node_handle_;
  ros::Publisher magnetometer_pub_;
  std::string frame_id_;
  std::string link_name_;

  // Pointer to the parent sensor
  sensors::MagnetometerSensorPtr parent_sensor_;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection_;

  // Magnetic field message to be published on sensor update
  sensor_msgs::MagneticField magnetic_field_message_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_MAGNETOMETER_PLUGIN_H

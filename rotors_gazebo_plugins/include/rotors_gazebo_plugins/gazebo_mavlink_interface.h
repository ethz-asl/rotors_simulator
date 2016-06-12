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

#ifndef ROTORS_GAZEBO_PLUGINS_MAVLINK_INTERFACE_H
#define ROTORS_GAZEBO_PLUGINS_MAVLINK_INTERFACE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <mav_msgs/Actuators.h>
#include <mavros_msgs/mavlink_convert.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "rotors_gazebo_plugins/common.h"

namespace gazebo {
// Constants
static constexpr double kEarthRadius = 6378137.0;
static constexpr double kMotorSpeedOffset = 600.0;
static constexpr double kMotorSpeedScaling = 150.0;
static constexpr int kAllFieldsUpdated = 4095;

// Default values
static const std::string kDefaultMavlinkControlSubTopic = "/mavlink/from";
static const std::string kDefaultMavlinkHilSensorPubTopic = "/mavlink/to";
static const std::string kDefaultMotorSpeedsPubTopic = "gazebo/command/motor_speed";
static constexpr double kDefaultGpsUpdateFreq = 5.0;
static constexpr int kDefaultRotorCount = 4;

// Default reference magnetic field values (in Gauss), obtained from World
// Magnetic Model: (https://www.ngdc.noaa.gov/geomag/WMM/calculators.shtml) for
// Zurich: lat=+47.3667degN, lon=+8.5500degE, h=+500m, WGS84
static constexpr double kDefaultRefMagNorth = 0.21475;
static constexpr double kDefaultRefMagEast = 0.00797;
static constexpr double kDefaultRefMagDown = 0.42817;

// Default reference values (Zurich: lat=+47.3667degN, lon=+8.5500degE, h=+500m, WGS84)
static constexpr double kDefaultRefLat = 47.3667;
static constexpr double kDefaultRefLon = 8.5500;
static constexpr double kDefaultRefAlt = 500.0;

class GazeboMavlinkInterface : public ModelPlugin {
 public:
  GazeboMavlinkInterface();
  ~GazeboMavlinkInterface();

  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo&);

 private:
  std::string namespace_;
  ros::NodeHandle* node_handle_;
  ros::Subscriber mav_control_sub_;
  ros::Subscriber imu_sub_;
  ros::Publisher hil_sensor_pub_;
  ros::Publisher motor_speeds_pub_;

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  event::ConnectionPtr updateConnection_;

  void MavlinkControlCallback(const mavros_msgs::Mavlink::ConstPtr &rmsg);
  void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg);
  
  common::Time last_time_;
  common::Time last_gps_time_;
  double gps_update_interval_;
  int rotor_count_;

  mavlink_hil_sensor_t hil_sensor_msg_;
  mavlink_hil_gps_t hil_gps_msg_;

  math::Vector3 gravity_W_;
  math::Vector3 velocity_prev_W_;
  math::Vector3 mag_W_;

  double ref_lat_;
  double ref_lon_;
  double ref_alt_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_MAVLINK_INTERFACE_H

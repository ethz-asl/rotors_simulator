/*
 * Copyright (C) 2012-2017 Open Source Robotics Foundation
 * Copyright (C) 2017-2020 PX4 Development Team. All rights reserved
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
/**
 * @brief GPS Plugin
 *
 * This plugin publishes GPS data to be used and propagated
 *
 * @author Amy Wagoner <arwagoner@gmail.com>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#ifndef _GAZEBO_GPS_PLUGIN_HH_
#define _GAZEBO_GPS_PLUGIN_HH_

#include <math.h>
#include <cstdio>
#include <cstdlib>
#include <queue>
#include <random>

#include <sdf/sdf.hh>
#include "common.h"

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/GpsSensor.hh>

#include <SITLGps.pb.h>

namespace gazebo
{
static constexpr double kDefaultUpdateRate = 5.0;               // hz
static constexpr double kDefaultGpsXYRandomWalk = 2.0;          // (m/s) / sqrt(hz)
static constexpr double kDefaultGpsZRandomWalk = 4.0;           // (m/s) / sqrt(hz)
static constexpr double kDefaultGpsXYNoiseDensity = 2.0e-4;     // (m) / sqrt(hz)
static constexpr double kDefaultGpsZNoiseDensity = 4.0e-4;      // (m) / sqrt(hz)
static constexpr double kDefaultGpsVXYNoiseDensity = 0.2;       // (m/s) / sqrt(hz)
static constexpr double kDefaultGpsVZNoiseDensity = 0.4;        // (m/s) / sqrt(hz)

class GAZEBO_VISIBLE GpsPlugin : public SensorPlugin
{
public:
  GpsPlugin();
  virtual ~GpsPlugin();

protected:
  virtual void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
  virtual void OnSensorUpdate();
  virtual void OnWorldUpdate(const common::UpdateInfo& /*_info*/);

private:
  std::string namespace_;
  std::string gps_id_;
  std::default_random_engine random_generator_;
  std::normal_distribution<float> standard_normal_distribution_;

  bool gps_noise_;

  std::string model_name_;

  sensors::GpsSensorPtr parentSensor_;
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  event::ConnectionPtr updateWorldConnection_;
  event::ConnectionPtr updateSensorConnection_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr gps_pub_;

  std::string gps_topic_;
  double update_rate_;

  common::Time last_gps_time_;
  common::Time last_time_;
  common::Time current_time_;
  common::Time start_time_;

  std::mutex data_mutex_;

  // Home defaults to Zurich Irchel Park
  // @note The home position can be specified using the environment variables:
  // PX4_HOME_LAT, PX4_HOME_LON, and PX4_HOME_ALT
  double lat_home_ = kDefaultHomeLatitude;
  double lon_home_ = kDefaultHomeLongitude;
  double alt_home_ = kDefaultHomeAltitude;
  double world_latitude_ = 0.0;
  double world_longitude_ = 0.0;
  double world_altitude_ = 0.0;

  // gps delay related
  static constexpr double gps_delay_ = 0.12;           // 120 ms
  static constexpr int gps_buffer_size_max_ = 1000;
  std::queue<sensor_msgs::msgs::SITLGps> gps_delay_buffer_;

  ignition::math::Vector3d gps_bias_;
  ignition::math::Vector3d noise_gps_pos_;
  ignition::math::Vector3d noise_gps_vel_;
  ignition::math::Vector3d random_walk_gps_;
  ignition::math::Vector3d gravity_W_;
  ignition::math::Vector3d velocity_prev_W_;

  // gps noise parameters
  double std_xy_;    // meters
  double std_z_;     // meters
  std::default_random_engine rand_;
  std::normal_distribution<float> randn_;
  static constexpr const double gps_corellation_time_ = 60.0;    // s
  double gps_xy_random_walk_;
  double gps_z_random_walk_;
  double gps_xy_noise_density_;
  double gps_z_noise_density_;
  double gps_vxy_noise_density_;
  double gps_vz_noise_density_;
};     // class GAZEBO_VISIBLE GpsPlugin
}      // namespace gazebo
#endif // _GAZEBO_GPS_PLUGIN_HH_

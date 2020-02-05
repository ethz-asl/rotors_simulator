/*
 * Copyright 2016 Pavel Vechersky, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
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

#include <random>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "MagneticField.pb.h"

#include "rotors_gazebo_plugins/common.h"
#include "rotors_gazebo_plugins/sdf_api_wrapper.hpp"

namespace gazebo {

// Default magnetic field [Tesla] in NED frame, obtained from World Magnetic
// Model: (http://www.ngdc.noaa.gov/geomag-web/#igrfwmm) for Zurich:
// lat=+47.3667degN, lon=+8.5500degE, h=+500m, WGS84
static constexpr double kDefaultRefMagNorth = 0.000021493;
static constexpr double kDefaultRefMagEast = 0.000000815;
static constexpr double kDefaultRefMagDown = 0.000042795;

class GazeboMagnetometerPlugin : public ModelPlugin {

 public:
  typedef std::normal_distribution<> NormalDistribution;
  typedef std::uniform_real_distribution<> UniformDistribution;

  GazeboMagnetometerPlugin();
  virtual ~GazeboMagnetometerPlugin();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo&);

 private:

  /// \brief    Flag that is set to true once CreatePubsAndSubs() is called, used
  ///           to prevent CreatePubsAndSubs() from be called on every OnUpdate().
  bool pubs_and_subs_created_;

  /// \brief    Creates all required publishers and subscribers, incl. routing of messages to/from ROS if required.
  /// \details  Call this once the first time OnUpdate() is called (can't
  ///           be called from Load() because there is no guarantee GazeboRosInterfacePlugin has
  ///           has loaded and listening to ConnectGazeboToRosTopic and ConnectRosToGazeboTopic messages).
  void CreatePubsAndSubs();

  std::string namespace_;
  std::string magnetometer_topic_;
  gazebo::transport::NodePtr node_handle_;
  gazebo::transport::PublisherPtr magnetometer_pub_;
  std::string frame_id_;

  /// \brief    Pointer to the world.
  physics::WorldPtr world_;

  /// \brief    Pointer to the model.
  physics::ModelPtr model_;

  /// \brief    Pointer to the link.
  physics::LinkPtr link_;

  //// \brief    Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  math::Vector3 mag_W_;

  /// \brief    Magnetic field message.
  /// \details  Reused message object which is defined here to reduce
  ///           memory allocation.
  gz_sensor_msgs::MagneticField mag_message_;

  NormalDistribution noise_n_[3];

  std::random_device random_device_;
  std::mt19937 random_generator_;
};

} // namespace gazebo

#endif // ROTORS_GAZEBO_PLUGINS_MAGNETOMETER_PLUGIN_H

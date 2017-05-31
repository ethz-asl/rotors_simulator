/*
 * Copyright 2016 Pavel Vechersky, ASL, ETH Zurich, Switzerland
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

#ifndef ROTORS_GAZEBO_PLUGINS_PRESSURE_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_PRESSURE_PLUGIN_H

#include <random>

#include <glog/logging.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "FluidPressure.pb.h"

#include "rotors_gazebo_plugins/common.h"

namespace gazebo {
// Constants
static constexpr double kGasConstantNmPerKmolKelvin = 8314.32;
static constexpr double kMeanMolecularAirWeightKgPerKmol = 28.9644;
static constexpr double kGravityMagnitude = 9.80665;
static constexpr double kEarthRadiusMeters = 6356766.0;
static constexpr double kPressureOneAtmospherePascals = 101325.0;
static constexpr double kSeaLevelTempKelvin = 288.15;
static constexpr double kTempLapseKelvinPerMeter = 0.0065;
static constexpr double kAirConstantDimensionless = kGravityMagnitude *
    kMeanMolecularAirWeightKgPerKmol /
        (kGasConstantNmPerKmolKelvin * -kTempLapseKelvinPerMeter);

// Default values
static const std::string kDefaultPressurePubTopic = "air_pressure";
static constexpr double kDefaultRefAlt = 500.0; /* m, Zurich: h=+500m, WGS84) */
static constexpr double kDefaultPressureVar = 0.0; /* Pa^2, pressure variance */

class GazeboPressurePlugin : public ModelPlugin {
 public:
  /// \brief    Constructor.
  GazeboPressurePlugin();

  /// \brief    Destructor.
  virtual ~GazeboPressurePlugin();

  typedef std::normal_distribution<> NormalDistribution;

 protected:
  /// \brief    Called when the plugin is first created, and after the world
  ///           has been loaded. This function should not be blocking.
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief  	This gets called by the world update start event.
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

  /// \brief    Handle for the Gazebo node.
  gazebo::transport::NodePtr node_handle_;

  /// \brief    Pressure message publisher.
  gazebo::transport::PublisherPtr pressure_pub_;

  /// \brief    Transport namespace.
  std::string namespace_;

  /// \brief    Topic name for pressure messages.
  std::string pressure_topic_;

  /// \brief    Frame ID for pressure messages.
  std::string frame_id_;

  /// \brief    Pointer to the world.
  physics::WorldPtr world_;

  /// \brief    Pointer to the model.
  physics::ModelPtr model_;

  /// \brief    Pointer to the link.
  physics::LinkPtr link_;

  /// \brief    Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  /// \brief    Reference altitude (meters).
  double ref_alt_;

  /// \brief    Pressure measurement variance (Pa^2).
  double pressure_var_;

  /// \brief    Normal distribution for pressure noise.
  NormalDistribution pressure_n_[1];

  /// \brief    Fluid pressure message.
  /// \details  This is modified everytime OnUpdate() is called,
  //            and then published onto a topic
  gz_sensor_msgs::FluidPressure pressure_message_;

  std::mt19937 random_generator_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_PRESSURE_PLUGIN_H

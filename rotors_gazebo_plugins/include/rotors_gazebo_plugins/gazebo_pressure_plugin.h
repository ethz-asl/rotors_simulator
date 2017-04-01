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

#ifndef ROTORS_GAZEBO_PLUGINS_PRESSURE_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_PRESSURE_PLUGIN_H

#include <Eigen/Core>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>

#include "rotors_gazebo_plugins/common.h"

namespace gazebo {
// Constants
static constexpr double kRs = 8314.32; /* Nm/ (kmol K), gas constant */
static constexpr double kM0 = 28.9644; /* kg/kmol, mean molecular weight of air */
static constexpr double kG0 = 9.80665; /* m/s^2, acceleration due to gravity at 45.5425 deg lat */
static constexpr double kR0 = 6356766.0; /* m, Earth radius at g0 */
static constexpr double kP0 = 101325.0; /* Pa, air pressure at g0 */
static constexpr double kT0 = 288.15; /* K, standard sea-level temperature */
static constexpr double kTl = 0.0065; /* K/m, temperature lapse */
static constexpr double kAs = kG0 * kM0 / (kRs * -kTl);

// Default values
static const std::string kDefaultPressurePubTopic = "air_pressure";
static constexpr double kDefaultRefAlt = 500.0; /* m, Zurich: h=+500m, WGS84) */
static constexpr double kDefaultPressureVar = 0.0; /* Pa^2, pressure variance */

class GazeboPressurePlugin : public ModelPlugin {
 public:
  GazeboPressurePlugin();
  virtual ~GazeboPressurePlugin();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo&);

 private:
  std::string namespace_;
  std::string pressure_topic_;
  ros::NodeHandle* node_handle_;
  ros::Publisher pressure_pub_;
  std::string frame_id_;

  // Pointer to the world
  physics::WorldPtr world_;
  // Pointer to the model
  physics::ModelPtr model_;
  // Pointer to the link
  physics::LinkPtr link_;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection_;

  int pressure_sequence_;

  double ref_alt_;
  double pressure_var_;

  sensor_msgs::FluidPressure pressure_message_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_PRESSURE_PLUGIN_H

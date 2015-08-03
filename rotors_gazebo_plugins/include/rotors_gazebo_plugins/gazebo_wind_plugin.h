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


#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H

#include <string>
#include <ros/ros.h>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "rotors_gazebo_plugins/common.h"

namespace gazebo {
// Default values
static const std::string kDefaultFrameId = "world";
static const std::string kDefaultLinkName = "base_link";

static constexpr double kDefaultWindForceMean = 0.0;
static constexpr double kDefaultWindForceVariance = 0.0;
static constexpr double kDefaultWindGustForceMean = 0.0;
static constexpr double kDefaultWindGustForceVariance = 0.0;

static constexpr double kDefaultWindGustStart = 10.0;
static constexpr double kDefaultWindGustDuration = 0.0;

static const math::Vector3 kDefaultWindDirection = math::Vector3(1, 0, 0);
static const math::Vector3 kDefaultWindGustDirection = math::Vector3(0, 1, 0);



/// \brief This gazebo plugin simulates wind acting on a model.
class GazeboWindPlugin : public ModelPlugin {
 public:
  GazeboWindPlugin()
      : ModelPlugin(),
        namespace_(kDefaultNamespace),
        wind_pub_topic_(mav_msgs::default_topics::WIND),
        wind_force_mean_(kDefaultWindForceMean),
        wind_force_variance_(kDefaultWindForceVariance),
        wind_gust_force_mean_(kDefaultWindGustForceMean),
        wind_gust_force_variance_(kDefaultWindGustForceVariance),
        wind_direction_(kDefaultWindDirection),
        wind_gust_direction_(kDefaultWindGustDirection),
        frame_id_(kDefaultFrameId),
        link_name_(kDefaultLinkName),
        node_handle_(NULL) {}

  virtual ~GazeboWindPlugin();

 protected:
  /// \brief Load the plugin.
  /// \param[in] _model Pointer to the model that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Called when the world is updated.
  /// \param[in] _info Update timing information.
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:
    /// \brief Pointer to the update event connection.
  event::ConnectionPtr update_connection_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;

  std::string namespace_;

  std::string frame_id_;
  std::string link_name_;
  std::string wind_pub_topic_;

  double wind_force_mean_;
  double wind_force_variance_;
  double wind_gust_force_mean_;
  double wind_gust_force_variance_;

  math::Vector3 xyz_offset_;
  math::Vector3 wind_direction_;
  math::Vector3 wind_gust_direction_;

  common::Time wind_gust_end_;
  common::Time wind_gust_start_;

  ros::Publisher wind_pub_;

  ros::NodeHandle *node_handle_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H

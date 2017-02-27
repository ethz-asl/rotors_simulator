/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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


#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H

#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <mav_msgs/default_topics.h>  // This comes from the mav_comm repo

#include "rotors_gazebo_plugins/common.h"

#include "WindSpeed.pb.h"             // Wind speed message
#include "WrenchStamped.pb.h"         // Wind force message

namespace gazebo {
// Default values
static const std::string kDefaultFrameId = "world";
static const std::string kDefaultLinkName = "base_link";
static const std::string kDefaultWindSpeedPubTopic = "wind_speed";

static constexpr double kDefaultWindForceMean = 0.0;
static constexpr double kDefaultWindForceVariance = 0.0;
static constexpr double kDefaultWindGustForceMean = 0.0;
static constexpr double kDefaultWindGustForceVariance = 0.0;

static constexpr double kDefaultWindGustStart = 10.0;
static constexpr double kDefaultWindGustDuration = 0.0;

static constexpr double kDefaultWindSpeedMean = 0.0;
static constexpr double kDefaultWindSpeedVariance = 0.0;

static const math::Vector3 kDefaultWindDirection = math::Vector3(1, 0, 0);
static const math::Vector3 kDefaultWindGustDirection = math::Vector3(0, 1, 0);



/// \brief    This gazebo plugin simulates wind acting on a model.
/// \details  This plugin publishes on a Gazebo topic and instructs the ROS interface plugin to
///           forward the message onto ROS.
class GazeboWindPlugin : public ModelPlugin {
 public:
  GazeboWindPlugin()
      : ModelPlugin(),
        namespace_(kDefaultNamespace),
        wind_force_pub_topic_(mav_msgs::default_topics::EXTERNAL_FORCE),
        wind_speed_pub_topic_(mav_msgs::default_topics::WIND_SPEED),
        wind_force_mean_(kDefaultWindForceMean),
        wind_force_variance_(kDefaultWindForceVariance),
        wind_gust_force_mean_(kDefaultWindGustForceMean),
        wind_gust_force_variance_(kDefaultWindGustForceVariance),
        wind_speed_mean_(kDefaultWindSpeedMean),
        wind_speed_variance_(kDefaultWindSpeedVariance),
        wind_direction_(kDefaultWindDirection),
        wind_gust_direction_(kDefaultWindGustDirection),
        frame_id_(kDefaultFrameId),
        link_name_(kDefaultLinkName),
        node_handle_(nullptr),
        pubs_and_subs_created_(false) {}

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

  /// \brief    Flag that is set to true once CreatePubsAndSubs() is called, used
  ///           to prevent CreatePubsAndSubs() from be called on every OnUpdate().
  bool pubs_and_subs_created_;

  /// \brief    Creates all required publishers and subscribers, incl. routing of messages to/from ROS if required.
  /// \details  Call this once the first time OnUpdate() is called (can't
  ///           be called from Load() because there is no guarantee GazeboRosInterfacePlugin has
  ///           has loaded and listening to ConnectGazeboToRosTopic and ConnectRosToGazeboTopic messages).
  void CreatePubsAndSubs();

  /// \brief    Pointer to the update event connection.
  event::ConnectionPtr update_connection_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;

  std::string namespace_;

  std::string frame_id_;
  std::string link_name_;
  std::string wind_force_pub_topic_;
  std::string wind_speed_pub_topic_;

  double wind_force_mean_;
  double wind_force_variance_;
  double wind_gust_force_mean_;
  double wind_gust_force_variance_;
  double wind_speed_mean_;
  double wind_speed_variance_;

  math::Vector3 xyz_offset_;
  math::Vector3 wind_direction_;
  math::Vector3 wind_gust_direction_;

  common::Time wind_gust_end_;
  common::Time wind_gust_start_;

  gazebo::transport::PublisherPtr wind_force_pub_;
  gazebo::transport::PublisherPtr wind_speed_pub_;

  gazebo::transport::NodePtr node_handle_;

  /// \brief    Gazebo message for sending wind data.
  /// \details  This is defined at the class scope so that it is re-created
  ///           everytime a wind message needs to be sent, increasing performance.
  gz_geometry_msgs::WrenchStamped wrench_stamped_msg_;

  /// \brief    Gazebo message for sending wind speed data.
  /// \details  This is defined at the class scope so that it is re-created
  ///           everytime a wind speed message needs to be sent, increasing performance.
  gz_mav_msgs::WindSpeed wind_speed_msg_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H

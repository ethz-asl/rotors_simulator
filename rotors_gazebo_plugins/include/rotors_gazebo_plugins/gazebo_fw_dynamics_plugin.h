/*
 * Copyright 2017 Pavel Vechersky, ASL, ETH Zurich, Switzerland
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

#ifndef ROTORS_GAZEBO_PLUGINS_FW_DYNAMICS_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_FW_DYNAMICS_PLUGIN_H

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <mav_msgs/default_topics.h>

#include "WindSpeed.pb.h"

#include "rotors_gazebo_plugins/common.h"
#include "rotors_gazebo_plugins/fw_parameters.h"

namespace gazebo {

typedef const boost::shared_ptr<const gz_mav_msgs::WindSpeed> GzWindSpeedMsgPtr;

// Default interface values
static const std::string kDefaultCommandSubTopic = "gazebo/command/motor_speed";

// Constants
static constexpr double kG = 9.81;
static constexpr double kRhoAir = 1.18;

class GazeboFwDynamicsPlugin : public ModelPlugin {
 public:
  GazeboFwDynamicsPlugin();
  virtual ~GazeboFwDynamicsPlugin();

 protected:
  void UpdateForcesAndMoments();
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
  std::string wind_speed_sub_topic_;

  /// \brief    Handle for the Gazebo node.
  transport::NodePtr node_handle_;

  gazebo::transport::SubscriberPtr wind_speed_sub_;

  // Pointer to the world
  physics::WorldPtr world_;
  // Pointer to the model
  physics::ModelPtr model_;
  // Pointer to the link
  physics::LinkPtr link_;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection_;

  math::Vector3 wind_speed_W_;

  void WindSpeedCallback(GzWindSpeedMsgPtr& wind_speed_msg);
};

}  // namespace gazebo

#endif // ROTORS_GAZEBO_PLUGINS_FW_DYNAMICS_PLUGIN_H

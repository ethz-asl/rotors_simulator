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


#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_MULTIROTOR_BASE_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_MULTIROTOR_BASE_PLUGIN_H

#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <mav_msgs/MotorSpeed.h>
#include <ros/ros.h>

#include "rotors_gazebo_plugins/common.h"

namespace gazebo {
// Default values
static const std::string kDefaultNamespace = "";

static const std::string kDefaultMotorPubTopic = "motors";
static const std::string kDefaultLinkName = "base_link";
static const std::string kDefaultFrameId = "base_link";

static constexpr double kDefaultRotorVelocitySlowdownSim = 10.0;


/// \brief This plugin publishes the motor speeds of your multirotor model.
class GazeboMultirotorBasePlugin : public ModelPlugin {
  typedef std::map<const unsigned int, const physics::JointPtr> MotorNumberToJointMap;
  typedef std::pair<const unsigned int, const physics::JointPtr> MotorNumberToJointPair;
 public:
  GazeboMultirotorBasePlugin()
      : ModelPlugin(),
        namespace_(kDefaultNamespace),
        motor_pub_topic_(kDefaultMotorPubTopic),
        link_name_(kDefaultLinkName),
        frame_id_(kDefaultFrameId),
        rotor_velocity_slowdown_sim_(kDefaultRotorVelocitySlowdownSim),
        node_handle_(NULL) {}

  virtual ~GazeboMultirotorBasePlugin();

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

  physics::Link_V child_links_;

  MotorNumberToJointMap motor_joints_;

  std::string namespace_;
  std::string motor_pub_topic_;
  std::string link_name_;
  std::string frame_id_;
  double rotor_velocity_slowdown_sim_;

  ros::Publisher motor_pub_;
  ros::NodeHandle *node_handle_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_MULTIROTOR_BASE_PLUGIN_H

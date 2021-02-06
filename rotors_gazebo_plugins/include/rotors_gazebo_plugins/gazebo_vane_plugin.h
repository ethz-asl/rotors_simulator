/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_VANE_PLUGIN_HH_
#define _GAZEBO_VANE_PLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/transport/transport.hh"

#include "rotors_gazebo_plugins/common.h"
#include "ConnectGazeboToRosTopic.pb.h"
#include "vector2d.pb.h"

namespace gazebo
{
  /// \brief A template model plugin
  class GAZEBO_VISIBLE GazeboVanePlugin : public ModelPlugin
  {
  public:
      /// \brief Constructor.
      GazeboVanePlugin();

      /// \brief Destructor.
      ~GazeboVanePlugin();

      // Documentation Inherited.
      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  protected:
      /// \brief Callback for World Update events.
      void OnUpdate();

      /// \brief Connection to World Update events.
      event::ConnectionPtr update_connection_;

      /// \brief Pointer to world.
      physics::WorldPtr world_;

      /// \brief Pointer to physics engine.
      physics::PhysicsEnginePtr physics_;

      /// \brief Pointer to model containing plugin.
      physics::ModelPtr model_;

      /// \brief SDF for this plugin;
      sdf::ElementPtr sdf_;

  private:

      /// \brief    Creates all required publishers and subscribers, incl. routing of messages to/from ROS if required.
      /// \details  Call this once the first time OnUpdate() is called (can't
      ///           be called from Load() because there is no guarantee GazeboRosInterfacePlugin has
      ///           has loaded and listening to ConnectGazeboToRosTopic and ConnectRosToGazeboTopic messages).
      void CreatePubsAndSubs();

      bool pubs_and_subs_created_ = false;

      transport::NodePtr node_handle_;
      std::string namespace_;
      physics::JointPtr alpha_joint_;
      physics::JointPtr beta_joint_;
      std::string alpha_joint_name_;
      std::string beta_joint_name_;

      std::string vane_topic_;
      transport::PublisherPtr vane_pub_;
      gazebo::msgs::Vector2d vane_msg_;

  };
}
#endif

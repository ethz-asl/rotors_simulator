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
#ifndef _GAZEBO_PAYLOAD_PLUGIN_HH_
#define _GAZEBO_PAYLOAD_PLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/transport/transport.hh"

#include "rotors_gazebo_plugins/common.h"
#include "ConnectGazeboToRosTopic.pb.h"
#include "Float32.pb.h"

namespace gazebo
{

typedef const boost::shared_ptr<const gz_std_msgs::Float32> Float32Ptr;

/// \brief A template model plugin
class GAZEBO_VISIBLE GazeboPayloadPlugin : public ModelPlugin
{
    /// \brief Constructor.
public: GazeboPayloadPlugin();

    /// \brief Destructor.
public: ~GazeboPayloadPlugin();

    // Documentation Inherited.
public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
protected: void OnUpdate();

    /// \brief Connection to World Update events.
protected: event::ConnectionPtr update_connection_;

    /// \brief Pointer to world.
protected: physics::WorldPtr world_;

    /// \brief Pointer to model containing plugin.
protected:
    physics::ModelPtr model_;
    // physics::JointPtr joint_;
    // physics::LinkPtr link_;
    physics::LinkPtr parent_;
    physics::LinkPtr payload_;

    ignition::math::Vector3d hoist_pos_payload_;
    ignition::math::Vector3d hoist_pos_parent_;
    ignition::math::Quaterniond q_pr_pa_;

private:
    transport::NodePtr node_handle_;

    std::string namespace_;
    bool drop_ = false;
    bool reload_ = false;
    bool trigg_ = false;
    bool ini_ = true;
    bool reset_ = false;
    common::Timer reload_timer_;

    std::string drop_topic_;
    transport::SubscriberPtr drop_sub_;

    bool pubs_and_subs_created_ = false;
    void CreatePubsAndSubs();
    void DropCallback(Float32Ptr& drop_msg);

};
}
#endif

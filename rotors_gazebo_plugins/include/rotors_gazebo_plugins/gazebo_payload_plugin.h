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
#include <gazebo/common/common.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/transport/transport.hh"

#include "rotors_gazebo_plugins/common.h"
#include "ConnectGazeboToRosTopic.pb.h"
#include "Float32.pb.h"
#include <Lidar.pb.h>

namespace gazebo
{

typedef ignition::math::Vector3d V3D;
typedef const boost::shared_ptr<const gz_std_msgs::Float32> Float32Ptr;
typedef const boost::shared_ptr<const lidar_msgs::msgs::lidar> LidarPtr;

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

    V3D hoist_pos_payload_;
    V3D hoist_pos_parent_;
    ignition::math::Quaterniond q_pr_pa_;

private:
    transport::NodePtr node_handle_;

    std::string namespace_;
    float omega_ = 100;
    float recovery_radius_ = 1;
    bool drop_ = false;
    bool reload_ = false;
    bool trigg_ = false;
    bool ini_ = true;
    bool reset_ = false;
    bool pos_only_ = false;
    bool proximity_recovery_ = false;
    common::Timer reload_timer_;

    std::string drop_topic_;
    transport::SubscriberPtr drop_sub_;

    struct Servo {

        Servo():
        ref(0),
        slew(1.0/0.3),
        torque(0.85),
        init(false),
        P_vel(0.002),
        P_pos(0.4){}  //slew/max_err

        double ref;       // rad
        double slew;      // rad/s
        double torque;    // Nm
        common::Time last_srv_time;
        bool init;

        double P_vel;
        double P_pos;
    };

    struct HookControl {

        HookControl(){}

        physics::WorldPtr world;

        bool loaded = true;

        common::PID pid_no_load;
        common::PID pid_w_load;
        common::Time last_time;
        bool init = false;
        Servo srv;

        std::string hook_ref_topic;
        std::string lidar_topic;
        std::string hook_joint_name;
        std::string hook_elev_joint_name;

        physics::JointPtr hook_joint = nullptr;
        physics::JointPtr hook_elev_joint = nullptr;

        float lidar_dist = 0;
        float lidar_max_dist = 0;
        float lidar_min_dist = 0;
        float hook_ref = 0;

        transport::SubscriberPtr lidar_sub = nullptr;
        transport::SubscriberPtr hook_ref_sub = nullptr;

        std::mutex ref_lock;
        std::mutex lidar_lock;

        void LidarCallback(LidarPtr& lidar_msg) {
            std::unique_lock<std::mutex> lock(lidar_lock);
            lidar_dist = lidar_msg->current_distance();
            lidar_max_dist = lidar_msg->max_distance();
            lidar_min_dist = lidar_msg->min_distance();
            // gzdbg<<"lidar_range: "<<lidar_dist<<"\n";
            // gzdbg<<"lidar_min: "<<lidar_max_dist<<"\n";
            // gzdbg<<"lidar_max: "<<lidar_min_dist<<"\n";
            lock.unlock();
        }

        void RefCallback(Float32Ptr& hook_ref_msg) {
            std::unique_lock<std::mutex> lock(ref_lock);
            hook_ref = hook_ref_msg->data();
            lock.unlock();
        }

        void DoControl();
    };

    HookControl hook_ctrl_;
    bool do_hook_ctrl = false;

    bool pubs_and_subs_created_ = false;
    void CreatePubsAndSubs();
    void DropCallback(Float32Ptr& drop_msg);

};
}
#endif

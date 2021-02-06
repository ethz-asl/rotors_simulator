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
#ifndef _GAZEBO_IMU_S_PLUGIN_HH_
#define _GAZEBO_IMU_S_PLUGIN_HH_

#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/gazebo.hh>

#include "Imu.pb.h"
#include "ConnectGazeboToRosTopic.pb.h"
#include "rotors_gazebo_plugins/common.h"
//#include "rotors_gazebo_plugins/uav_parameters.h"
#include "mav_msgs/default_topics.h"

namespace gazebo
{
/// \brief A template model plugin
class GAZEBO_VISIBLE GazeboImuSPlugin : public SensorPlugin
{
    /// \brief Constructor.
public: GazeboImuSPlugin();

    /// \brief Destructor.
public: ~GazeboImuSPlugin();

    // Documentation Inherited.
protected:

    void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    void OnUpdate();

private:

    /// \brief    Flag that is set to true once CreatePubsAndSubs() is called, used
    ///           to prevent CreatePubsAndSubs() from be called on every OnUpdate().
    bool pubs_and_subs_created_;

    /// \brief    Creates all required publishers and subscribers, incl. routing of messages to/from ROS if required.
    /// \details  Call this once the first time OnUpdate() is called (can't
    ///           be called from Load() because there is no guarantee GazeboRosInterfacePlugin has
    ///           has loaded and listening to ConnectGazeboToRosTopic and ConnectRosToGazeboTopic messages).
    void CreatePubsAndSubs();

    /// \brief Pointer to world.
    physics::WorldPtr world_;

    /// \brief Pointer to physics engine.
    physics::PhysicsEnginePtr physics_;

    /// \brief Pointer to the parent sensor (e.g. imu)
    sensors::ImuSensorPtr parent_sensor_;

    /// \brief Pointer to the sensor link.
    physics::LinkPtr link_;

    /// \brief Pointer to the update event connection.
    event::ConnectionPtr updateConnection_;

    std::string namespace_;

    common::Time last_time_;
    uint64_t seq_ = 0;

    transport::NodePtr node_handle_;
    std::string imu_topic_ = "imu_gugus";
    transport::PublisherPtr imu_pub_;
    gz_sensor_msgs::Imu imu_message_;

};
}
#endif

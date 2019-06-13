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
#ifndef _GAZEBO_TEMPLATE_SENSOR_PLUGIN_HH_
#define _GAZEBO_TEMPLATE_SENSOR_PLUGIN_HH_

#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/gazebo.hh>

namespace gazebo
{
/// \brief A template sensor plugin
class GAZEBO_VISIBLE GazeboSensorTemplatePlugin : public SensorPlugin
{
    /// \brief Constructor.
public: GazeboSensorTemplatePlugin();

    /// \brief Destructor.
public: ~GazeboSensorTemplatePlugin();

    // Documentation Inherited.
protected:

    void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    void OnUpdate();

private:
    /// \brief Pointer to world.
    physics::WorldPtr world_;

    /// \brief Pointer to physics engine.
    physics::PhysicsEnginePtr physics_;

    transport::NodePtr node_handle_;

    /// \brief Pointer to the parent sensor (e.g. imu)
    sensors::ImuSensorPtr parent_sensor_;

    /// \brief Pointer to the sensor link.
    physics::LinkPtr link_;

    /// \brief Pointer to the update event connection.
    event::ConnectionPtr updateConnection_;

    std::string namespace_;
};
}
#endif

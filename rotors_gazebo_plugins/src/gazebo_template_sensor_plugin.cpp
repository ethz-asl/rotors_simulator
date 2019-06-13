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
 *
 * Modifications by David Rohr, drohr@student.ethz.ch
 *
 */

#include "rotors_gazebo_plugins/gazebo_template_sensor_plugin.h"

using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(GazeboSensorTemplatePlugin)

/////////////////////////////////////////////////
GazeboSensorTemplatePlugin::GazeboSensorTemplatePlugin()
{
    gzdbg<<"GazeboSensorTemplatePlugin constructed\n";
}

/////////////////////////////////////////////////
GazeboSensorTemplatePlugin::~GazeboSensorTemplatePlugin()
{
    gzdbg<<"GazeboSensorTemplatePlugin destructed\n";
}

/////////////////////////////////////////////////
void GazeboSensorTemplatePlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    gzdbg<<"load called"<<std::endl;

    GZ_ASSERT(_sensor, "GazeboSensorTemplatePlugin _sensor pointer is NULL");
    GZ_ASSERT(_sdf, "GazeboSensorTemplatePlugin _sdf pointer is NULL");

    // Store the pointer to the parent sensor (e.g. imu).
    parent_sensor_ = std::dynamic_pointer_cast<sensors::ImuSensor>(_sensor);
    
    world_ = physics::get_world(parent_sensor_->WorldName());
    GZ_ASSERT(world_, "GazeboSensorTemplatePlugin world pointer is NULL");

#if GAZEBO_MAJOR_VERSION >= 9
    physics_ = world_->Physics();
#else
    physics_ = world_->GetPhysicsEngine();
#endif
    GZ_ASSERT(physics_, "GazeboSensorTemplatePlugin physics pointer is NULL");
    
    namespace_.clear();

    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzerr << "Please specify a robotNamespace.\n";

    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init();

    std::string link_name;
    if (_sdf->HasElement("linkName"))
      link_name = _sdf->GetElement("linkName")->Get<std::string>();
    else
      gzerr << "Please specify a linkName.\n";

    // Get the pointer to the link that holds the sensor.
    link_ = boost::dynamic_pointer_cast<physics::Link>(world_->EntityByName(link_name));
    if (link_ == NULL)
      gzerr << "Couldn't find specified link \"" << link_name
            << "\"\n";

    // Connect to the sensor update event.
    this->updateConnection_ = this->parent_sensor_->ConnectUpdated(
        boost::bind(&GazeboSensorTemplatePlugin::OnUpdate, this));

    // Make sure the parent sensor is active.
    parent_sensor_->SetActive(true);
}

/////////////////////////////////////////////////

void GazeboSensorTemplatePlugin::OnUpdate()
{
    gzdbg<<"looping\n";
}



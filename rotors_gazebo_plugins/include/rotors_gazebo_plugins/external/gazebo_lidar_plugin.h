/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
/*
 * Desc: Ray Plugin
 * Author: Nate Koenig mod by John Hsu
 */

#ifndef _GAZEBO_RAY_PLUGIN_HH_
#define _GAZEBO_RAY_PLUGIN_HH_

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/sensors/RaySensor.hh"
#include "gazebo/util/system.hh"

#include "Lidar.pb.h"

namespace gazebo
{
  /// \brief    A Gazebo LIDAR plugin.
  class GAZEBO_VISIBLE GazeboLidarPlugin : public SensorPlugin
  {
    /// \brief Constructor
    public: GazeboLidarPlugin();

    /// \brief Destructor
    public: virtual ~GazeboLidarPlugin();

    /// \brief Update callback
    public: virtual void OnNewLaserScans();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Pointer to parent
    protected: physics::WorldPtr world;

    /// \brief The parent sensor
    private: 
      sensors::RaySensorPtr parentSensor;
      transport::NodePtr node_handle_;
      transport::PublisherPtr lidar_pub_;
      std::string namespace_;


    /// \brief The connection tied to RayPlugin::OnNewLaserScans()
    private: 
      event::ConnectionPtr newLaserScansConnection;
      lidar_msgs::msgs::lidar lidar_message;
  };
}
#endif

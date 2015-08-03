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

#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_OCTOMAP_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_OCTOMAP_PLUGIN_H

#include <iostream>
#include <math.h>

#include <rotors_gazebo_plugins/common.h>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <octomap/octomap.h>
#include <ros/ros.h>
#include <rotors_comm/Octomap.h>
#include <sdf/sdf.hh>
#include <std_srvs/Empty.h>

namespace gazebo {

class OctomapFromGazeboWorld : public WorldPlugin {
 public:
  OctomapFromGazeboWorld()
      : WorldPlugin(),
        node_handle_(kDefaultNamespace),
        octomap_(NULL) {}
  virtual ~OctomapFromGazeboWorld();

 protected:
  /// \brief Load the plugin.
  /// \param[in] _parent Pointer to the world that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
  void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
  bool CheckIfInsideObject(const std::string& name, const math::Vector3& central_point, gazebo::physics::RayShapePtr ray);
  bool CheckIfInsideObjectInX(const std::string& name, const math::Vector3& central_point, gazebo::physics::RayShapePtr ray);
  void CreateOctomap(const rotors_comm::Octomap::Request& msg);

 private:
  physics::WorldPtr world_;
  ros::NodeHandle node_handle_;
  ros::ServiceServer srv_;
  octomap::OcTree* octomap_;
  bool ServiceCallback(rotors_comm::Octomap::Request& req, rotors_comm::Octomap::Response& res);
};

}

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_OCTOMAP_PLUGIN_H

/*
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
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

#ifndef ROTORS_GAZEBO_PLUGINS_INITIAL_CONFIGURATION_H
#define ROTORS_GAZEBO_PLUGINS_INITIAL_CONFIGURATION_H

#include <stdio.h>

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include "rotors_gazebo_plugins/common.h"

namespace gazebo {

class GazeboRobotInitialConfiguration : public ModelPlugin
{
 public:
  GazeboRobotInitialConfiguration()
      : ModelPlugin()
  {
  }

  virtual ~GazeboRobotInitialConfiguration();

 protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

};
}

#endif // ROTORS_GAZEBO_PLUGINS_INITIAL_CONFIGURATION_H

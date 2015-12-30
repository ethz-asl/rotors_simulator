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

#ifndef ROTORS_GAZEBO_PLUGINS_SERVO_MOTOR_H
#define ROTORS_GAZEBO_PLUGINS_SERVO_MOTOR_H

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

class GazeboShockAbsorber : public ModelPlugin
{
 public:
  GazeboShockAbsorber()
      : ModelPlugin(),
        first_call_(true),
        sampling_time_(0.01)
  {
  }

  virtual ~GazeboShockAbsorber();

  virtual void InitializeParams();
  virtual void Publish();

 protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

 private:
  std::string parent_link_name_;
  std::string child_link_name_;
  std::string namespace_;

  double sampling_time_;
  double prev_sim_time_;
  bool first_call_;

  // spring damper parameters
  math::Vector3 translational_spring_constant_;
  math::Vector3 translational_damper_constant_;

  math::Vector3 rotational_spring_constant_;
  math::Vector3 rotational_damper_constant_;


  //double max_elongation_;
 //double minimum_elongation_;

  physics::ModelPtr model_;
  physics::LinkPtr parent_link_;
  physics::LinkPtr child_link_;
  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

};
}

#endif // ROTORS_GAZEBO_PLUGINS_SERVO_MOTOR_H

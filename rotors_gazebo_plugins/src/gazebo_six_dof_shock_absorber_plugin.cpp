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

#include "rotors_gazebo_plugins/gazebo_six_dof_shock_absorber_plugin.h"

namespace gazebo {

GazeboShockAbsorber::~GazeboShockAbsorber()
{
  updateConnection_.reset();
}

void GazeboShockAbsorber::InitializeParams()
{
}

void GazeboShockAbsorber::Publish()
{
}

void GazeboShockAbsorber::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;

  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_shock_absorber] Please specify a robotNamespace.\n";

  if (_sdf->HasElement("ParentLinkName"))
    parent_link_name_ = _sdf->GetElement("ParentLinkName")->Get<std::string>();
  else
    gzerr << "[gazebo_shock_absorber] Please specify a ParentLinkName.\n";

  // Get the pointer to the parent link.
  parent_link_ = model_->GetLink(parent_link_name_);
  if (parent_link_ == NULL)
    gzthrow("[gazebo_shock_absorber] Couldn't find specified link \"" << parent_link_name_ << "\".");

  if (_sdf->HasElement("ChildLinkName"))
    child_link_name_ = _sdf->GetElement("ChildLinkName")->Get<std::string>();
  else
    gzerr << "[gazebo_shock_absorber] Please specify a ChildLinkName.\n";

  // Get the pointer to the child link.
  child_link_ = model_->GetLink(child_link_name_);
  if (child_link_ == NULL)
    gzthrow("[gazebo_shock_absorber] Couldn't find specified link \"" << child_link_name_ << "\".");

  if (_sdf->HasElement("translational_spring_constant")) {
    translational_spring_constant_ = _sdf->GetElement("translational_spring_constant")
        ->Get<ignition::math::Vector3d>();
  } else {
    gzthrow("[gazebo_shock_absorber] Couldn't find translational_spring_constant.");
  }
  if (_sdf->HasElement("translational_damper_constant")) {
    translational_damper_constant_ = _sdf->GetElement("translational_damper_constant")
        ->Get<ignition::math::Vector3d>();
  } else {
    gzthrow("[gazebo_shock_absorber] Couldn't find translational_damper_constant.");
  }
  if (_sdf->HasElement("rotational_spring_constant")) {
    rotational_spring_constant_ =
        _sdf->GetElement("rotational_spring_constant")->Get<ignition::math::Vector3d>();
  } else {
    gzthrow("[gazebo_shock_absorber] Couldn't find rotational_spring_constant.");
  }
  if (_sdf->HasElement("rotational_damper_constant")) {
    rotational_damper_constant_ =
        _sdf->GetElement("rotational_damper_constant")->Get<ignition::math::Vector3d>();
  } else {
    gzthrow("[gazebo_shock_absorber] Couldn't find rotational_damper_constant.");
  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboShockAbsorber::OnUpdate, this, _1));
}

// This gets called by the world update start event.
void GazeboShockAbsorber::OnUpdate(const common::UpdateInfo& _info)
{

  //get correct sampling time
  if (first_call_ == true) {
    prev_sim_time_ = _info.simTime.Double();
    first_call_ = false;
    return;
  }
  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();
  sampling_time_ = limit(sampling_time_, 0.1, 0.001);

  //get links poses
  ignition::math::Pose3d parent_link_pose = parent_link_->WorldCoGPose();
  ignition::math::Pose3d child_link_pose = child_link_->WorldCoGPose();
  ignition::math::Pose3d initial_child_pos = child_link_->InitialRelativePose();

  //translational shock absorber
  ignition::math::Vector3d delta_position = parent_link_pose.Pos() - child_link_pose.Pos() + initial_child_pos.Pos();
  ignition::math::Vector3d delta_velocity = parent_link_->WorldLinearVel() - child_link_->WorldLinearVel();
  ignition::math::Vector3d force = translational_spring_constant_*delta_position + translational_damper_constant_*delta_velocity;

  child_link_->AddForce(force);
  parent_link_->AddForce(-force);

  //rotational
  ignition::math::Quaterniond delta_rotation = parent_link_pose.Rot().Inverse() * child_link_pose.Rot()
      * initial_child_pos.Rot();
  ignition::math::Vector3d delta_angular_velocity_world_frame = parent_link_->WorldAngularVel()
      - child_link_->WorldAngularVel();

  ignition::math::Vector3d torque = rotational_spring_constant_*delta_rotation.Euler();
  ignition::math::Vector3d damping_torque = rotational_damper_constant_*delta_angular_velocity_world_frame;

  child_link_->AddRelativeTorque(initial_child_pos.Rot().RotateVectorReverse(torque));
  child_link_->AddTorque(damping_torque);
  parent_link_->AddRelativeTorque(-torque);
  parent_link_->AddTorque(-damping_torque);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboShockAbsorber);
}

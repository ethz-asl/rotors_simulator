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

#include "rotors_gazebo_plugins/gazebo_robot_initial_configuration_plugin.h"

namespace gazebo {

GazeboRobotInitialConfiguration::~GazeboRobotInitialConfiguration()
{
}

void GazeboRobotInitialConfiguration::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  physics::ModelPtr model = _model;
  std::vector<physics::JointPtr> loop_closure_joints;
  std::vector<physics::LinkPtr> loop_closure_parent_links;
  std::vector<physics::LinkPtr> loop_closure_child_links;
  std::string robot_name_space = "";

  if (_sdf->HasElement("robotNamespace"))
    robot_name_space = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_robot_initial_configuration] Please specify a robotNamespace.\n";

  if (_sdf->HasElement("loop_closure_joints")) {
    sdf::ElementPtr closure_joint_ptr;
    sdf::ElementPtr loop_closure = _sdf->GetElement("loop_closure_joints");
    if (loop_closure->HasElement("joint")) {
      closure_joint_ptr = loop_closure->GetElement("joint");
    }

    while (closure_joint_ptr) {
      std::string joint_name = closure_joint_ptr->GetAttribute("joint_name")->GetAsString();
      physics::JointPtr joint = model->GetJoint(joint_name);
      physics::LinkPtr parent_link = joint->GetParent();
      physics::LinkPtr child_link = joint->GetChild();
      if (joint == NULL) {
        gzthrow(
            "[gazebo_robot_initial_configuration] Couldn't find specified joint \"" << joint_name << "\".");
      }
      loop_closure_joints.push_back(joint);
      loop_closure_parent_links.push_back(parent_link);
      loop_closure_child_links.push_back(child_link);
      joint->Detach();
      closure_joint_ptr = closure_joint_ptr->GetNextElement();
    }
  }

  if (_sdf->HasElement("initial_configuration")) {
    sdf::ElementPtr config_ptr;
    sdf::ElementPtr init_conf = _sdf->GetElement("initial_configuration");
    if (init_conf->HasElement("config")) {
      config_ptr = init_conf->GetElement("config");
    }
    while (config_ptr) {
      std::string joint_name = config_ptr->GetAttribute("joint_name")->GetAsString();
      double initial_position;
      config_ptr->GetAttribute("initial_position")->Get<double>(initial_position);
      physics::JointPtr joint = model->GetJoint(joint_name);
      physics::LinkPtr parent_link = joint->GetParent();
      physics::LinkPtr child_link = joint->GetChild();

      if (joint == NULL) {
        gzthrow(
            "[gazebo_robot_initial_configuration] Couldn't find specified joint \"" << joint_name << "\".");
      }
      joint->SetPosition(0, initial_position);
      config_ptr = config_ptr->GetNextElement();
    }
  }
  model->Update();
  for (int i = 0; i < loop_closure_joints.size(); i++) {
    loop_closure_joints.at(i)->Attach(loop_closure_parent_links.at(i),
                                      loop_closure_child_links.at(i));
  }

}

GZ_REGISTER_MODEL_PLUGIN(GazeboRobotInitialConfiguration);
}

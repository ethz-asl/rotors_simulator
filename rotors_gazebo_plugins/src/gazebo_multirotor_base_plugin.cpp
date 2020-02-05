/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
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

// MODULE HEADER INCLUDE
#include "rotors_gazebo_plugins/gazebo_multirotor_base_plugin.h"

// STANDARD LIB INCLUDES
#include <ctime>

#include "ConnectGazeboToRosTopic.pb.h"

namespace gazebo {

GazeboMultirotorBasePlugin::~GazeboMultirotorBasePlugin() {
  
}

void GazeboMultirotorBasePlugin::Load(physics::ModelPtr _model,
                                      sdf::ElementPtr _sdf) {
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  model_ = _model;
  world_ = model_->GetWorld();
  namespace_.clear();

  getSdfParam<std::string>(_sdf, "robotNamespace", namespace_, namespace_,
                           true);
  getSdfParam<std::string>(_sdf, "linkName", link_name_, link_name_, true);
  getSdfParam<std::string>(_sdf, "motorPubTopic", actuators_pub_topic_,
                           actuators_pub_topic_);
  getSdfParam<double>(_sdf, "rotorVelocitySlowdownSim",
                      rotor_velocity_slowdown_sim_,
                      rotor_velocity_slowdown_sim_);

  node_handle_ = gazebo::transport::NodePtr(new transport::Node());

  // Initialise with default namespace (typically /gazebo/default/)
  node_handle_->Init();

  frame_id_ = link_name_;

  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_multirotor_base_plugin] Couldn't find specified link \""
            << link_name_ << "\".");

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboMultirotorBasePlugin::OnUpdate, this, _1));

  child_links_ = link_->GetChildJointsLinks();
  for (unsigned int i = 0; i < child_links_.size(); i++) {
    std::string link_name = child_links_[i]->GetScopedName();

    // Check if link contains rotor_ in its name.
    int pos = link_name.find("rotor_");
    if (pos != link_name.npos) {
      std::string motor_number_str = link_name.substr(pos + 6);
      unsigned int motor_number = std::stoi(motor_number_str);
      std::string joint_name = child_links_[i]->GetName() + "_joint";
      physics::JointPtr joint = this->model_->GetJoint(joint_name);
      motor_joints_.insert(MotorNumberToJointPair(motor_number, joint));
    }
  }
}

// This gets called by the world update start event.
void GazeboMultirotorBasePlugin::OnUpdate(const common::UpdateInfo& _info) {
  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  // Get the current simulation time.
  common::Time now = world_->GetSimTime();

  actuators_msg_.mutable_header()->mutable_stamp()->set_sec(now.sec);
  actuators_msg_.mutable_header()->mutable_stamp()->set_nsec(now.nsec);
  actuators_msg_.mutable_header()->set_frame_id(frame_id_);

  joint_state_msg_.mutable_header()->mutable_stamp()->set_sec(now.sec);
  joint_state_msg_.mutable_header()->mutable_stamp()->set_nsec(now.nsec);
  joint_state_msg_.mutable_header()->set_frame_id(frame_id_);

  actuators_msg_.clear_angular_velocities();

  joint_state_msg_.clear_name();
  joint_state_msg_.clear_position();

  MotorNumberToJointMap::iterator m;
  for (m = motor_joints_.begin(); m != motor_joints_.end(); ++m) {
    double motor_rot_vel =
        m->second->GetVelocity(0) * rotor_velocity_slowdown_sim_;

    actuators_msg_.add_angular_velocities(motor_rot_vel);

    joint_state_msg_.add_name(m->second->GetName());
    joint_state_msg_.add_position(m->second->GetAngle(0).Radian());
  }

  joint_state_pub_->Publish(joint_state_msg_);
  motor_pub_->Publish(actuators_msg_);
}

void GazeboMultirotorBasePlugin::CreatePubsAndSubs() {
  // Create temporary "ConnectGazeboToRosTopic" publisher and message
  gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
          "~/" + kConnectGazeboToRosSubtopic, 1);

  gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;

  // ============================================ //
  // =========== ACTUATORS MSG SETUP ============ //
  // ============================================ //
  motor_pub_ = node_handle_->Advertise<gz_sensor_msgs::Actuators>(
      "~/" + namespace_ + "/" + actuators_pub_topic_, 10);

  // connect_gazebo_to_ros_topic_msg.set_gazebo_namespace(namespace_);
  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + model_->GetName() +
                                                   "/" + actuators_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                actuators_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::ACTUATORS);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);

  // ============================================ //
  // ========== JOINT STATE MSG SETUP =========== //
  // ============================================ //
  joint_state_pub_ = node_handle_->Advertise<gz_sensor_msgs::JointState>(
      "~/" + namespace_ + "/" + joint_state_pub_topic_, 1);

  // connect_gazebo_to_ros_topic_msg.set_gazebo_namespace(namespace_);
  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   joint_state_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                joint_state_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::JOINT_STATE);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMultirotorBasePlugin);

}  // namespace gazebo

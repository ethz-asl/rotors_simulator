/*
 * Copyright (C) 2014 Roman Bapst, CVG, ETH Zurich, Switzerland
 * 
 * This software is released to the Contestants of the european 
 * robotics challenges (EuRoC) for the use in stage 1. (Re)-distribution, whether 
 * in parts or entirely, is NOT PERMITTED. 
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 */

#include <mav_gazebo_plugins/gazebo_wing_model_plugin.h>
#include <ctime>
#include <mav_gazebo_plugins/common.h>

namespace gazebo {
GazeboWingModelPlugin::GazeboWingModelPlugin()
    : ModelPlugin(),
      node_handle_(0) {
}

GazeboWingModelPlugin::~GazeboWingModelPlugin() {
  event::Events::DisconnectWorldUpdateBegin(update_connection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboWingModelPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model_ = _model;
  world_ = model_->GetWorld();
  namespace_.clear();

  getSdfParam<std::string>(_sdf, "robotNamespace", namespace_, "", true);
  node_handle_ = new ros::NodeHandle(namespace_);
  
  getSdfParam<std::string>(_sdf, "linkName", link_name_, "", true);
  frame_id_ = link_name_;
  //link_name_ = "wing_neutral_point";
  gzmsg << link_name_;
  // Get the pointer to the link
  link_ = this->model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_wing_model_plugin] link \"" << link_name_ << "\" not found");

  getSdfParam<std::string>(_sdf, "wingPubTopic", wing_pub_topic_, "wing");

  //wing_pub_ = node_handle_->advertise<mav_msgs::MotorSpeed>(motor_pub_topic_, 10);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboWingModelPlugin::OnUpdate, this, _1));


}

// Called by the world update start event
void GazeboWingModelPlugin::OnUpdate(const common::UpdateInfo& _info) {
  common::Time now = world_->GetSimTime();
  // mav_msgs::MotorSpeedPtr msg(new mav_msgs::MotorSpeed);
  // msg->motor_speed.resize(motor_joints_.size());

  // MotorNumberToJointMap::iterator m;
  // for (m = motor_joints_.begin(); m != motor_joints_.end(); ++m) {
  //   double motor_rot_vel = m->second->GetVelocity(0) * rotor_velocity_slowdown_sim_;
  //   msg->motor_speed[m->first] = motor_rot_vel;
  // }
  // msg->header.stamp.sec = now.sec;
  // msg->header.stamp.nsec = now.nsec;
  // msg->header.frame_id = frame_id_;

  // motor_pub_.publish(msg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboWingModelPlugin);
}

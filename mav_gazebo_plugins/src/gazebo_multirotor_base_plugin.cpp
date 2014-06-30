//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>
// All rights reserved.
//
// ASL 2.0
//==============================================================================

#include <mav_gazebo_plugins/gazebo_multirotor_base_plugin.h>
#include <ctime>
#include <mav_msgs/MotorSpeed.h>
#include <mav_gazebo_plugins/common.h>

namespace gazebo {
GazeboMultirotorBasePlugin::GazeboMultirotorBasePlugin()
    : ModelPlugin(),
      node_handle_(0) {
}

GazeboMultirotorBasePlugin::~GazeboMultirotorBasePlugin() {
  event::Events::DisconnectWorldUpdateBegin(update_connection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}
;

// void GazeboMultirotorBasePlugin::InitializeParams() {};
// void GazeboMultirotorBasePlugin::Publish() {};

void GazeboMultirotorBasePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  // world_ = physics::get_world(model_->world.name);
  world_ = model_->GetWorld();
  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_multirotor_base_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  // default params
  motor_pub_topic_ = "motors";
  link_name_ = "base_link";
  frame_id_ = link_name_;

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzwarn << "[gazebo_multirotor_base_plugin] No linkName specified, using default " << link_name_ << ".\n";

  // Get the pointer to the link
  link_ = this->model_->GetLink(link_name_);
  if(link_ == NULL)
    gzthrow("[gazebo_multirotor_base_plugin] link \"" << link_name_ << "\" not found");

  if (_sdf->HasElement("motorPubTopic"))
    motor_pub_topic_ = _sdf->GetElement("motorPubTopic")->Get<std::string>();

  getSdfParam<double>(_sdf, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_, 10);

  motor_pub_ = node_handle_->advertise<mav_msgs::MotorSpeed>(motor_pub_topic_, 10);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboMultirotorBasePlugin::OnUpdate, this, _1));

  child_links_ = link_->GetChildJointsLinks();
  for (unsigned int i = 0; i < child_links_.size(); i++) {
    std::string link_name = child_links_[i]->GetScopedName();

    // Check if link contains rotor_ in its name
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

// Called by the world update start event
void GazeboMultirotorBasePlugin::OnUpdate(const common::UpdateInfo& _info) {
  common::Time now = world_->GetSimTime();
  mav_msgs::MotorSpeedPtr msg(new mav_msgs::MotorSpeed);
  msg->motor_speed.resize(motor_joints_.size());

  MotorNumberToJointMap::iterator m;
  for (m = motor_joints_.begin(); m != motor_joints_.end(); ++m) {
    double motor_rot_vel = m->second->GetVelocity(0) * rotor_velocity_slowdown_sim_;
    msg->motor_speed[m->first] = motor_rot_vel;
  }
  msg->header.stamp.sec = now.sec;
  msg->header.stamp.nsec = now.nsec;
  msg->header.frame_id = frame_id_;

  motor_pub_.publish(msg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMultirotorBasePlugin);
}

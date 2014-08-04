/*
 * Copyright (C) 2014 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Pascal Gohl, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Sammy Omari, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * This software is released to the Contestants of the european 
 * robotics challenges (EuRoC) for the use in stage 1. (Re)-distribution, whether 
 * in parts or entirely, is NOT PERMITTED. 
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 */


#include <mav_gazebo_plugins/gazebo_wind_plugin.h>
#include <mav_gazebo_plugins/common.h>
#include <geometry_msgs/WrenchStamped.h>

namespace gazebo {
GazeboWindPlugin::GazeboWindPlugin()
    : ModelPlugin(),
      node_handle_(0) {
}

GazeboWindPlugin::~GazeboWindPlugin() {
  event::Events::DisconnectWorldUpdateBegin(update_connection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}
;

void GazeboWindPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();
  frame_id_ = "world";
  link_name_ = "base_link";
  wind_direction_ = math::Vector3(1, 0, 0); // Default normalized wind direction.
  wind_gust_direction_ = math::Vector3(0, 1, 0); // Default normalized wind gust direction.
  double wind_gust_start = 10;
  double wind_gust_duration = 0;

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_wind_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("xyzOffset"))
    xyz_offset_ = _sdf->GetElement("xyzOffset")->Get<math::Vector3>();
  else
    gzerr << "[gazebo_wind_plugin] Please specify a xyzOffset.\n";

  if (_sdf->HasElement("frameId"))
    frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzwarn << "[gazebo_wind_plugin] No linkName specified, using default " << link_name_ << ".\n";
  // Get the pointer to the link
  link_ = this->model_->GetLink(link_name_);

  // Wind params
  if (_sdf->HasElement("windDirection")) {
    wind_direction_ = _sdf->GetElement("windDirection")->Get<math::Vector3>();
    wind_direction_.Normalize();
  }

  if (_sdf->HasElement("windForceMean"))
    wind_force_mean_ = _sdf->GetElement("windForceMean")->Get<double>();

  if (_sdf->HasElement("windForceVariance")) {
    wind_force_variance_ = _sdf->GetElement("windForceVariance")->Get<double>();
  }

  // Wind gust params
  if (_sdf->HasElement("windGustDirection")) {
    wind_gust_direction_ = _sdf->GetElement("windGustDirection")->Get<math::Vector3>();
    wind_gust_direction_.Normalize();
  }

  if (_sdf->HasElement("windGustStart"))
    wind_gust_start = _sdf->GetElement("windGustStart")->Get<double>();
  wind_gust_start_ = common::Time(wind_gust_start);

  if (_sdf->HasElement("windGustDuration")) {
    wind_gust_duration = _sdf->GetElement("windGustDuration")->Get<double>();
  }
  wind_gust_end_ = common::Time(wind_gust_start + wind_gust_duration);

  if (_sdf->HasElement("windGustForceMean")) {
    wind_gust_force_mean_ = _sdf->GetElement("windGustForceMean")->Get<double>();
  }

  if (_sdf->HasElement("windGustForceVariance")) {
    wind_gust_force_variance_ = _sdf->GetElement("windGustForceVariance")->Get<double>();
  }

  getSdfParam<std::string>(_sdf, "windPubTopic", wind_pub_topic_, "/" + namespace_ + "/wind");

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboWindPlugin::OnUpdate, this, _1));

  wind_pub_ = node_handle_->advertise<geometry_msgs::WrenchStamped>(wind_pub_topic_, 10);
}

// Called by the world update start event
void GazeboWindPlugin::OnUpdate(const common::UpdateInfo& _info) {
  // Get the current simulation time.
  common::Time now = world_->GetSimTime();

  // Constant wind
  double wind_strength = wind_force_mean_;
  math::Vector3 wind = wind_strength * wind_direction_;
  // Apply a force from the constant wind to the link
  this->link_->AddForceAtRelativePosition(wind, xyz_offset_);

  math::Vector3 wind_gust(0, 0, 0);
  // Wind Gusts
  if (now >= wind_gust_start_ && now < wind_gust_end_) {
    double wind_gust_strength = wind_gust_force_mean_;
    wind_gust = wind_gust_strength * wind_gust_direction_;
    // Apply a force from the wind gust to the link
    this->link_->AddForceAtRelativePosition(wind_gust, xyz_offset_);
  }

  geometry_msgs::WrenchStamped wrench_msg;

  wrench_msg.header.frame_id = frame_id_;
  wrench_msg.header.stamp.sec = now.sec;
  wrench_msg.header.stamp.nsec = now.nsec;
  wrench_msg.wrench.force.x = wind.x + wind_gust.x;
  wrench_msg.wrench.force.y = wind.y + wind_gust.y;
  wrench_msg.wrench.force.z = wind.z + wind_gust.z;
  wrench_msg.wrench.torque.x = 0;
  wrench_msg.wrench.torque.y = 0;
  wrench_msg.wrench.torque.z = 0;

  wind_pub_.publish(wrench_msg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboWindPlugin);
}

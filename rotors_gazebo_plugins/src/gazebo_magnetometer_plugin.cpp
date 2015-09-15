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


#include "rotors_gazebo_plugins/gazebo_magnetometer_plugin.h"

#include <chrono>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <rotors_gazebo_plugins/common.h>

namespace gazebo {

GazeboMagnetometerPlugin::~GazeboMagnetometerPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

// void GazebomagnetometerPlugin::InitializeParams() {};
// void GazeboMagnetometerPlugin::Publish() {};

void GazeboMagnetometerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();



  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_magnetometer_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_magnetometer_plugin] Please specify a linkName.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_magnetometer_plugin] Couldn't find specified link \"" << link_name_ << "\".");


  if (_sdf->HasElement("randomEngineSeed")) {
    random_generator_.seed(_sdf->GetElement("randomEngineSeed")->Get<unsigned int>());
  }
  else {
    random_generator_.seed(std::chrono::system_clock::now().time_since_epoch().count());
  }
  getSdfParam<std::string>(_sdf, "magnetometerTopic", magnetometer_pub_topic_, magnetometer_pub_topic_);
  getSdfParam<std::string>(_sdf, "parentFrameId", parent_frame_id_, parent_frame_id_);

  parent_link_ = world_->GetEntity(parent_frame_id_);
  if (parent_link_ == NULL && parent_frame_id_ != kDefaultParentFrameId) {
    gzthrow("[gazebo_magnetometer_plugin] Couldn't find specified parent link \"" << parent_frame_id_ << "\".");
  }


  link_name_ = namespace_ + "/" + link_name_;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboMagnetometerPlugin::OnUpdate, this, _1));

  // TODO add publisher

  //  magnetometer_pub_ = node_handle_->advertise<geometry_msgs::PoseStamped>(pose_pub_topic_, 10);
}

// This gets called by the world update start event.
void GazeboMagnetometerPlugin::OnUpdate(const common::UpdateInfo& _info) {
  // C denotes child frame, P parent frame, and W world frame.
  // Further C_pose_W_P denotes pose of P wrt. W expressed in C.
  math::Pose W_pose_W_C = link_->GetWorldCoGPose();

  math::Pose gazebo_pose = W_pose_W_C;


  // TODO(alessandro): insert code here.
  // the pose is in gazebo_pose.

  // TODO calculate magnetic field

  // TODO add noise

  // TODO add publisher


//  if (gazebo_sequence_ % measurement_divisor_ == 0) {
//    nav_msgs::Odometry odometry;
//    odometry.header.frame_id = parent_frame_id_;
//    odometry.header.seq = odometry_sequence_++;
//    odometry.header.stamp.sec = (world_->GetSimTime()).sec + ros::Duration(unknown_delay_).sec;
//    odometry.header.stamp.nsec = (world_->GetSimTime()).nsec + ros::Duration(unknown_delay_).nsec;
//    odometry.child_frame_id = link_name_;
//    copyPosition(gazebo_pose.pos, &odometry.pose.pose.position);
//    odometry.pose.pose.orientation.w = gazebo_pose.rot.w;
//    odometry.pose.pose.orientation.x = gazebo_pose.rot.x;
//    odometry.pose.pose.orientation.y = gazebo_pose.rot.y;
//    odometry.pose.pose.orientation.z = gazebo_pose.rot.z;

//  ++gazebo_sequence_;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMagnetometerPlugin);
}

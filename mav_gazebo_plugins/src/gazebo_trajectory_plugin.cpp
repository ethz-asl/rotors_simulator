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

#include <mav_gazebo_plugins/common.h>
#include <mav_gazebo_plugins/gazebo_trajectory_plugin.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <chrono>

namespace gazebo {

GazeboCameraTrajectoryPlugin::GazeboCameraTrajectoryPlugin()
    : ModelPlugin(),
      node_handle_(0),
      gazebo_seq_(0),
      pose_seq_(0)
{
}

GazeboCameraTrajectoryPlugin::~GazeboCameraTrajectoryPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}


// void GazeboCameraTrajectoryPlugin::InitializeParams() {};
// void GazeboCameraTrajectoryPlugin::Publish() {};

void GazeboCameraTrajectoryPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  // default params
  namespace_.clear();
  imu_topic_ = "imu";

  gazebo_seq_ = 0;
  pose_seq_ = 0;

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_pose_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_camera_trajectory_plugin] Please specify a linkName.\n";
  // Get the pointer to the link
  link_ = this->model_->GetLink(link_name_);

  if (_sdf->HasElement("imuTopic"))
    imu_topic_ = _sdf->GetElement("imuTopic")->Get<std::string>();


  frame_id_ = namespace_ + "/" + link_name_;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboCameraTrajectoryPlugin::OnUpdate, this, _1));

  imu_pub_ = node_handle_->advertise<sensor_msgs::Imu>(imu_topic_, 10);
}

// Called by the world update start event
void GazeboCameraTrajectoryPlugin::OnUpdate(const common::UpdateInfo& _info) {


  //link_->SetWorldPose();

  math::Pose gazebo_pose = link_->GetWorldCoGPose();

  ++gazebo_seq_;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboCameraTrajectoryPlugin);
}

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
  last_time_ = world_->GetSimTime();

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboCameraTrajectoryPlugin::OnUpdate, this, _1));

  imu_pub_ = node_handle_->advertise<sensor_msgs::Imu>(imu_topic_, 10);
}

// Called by the world update start event
void GazeboCameraTrajectoryPlugin::OnUpdate(const common::UpdateInfo& _info) {
  common::Time current_time  = world_->GetSimTime();
  double time_diff = (current_time - last_time_).Double();
  last_time_ = current_time;

  //TODO(ff): Replace this example code, by something that follows trajectories
  const double edge_time = 5;
  const double edge_distance = 10;
  const unsigned int edges = 4;
  const double lin_vel = edge_distance / edge_time;

  unsigned int direction = fmod(current_time.Double(), edge_time * edges);

  // Make a square in the z-y plane and yaw once around z on the way last edge.
  if (direction < edge_time) {
    model_->SetLinearVel(math::Vector3(0, lin_vel, 0));
    model_->SetAngularVel(math::Vector3(0, 0, 0));
  }
  else if (direction < edge_time * 2) {
    model_->SetLinearVel(math::Vector3(0, 0, lin_vel));
  }
  else if (direction < edge_time * 3) {
    model_->SetLinearVel(math::Vector3(0, -lin_vel, 0));
  }
  else if (direction < edge_time * 4) {
    model_->SetLinearVel(math::Vector3(0, 0, -lin_vel));
    model_->SetAngularVel(math::Vector3(0, 0, 2 * M_PI / edge_time));
  }

  // Setting the pose directly does not work with the hector imu, so we should
  // either set velocities as implemented here or work on our own IMU
  // implementation.
  // model_->SetLinkWorldPose(some_pose, link_);

  ++gazebo_seq_;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboCameraTrajectoryPlugin);
}

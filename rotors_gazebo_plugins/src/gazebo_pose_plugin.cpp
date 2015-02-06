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


#include "rotors_gazebo_plugins/gazebo_pose_plugin.h"

#include <chrono>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace gazebo {

GazeboPosePlugin::~GazeboPosePlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

// void GazeboPosePlugin::InitializeParams() {};
// void GazeboPosePlugin::Publish() {};

void GazeboPosePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  sdf::Vector3 noise_normal_position;
  sdf::Vector3 noise_normal_quaternion;
  sdf::Vector3 noise_uniform_position;
  sdf::Vector3 noise_uniform_quaternion;
  const sdf::Vector3 zeros3(0.0, 0.0, 0.0);

  pose_queue_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_pose_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_pose_plugin] Please specify a linkName.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_pose_plugin] Couldn't find specified link \"" << link_name_ << "\".");



  if (_sdf->HasElement("covarianceImage")) {
    std::string image_name = _sdf->GetElement("covarianceImage")->Get<std::string>();
    covariance_image_ = cv::imread(image_name, CV_LOAD_IMAGE_GRAYSCALE);
    if (covariance_image_.data == NULL)
      gzerr << "loading covariance image " << image_name << " failed" << std::endl;
    else
      gzlog << "loading covariance image " << image_name << " successful" << std::endl;
  }

  if (_sdf->HasElement("randomEngineSeed")) {
    gen_.seed(_sdf->GetElement("randomEngineSeed")->Get<unsigned int>());
  }
  else {
    gen_.seed(std::chrono::system_clock::now().time_since_epoch().count());
  }

  getSdfParam<std::string>(_sdf, "poseTopic", pose_pub_topic_, "pose");
  getSdfParam<sdf::Vector3>(_sdf, "noiseNormalPosition", noise_normal_position, zeros3);
  getSdfParam<sdf::Vector3>(_sdf, "noiseNormalQuaternion", noise_normal_quaternion, zeros3);
  getSdfParam<sdf::Vector3>(_sdf, "noiseUniformPosition", noise_uniform_position, zeros3);
  getSdfParam<sdf::Vector3>(_sdf, "noiseUniformQuaternion", noise_uniform_quaternion, zeros3);
  getSdfParam<int>(_sdf, "measurementDelay", measurement_delay_, measurement_delay_);
  getSdfParam<int>(_sdf, "measurementDivisor", measurement_divisor_, measurement_divisor_);
  getSdfParam<double>(_sdf, "unknownDelay", unknown_delay_, unknown_delay_);
  getSdfParam<double>(_sdf, "covarianceImageScale", covariance_image_scale_, covariance_image_scale_);

  pos_n_[0] = NormalDistribution(0, noise_normal_position.x);
  pos_n_[1] = NormalDistribution(0, noise_normal_position.y);
  pos_n_[2] = NormalDistribution(0, noise_normal_position.z);

  att_n_[0] = NormalDistribution(0, noise_normal_quaternion.x);
  att_n_[1] = NormalDistribution(0, noise_normal_quaternion.y);
  att_n_[2] = NormalDistribution(0, noise_normal_quaternion.z);

  pos_u_[0] = UniformDistribution(-noise_uniform_position.x, noise_uniform_position.x);
  pos_u_[1] = UniformDistribution(-noise_uniform_position.y, noise_uniform_position.y);
  pos_u_[2] = UniformDistribution(-noise_uniform_position.z, noise_uniform_position.z);

  att_u_[0] = UniformDistribution(-noise_uniform_quaternion.x, noise_uniform_quaternion.x);
  att_u_[1] = UniformDistribution(-noise_uniform_quaternion.y, noise_uniform_quaternion.y);
  att_u_[2] = UniformDistribution(-noise_uniform_quaternion.z, noise_uniform_quaternion.z);

  // Fill in covariance. We omit uniform noise here.
  Eigen::Map<Eigen::Matrix<double, 6, 6> > cov(covariance_matrix_.data());
  Eigen::Matrix<double, 6, 1> covd;

  covd << noise_normal_position.x * noise_normal_position.x, noise_normal_position.y * noise_normal_position.y, noise_normal_position.z * noise_normal_position.z, noise_normal_quaternion
      .x * noise_normal_quaternion.x, noise_normal_quaternion.y * noise_normal_quaternion.y, noise_normal_quaternion.z * noise_normal_quaternion.z;
  cov = covd.asDiagonal();

  frame_id_ = namespace_ + "/" + link_name_;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboPosePlugin::OnUpdate, this, _1));

  pose_pub_ = node_handle_->advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_pub_topic_, 10);
}

// This gets called by the world update start event.
void GazeboPosePlugin::OnUpdate(const common::UpdateInfo& _info) {
  math::Pose gazebo_pose = link_->GetWorldCoGPose();
  bool publish_pose = true;

  // First, determine whether we should publish a pose.
  if (covariance_image_.data != NULL) {
    // We have an image.

    // Image is always centered around the origin:
    int width = covariance_image_.cols;
    int height = covariance_image_.rows;
    int x = static_cast<int>(std::floor(gazebo_pose.pos.x / covariance_image_scale_)) + width / 2;
    int y = static_cast<int>(std::floor(gazebo_pose.pos.y / covariance_image_scale_)) + height / 2;

    if (x >= 0 && x < width && y >= 0 && y < height) {
      uint8_t val = covariance_image_.at<uint8_t>(y, x);
      if (val == 0)
        publish_pose = false;
      // TODO: covariance scaling, according to the intensity values could be implemented here.
    }
  }

  if (gazebo_sequence_ % measurement_divisor_ == 0) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id_;
    pose.header.seq = pose_sequence_++;
    pose.header.stamp.sec = (world_->GetSimTime()).sec + ros::Duration(unknown_delay_).sec;
    pose.header.stamp.nsec = (world_->GetSimTime()).nsec + ros::Duration(unknown_delay_).nsec;

    copyPosition(gazebo_pose.pos, &pose.pose.position);
    pose.pose.orientation.w = gazebo_pose.rot.w;
    pose.pose.orientation.x = gazebo_pose.rot.x;
    pose.pose.orientation.y = gazebo_pose.rot.y;
    pose.pose.orientation.z = gazebo_pose.rot.z;

    if (publish_pose)
      pose_queue_.push_back(std::make_pair(gazebo_sequence_ + measurement_delay_, pose));
  }

  // Is it time to publish the front element?
  if (gazebo_sequence_ == pose_queue_.front().first) {
    geometry_msgs::PoseWithCovarianceStampedPtr pose(new geometry_msgs::PoseWithCovarianceStamped);

    pose->header = pose_queue_.front().second.header;
    pose->pose.pose = pose_queue_.front().second.pose;
    pose_queue_.pop_front();

    // Calculate position distortions.
    Eigen::Vector3d pos_n;
    pos_n << pos_n_[0](gen_) + pos_u_[0](gen_), pos_n_[1](gen_) + pos_u_[1](gen_), pos_n_[2](gen_) + pos_u_[2](gen_);
    geometry_msgs::Point& p = pose->pose.pose.position;
    p.x += pos_n[0];
    p.y += pos_n[1];
    p.z += pos_n[2];

    // Calculate attitude distortions.
    Eigen::Vector3d theta;
    theta << att_n_[0](gen_) + att_u_[0](gen_), att_n_[1](gen_) + att_u_[1](gen_), att_n_[2](gen_) + att_u_[2](gen_);
    Eigen::Quaterniond q_n = QuaternionFromSmallAngle(theta);
    q_n.normalize();
    geometry_msgs::Quaternion& q = pose->pose.pose.orientation;
    Eigen::Quaterniond _q(q.w, q.x, q.y, q.z);
    _q = _q * q_n;
    q.w = _q.w();
    q.x = _q.x();
    q.y = _q.y();
    q.z = _q.z();

    pose->pose.covariance = covariance_matrix_;

    pose_pub_.publish(pose);
//    std::cout << "published pose with timestamp " << pose->header.stamp << "at time t" << world_->GetSimTime().Double()
//        << "delay should be " << measurement_delay_ << "sim cycles" << std::endl;
  }

  ++gazebo_sequence_;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboPosePlugin);
}

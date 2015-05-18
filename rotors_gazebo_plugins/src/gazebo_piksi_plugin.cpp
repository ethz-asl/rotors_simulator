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


#include "rotors_gazebo_plugins/gazebo_piksi_plugin.h"

#include <chrono>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <rotors_gazebo_plugins/common.h>

namespace gazebo {

GazeboPiksiPlugin::~GazeboPiksiPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

// void GazeboPiksiPlugin::InitializeParams() {};
// void GazeboPiksiPlugin::Publish() {};

void GazeboPiksiPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  sdf::Vector3 spp_noise_normal;
  sdf::Vector3 spp_offset;
  sdf::Vector3 rtk_fixed_noise_normal;
  sdf::Vector3 rtk_fixed_offset;
  const sdf::Vector3 zeros3(0.0, 0.0, 0.0);

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_piksi_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_piksi_plugin] Please specify a linkName.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_piksi_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  if (_sdf->HasElement("randomEngineSeed")) {
    random_generator_.seed(_sdf->GetElement("randomEngineSeed")->Get<unsigned int>());
  }
  else {
    random_generator_.seed(std::chrono::system_clock::now().time_since_epoch().count());
  }
  getSdfParam<std::string>(_sdf, "sppPositionTopic", spp_position_pub_topic_, spp_position_pub_topic_);
  getSdfParam<std::string>(_sdf, "rtkPositionTopic", rtk_position_pub_topic_, rtk_position_pub_topic_);
  getSdfParam<std::string>(_sdf, "parentFrameId", parent_frame_id_, parent_frame_id_);
  getSdfParam<std::string>(_sdf, "publishGroundTruth", publish_ground_truth_, publish_ground_truth_);
  getSdfParam<sdf::Vector3>(_sdf, "sppNoiseNormal", spp_noise_normal, zeros3);
  getSdfParam<sdf::Vector3>(_sdf, "sppOffset", spp_offset, zeros3);
  getSdfParam<sdf::Vector3>(_sdf, "rtkFixedNoiseNormal", rtk_fixed_noise_normal, zeros3);
  getSdfParam<sdf::Vector3>(_sdf, "rtkFixedOffset", rtk_fixed_offset, zeros3);


  parent_link_ = world_->GetEntity(parent_frame_id_);
  if (parent_link_ == NULL && parent_frame_id_ != kDefaultParentFrameId) {
    gzthrow("[gazebo_piksi_plugin] Couldn't find specified parent link \"" << parent_frame_id_ << "\".");
  }
  spp_position_n_[0] = NormalDistribution(0, spp_noise_normal.x);
  spp_position_n_[1] = NormalDistribution(0, spp_noise_normal.y);
  spp_position_n_[2] = NormalDistribution(0, spp_noise_normal.z);

  rtk_position_n_[0] = NormalDistribution(0, rtk_fixed_noise_normal.x);
  rtk_position_n_[1] = NormalDistribution(0, rtk_fixed_noise_normal.y);
  rtk_position_n_[2] = NormalDistribution(0, rtk_fixed_noise_normal.z);

  offset_spp_[0] = spp_offset.x;
  offset_spp_[1] = spp_offset.y;
  offset_spp_[2] = spp_offset.z;

  offset_rtk_fixed_[0] = rtk_fixed_offset.x;
  offset_rtk_fixed_[1] = rtk_fixed_offset.y;
  offset_rtk_fixed_[2] = rtk_fixed_offset.z;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboPiksiPlugin::OnUpdate, this, _1));
  spp_position_pub_ = node_handle_->advertise<geometry_msgs::Point>(spp_position_pub_topic_, 10);
  rtk_position_pub_ = node_handle_->advertise<geometry_msgs::Point>(rtk_position_pub_topic_, 10);

  if(publish_ground_truth_ == "true"){
    ground_truth_pub_ = node_handle_->advertise<geometry_msgs::Point>("position_ground_truth", 10);
  }
}

// This gets called by the world update start event.
void GazeboPiksiPlugin::OnUpdate(const common::UpdateInfo& _info) {
  // C denotes child frame, P parent frame, and W world frame.
  // Further C_pose_W_P denotes pose of P wrt. W expressed in C.
  math::Pose W_pose_W_C = link_->GetWorldCoGPose();
  math::Vector3 C_linear_velocity_W_C = link_->GetRelativeLinearVel();
  math::Vector3 C_angular_velocity_W_C = link_->GetRelativeAngularVel();

  math::Vector3 gazebo_linear_velocity = C_linear_velocity_W_C;
  math::Vector3 gazebo_angular_velocity = C_angular_velocity_W_C;
  math::Pose gazebo_pose = W_pose_W_C;

  if (parent_frame_id_ != kDefaultParentFrameId) {
    math::Pose W_pose_W_P = parent_link_->GetWorldPose();
    math::Vector3 P_linear_velocity_W_P = parent_link_->GetRelativeLinearVel();
    math::Vector3 P_angular_velocity_W_P = parent_link_->GetRelativeAngularVel();
    math::Pose C_pose_P_C_ = W_pose_W_C - W_pose_W_P;
    math::Vector3 C_linear_velocity_P_C;

    C_linear_velocity_P_C = - C_pose_P_C_.rot.GetInverse()
                            * P_angular_velocity_W_P.Cross(C_pose_P_C_.pos)
                            + C_linear_velocity_W_C
                            - C_pose_P_C_.rot.GetInverse() * P_linear_velocity_W_P;

    gazebo_angular_velocity = C_angular_velocity_W_C
                              - C_pose_P_C_.rot.GetInverse() * P_angular_velocity_W_P;
    gazebo_linear_velocity = C_linear_velocity_P_C;
    gazebo_pose = C_pose_P_C_;
  }

  /*
  nav_msgs::Odometry odometry;
  odometry.header.frame_id = parent_frame_id_;
  odometry.header.seq = odometry_sequence_++;
  odometry.header.stamp.sec = (world_->GetSimTime()).sec + ros::Duration(unknown_delay_).sec;
  odometry.header.stamp.nsec = (world_->GetSimTime()).nsec + ros::Duration(unknown_delay_).nsec;
  odometry.child_frame_id = namespace_;
  copyPosition(gazebo_pose.pos, &odometry.pose.pose.position);
  odometry.pose.pose.orientation.w = gazebo_pose.rot.w;
  odometry.pose.pose.orientation.x = gazebo_pose.rot.x;
  odometry.pose.pose.orientation.y = gazebo_pose.rot.y;
  odometry.pose.pose.orientation.z = gazebo_pose.rot.z;
  odometry.twist.twist.linear.x = gazebo_linear_velocity.x;
  odometry.twist.twist.linear.y = gazebo_linear_velocity.y;
  odometry.twist.twist.linear.z = gazebo_linear_velocity.z;
  odometry.twist.twist.angular.x = gazebo_angular_velocity.x;
  odometry.twist.twist.angular.y = gazebo_angular_velocity.y;
  odometry.twist.twist.angular.z = gazebo_angular_velocity.z;
  */

  // Is it time to publish the front element?
  /*
  nav_msgs::OdometryPtr odometry(new nav_msgs::Odometry);
  odometry->header = odometry_queue_.front().second.header;
  odometry->child_frame_id = odometry_queue_.front().second.child_frame_id;
  odometry->pose.pose = odometry_queue_.front().second.pose.pose;
  odometry->twist.twist.linear = odometry_queue_.front().second.twist.twist.linear;
  odometry->twist.twist.angular = odometry_queue_.front().second.twist.twist.angular;
  odometry_queue_.pop_front();
  */

  // Calculate position distortions.
  Eigen::Vector3d spp_pos_n;
  spp_pos_n << spp_position_n_[0](random_generator_),
               spp_position_n_[1](random_generator_),
               spp_position_n_[2](random_generator_);
  geometry_msgs::PointPtr p_spp(new geometry_msgs::Point);
  p_spp->x = gazebo_pose.pos.x + offset_spp_[0] + spp_pos_n[0];
  p_spp->y = gazebo_pose.pos.y + offset_spp_[1] + spp_pos_n[1];
  p_spp->z = gazebo_pose.pos.z + offset_spp_[2] + spp_pos_n[2];

  Eigen::Vector3d rtk_pos_n;
  rtk_pos_n << rtk_position_n_[0](random_generator_),
               rtk_position_n_[1](random_generator_),
               rtk_position_n_[2](random_generator_);
  geometry_msgs::PointPtr p_rtk(new geometry_msgs::Point);
  p_rtk->x = gazebo_pose.pos.x + offset_rtk_fixed_[0] + rtk_pos_n[0];
  p_rtk->y = gazebo_pose.pos.y + offset_rtk_fixed_[0] + rtk_pos_n[1];
  p_rtk->z = gazebo_pose.pos.z + offset_rtk_fixed_[0] + rtk_pos_n[2];

  // Publish all the topics, for which the topic name is specified.
  if (spp_position_pub_.getNumSubscribers() > 0) {
    spp_position_pub_.publish(p_spp);
  }
  if (rtk_position_pub_.getNumSubscribers() > 0) {
    rtk_position_pub_.publish(p_rtk);
  }

  // Publish Ground Truth
  if(publish_ground_truth_ == "true"){
    geometry_msgs::PointPtr p_spp_gt(new geometry_msgs::Point);
    p_spp_gt->x = gazebo_pose.pos.x;
    p_spp_gt->y = gazebo_pose.pos.y;
    p_spp_gt->z = gazebo_pose.pos.z;
    ground_truth_pub_.publish(p_spp_gt);
  }

  /*
  tf::Quaternion tf_q_W_L(q_W_L.x, q_W_L.y, q_W_L.z, q_W_L.w);
  tf::Vector3 tf_p(p.x, p.y, p.z);
  tf_ = tf::Transform(tf_q_W_L, tf_p);
  transform_broadcaster_.sendTransform(tf::StampedTransform(tf_, odometry->header.stamp, parent_frame_id_, namespace_));
  //    std::cout << "published odometry with timestamp " << odometry->header.stamp << "at time t" << world_->GetSimTime().Double()
  //        << "delay should be " << measurement_delay_ << "sim cycles" << std::endl;
   *
   */
}

GZ_REGISTER_MODEL_PLUGIN(GazeboPiksiPlugin);
}

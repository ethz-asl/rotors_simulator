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

// MODULE
#include "rotors_gazebo_plugins/gazebo_odometry_plugin.h"

// SYSTEM
#include <chrono>
#include <iostream>

// 3RD PARTY
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// USER
#include <rotors_gazebo_plugins/common.h>
#include "ConnectGazeboToRosTopic.pb.h"
#include "ConnectRosToGazeboTopic.pb.h"
#include "PoseStamped.pb.h"
#include "PoseWithCovarianceStamped.pb.h"
#include "TransformStamped.pb.h"
#include "TransformStampedWithFrameIds.pb.h"
#include "Vector3dStamped.pb.h"

namespace gazebo {

GazeboOdometryPlugin::~GazeboOdometryPlugin() {
}

void GazeboOdometryPlugin::Load(physics::ModelPtr _model,
                                sdf::ElementPtr _sdf) {
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  SdfVector3 noise_normal_position;
  SdfVector3 noise_normal_quaternion;
  SdfVector3 noise_normal_linear_velocity;
  SdfVector3 noise_normal_angular_velocity;
  SdfVector3 noise_uniform_position;
  SdfVector3 noise_uniform_quaternion;
  SdfVector3 noise_uniform_linear_velocity;
  SdfVector3 noise_uniform_angular_velocity;
  const SdfVector3 zeros3(0.0, 0.0, 0.0);

  odometry_queue_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_odometry_plugin] Please specify a robotNamespace.\n";

  node_handle_ = gazebo::transport::NodePtr(new transport::Node());

  // Initialise with default namespace (typically /gazebo/default/)
  node_handle_->Init();

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_odometry_plugin] Please specify a linkName.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_odometry_plugin] Couldn't find specified link \""
            << link_name_ << "\".");

  if (_sdf->HasElement("covarianceImage")) {
    std::string image_name =
        _sdf->GetElement("covarianceImage")->Get<std::string>();
    covariance_image_ = cv::imread(image_name, CV_LOAD_IMAGE_GRAYSCALE);
    if (covariance_image_.data == NULL)
      gzerr << "loading covariance image " << image_name << " failed"
            << std::endl;
    else
      gzlog << "loading covariance image " << image_name << " successful"
            << std::endl;
  }

  if (_sdf->HasElement("randomEngineSeed")) {
    random_generator_.seed(
        _sdf->GetElement("randomEngineSeed")->Get<unsigned int>());
  } else {
    random_generator_.seed(
        std::chrono::system_clock::now().time_since_epoch().count());
  }
  getSdfParam<std::string>(_sdf, "poseTopic", pose_pub_topic_, pose_pub_topic_);
  getSdfParam<std::string>(_sdf, "poseWithCovarianceTopic",
                           pose_with_covariance_stamped_pub_topic_,
                           pose_with_covariance_stamped_pub_topic_);
  getSdfParam<std::string>(_sdf, "positionTopic", position_stamped_pub_topic_,
                           position_stamped_pub_topic_);
  getSdfParam<std::string>(_sdf, "transformTopic", transform_stamped_pub_topic_,
                           transform_stamped_pub_topic_);
  getSdfParam<std::string>(_sdf, "odometryTopic", odometry_pub_topic_,
                           odometry_pub_topic_);
  getSdfParam<std::string>(_sdf, "parentFrameId", parent_frame_id_,
                           parent_frame_id_);
  getSdfParam<std::string>(_sdf, "childFrameId", child_frame_id_,
                           child_frame_id_);
  getSdfParam<SdfVector3>(_sdf, "noiseNormalPosition", noise_normal_position,
                          zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseNormalQuaternion",
                          noise_normal_quaternion, zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseNormalLinearVelocity",
                          noise_normal_linear_velocity, zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseNormalAngularVelocity",
                          noise_normal_angular_velocity, zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseUniformPosition", noise_uniform_position,
                          zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseUniformQuaternion",
                          noise_uniform_quaternion, zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseUniformLinearVelocity",
                          noise_uniform_linear_velocity, zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseUniformAngularVelocity",
                          noise_uniform_angular_velocity, zeros3);
  getSdfParam<int>(_sdf, "measurementDelay", measurement_delay_,
                   measurement_delay_);
  getSdfParam<int>(_sdf, "measurementDivisor", measurement_divisor_,
                   measurement_divisor_);
  getSdfParam<double>(_sdf, "unknownDelay", unknown_delay_, unknown_delay_);
  getSdfParam<double>(_sdf, "covarianceImageScale", covariance_image_scale_,
                      covariance_image_scale_);

  parent_link_ = world_->EntityByName(parent_frame_id_);
  if (parent_link_ == NULL && parent_frame_id_ != kDefaultParentFrameId) {
    gzthrow("[gazebo_odometry_plugin] Couldn't find specified parent link \""
            << parent_frame_id_ << "\".");
  }

  position_n_[0] = NormalDistribution(0, noise_normal_position.X());
  position_n_[1] = NormalDistribution(0, noise_normal_position.Y());
  position_n_[2] = NormalDistribution(0, noise_normal_position.Z());

  attitude_n_[0] = NormalDistribution(0, noise_normal_quaternion.X());
  attitude_n_[1] = NormalDistribution(0, noise_normal_quaternion.Y());
  attitude_n_[2] = NormalDistribution(0, noise_normal_quaternion.Z());

  linear_velocity_n_[0] =
      NormalDistribution(0, noise_normal_linear_velocity.X());
  linear_velocity_n_[1] =
      NormalDistribution(0, noise_normal_linear_velocity.Y());
  linear_velocity_n_[2] =
      NormalDistribution(0, noise_normal_linear_velocity.Z());

  angular_velocity_n_[0] =
      NormalDistribution(0, noise_normal_angular_velocity.X());
  angular_velocity_n_[1] =
      NormalDistribution(0, noise_normal_angular_velocity.Y());
  angular_velocity_n_[2] =
      NormalDistribution(0, noise_normal_angular_velocity.Z());

  position_u_[0] = UniformDistribution(-noise_uniform_position.X(),
                                       noise_uniform_position.X());
  position_u_[1] = UniformDistribution(-noise_uniform_position.Y(),
                                       noise_uniform_position.Y());
  position_u_[2] = UniformDistribution(-noise_uniform_position.Z(),
                                       noise_uniform_position.Z());

  attitude_u_[0] = UniformDistribution(-noise_uniform_quaternion.X(),
                                       noise_uniform_quaternion.X());
  attitude_u_[1] = UniformDistribution(-noise_uniform_quaternion.Y(),
                                       noise_uniform_quaternion.Y());
  attitude_u_[2] = UniformDistribution(-noise_uniform_quaternion.Z(),
                                       noise_uniform_quaternion.Z());

  linear_velocity_u_[0] = UniformDistribution(
      -noise_uniform_linear_velocity.X(), noise_uniform_linear_velocity.X());
  linear_velocity_u_[1] = UniformDistribution(
      -noise_uniform_linear_velocity.Y(), noise_uniform_linear_velocity.Y());
  linear_velocity_u_[2] = UniformDistribution(
      -noise_uniform_linear_velocity.Z(), noise_uniform_linear_velocity.Z());

  angular_velocity_u_[0] = UniformDistribution(
      -noise_uniform_angular_velocity.X(), noise_uniform_angular_velocity.X());
  angular_velocity_u_[1] = UniformDistribution(
      -noise_uniform_angular_velocity.Y(), noise_uniform_angular_velocity.Y());
  angular_velocity_u_[2] = UniformDistribution(
      -noise_uniform_angular_velocity.Z(), noise_uniform_angular_velocity.Z());

  // Fill in covariance. We omit uniform noise here.
  Eigen::Map<Eigen::Matrix<double, 6, 6> > pose_covariance(
      pose_covariance_matrix_.data());
  Eigen::Matrix<double, 6, 1> pose_covd;

  pose_covd << noise_normal_position.X() * noise_normal_position.X(),
      noise_normal_position.Y() * noise_normal_position.Y(),
      noise_normal_position.Z() * noise_normal_position.Z(),
      noise_normal_quaternion.X() * noise_normal_quaternion.X(),
      noise_normal_quaternion.Y() * noise_normal_quaternion.Y(),
      noise_normal_quaternion.Z() * noise_normal_quaternion.Z();
  pose_covariance = pose_covd.asDiagonal();

  // Fill in covariance. We omit uniform noise here.
  Eigen::Map<Eigen::Matrix<double, 6, 6> > twist_covariance(
      twist_covariance_matrix_.data());
  Eigen::Matrix<double, 6, 1> twist_covd;

  twist_covd << noise_normal_linear_velocity.X() *
                    noise_normal_linear_velocity.X(),
      noise_normal_linear_velocity.Y() * noise_normal_linear_velocity.Y(),
      noise_normal_linear_velocity.Z() * noise_normal_linear_velocity.Z(),
      noise_normal_angular_velocity.X() * noise_normal_angular_velocity.X(),
      noise_normal_angular_velocity.Y() * noise_normal_angular_velocity.Y(),
      noise_normal_angular_velocity.Z() * noise_normal_angular_velocity.Z();
  twist_covariance = twist_covd.asDiagonal();

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboOdometryPlugin::OnUpdate, this, _1));
}

// This gets called by the world update start event.
void GazeboOdometryPlugin::OnUpdate(const common::UpdateInfo& _info) {
  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  // C denotes child frame, P parent frame, and W world frame.
  // Further C_pose_W_P denotes pose of P wrt. W expressed in C.
  ignition::math::Pose3d W_pose_W_C = link_->WorldCoGPose();
  ignition::math::Vector3d C_linear_velocity_W_C = link_->RelativeLinearVel();
  ignition::math::Vector3d C_angular_velocity_W_C = link_->RelativeAngularVel();

  ignition::math::Vector3d gazebo_linear_velocity = C_linear_velocity_W_C;
  ignition::math::Vector3d gazebo_angular_velocity = C_angular_velocity_W_C;
  ignition::math::Pose3d gazebo_pose = W_pose_W_C;

  if (parent_frame_id_ != kDefaultParentFrameId) {
    ignition::math::Pose3d W_pose_W_P = parent_link_->WorldPose();
    ignition::math::Vector3d P_linear_velocity_W_P = parent_link_->RelativeLinearVel();
    ignition::math::Vector3d P_angular_velocity_W_P =
        parent_link_->RelativeAngularVel();
    ignition::math::Pose3d C_pose_P_C_ = W_pose_W_C - W_pose_W_P;
    ignition::math::Vector3d C_linear_velocity_P_C;
    // \prescript{}{C}{\dot{r}}_{PC} = -R_{CP}
    //       \cdot \prescript{}{P}{\omega}_{WP} \cross \prescript{}{P}{r}_{PC}
    //       + \prescript{}{C}{v}_{WC}
    //                                 - R_{CP} \cdot \prescript{}{P}{v}_{WP}
    C_linear_velocity_P_C =
        -C_pose_P_C_.Rot().Inverse() *
            P_angular_velocity_W_P.Cross(C_pose_P_C_.Pos()) +
        C_linear_velocity_W_C -
        C_pose_P_C_.Rot().Inverse() * P_linear_velocity_W_P;

    // \prescript{}{C}{\omega}_{PC} = \prescript{}{C}{\omega}_{WC}
    //       - R_{CP} \cdot \prescript{}{P}{\omega}_{WP}
    gazebo_angular_velocity =
        C_angular_velocity_W_C -
        C_pose_P_C_.Rot().Inverse() * P_angular_velocity_W_P;
    gazebo_linear_velocity = C_linear_velocity_P_C;
    gazebo_pose = C_pose_P_C_;
  }

  // This flag could be set to false in the following code...
  bool publish_odometry = true;

  // First, determine whether we should publish a odometry.
  if (covariance_image_.data != NULL) {
    // We have an image.

    // Image is always centered around the origin:
    int width = covariance_image_.cols;
    int height = covariance_image_.rows;
    int x = static_cast<int>(
                std::floor(gazebo_pose.Pos().X() / covariance_image_scale_)) +
            width / 2;
    int y = static_cast<int>(
                std::floor(gazebo_pose.Pos().Y() / covariance_image_scale_)) +
            height / 2;

    if (x >= 0 && x < width && y >= 0 && y < height) {
      uint8_t pixel_value = covariance_image_.at<uint8_t>(y, x);
      if (pixel_value == 0) {
        publish_odometry = false;
        // TODO: covariance scaling, according to the intensity values could be
        // implemented here.
      }
    }
  }

  if (gazebo_sequence_ % measurement_divisor_ == 0) {
    gz_geometry_msgs::Odometry odometry;
    odometry.mutable_header()->set_frame_id(parent_frame_id_);
    odometry.mutable_header()->mutable_stamp()->set_sec(
        (world_->SimTime()).sec + static_cast<int32_t>(unknown_delay_));
    odometry.mutable_header()->mutable_stamp()->set_nsec(
        (world_->SimTime()).nsec + static_cast<int32_t>(unknown_delay_));
    odometry.set_child_frame_id(child_frame_id_);

    odometry.mutable_pose()->mutable_pose()->mutable_position()->set_x(
        gazebo_pose.Pos().X());
    odometry.mutable_pose()->mutable_pose()->mutable_position()->set_y(
        gazebo_pose.Pos().Y());
    odometry.mutable_pose()->mutable_pose()->mutable_position()->set_z(
        gazebo_pose.Pos().Z());

    odometry.mutable_pose()->mutable_pose()->mutable_orientation()->set_x(
        gazebo_pose.Rot().X());
    odometry.mutable_pose()->mutable_pose()->mutable_orientation()->set_y(
        gazebo_pose.Rot().Y());
    odometry.mutable_pose()->mutable_pose()->mutable_orientation()->set_z(
        gazebo_pose.Rot().Z());
    odometry.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(
        gazebo_pose.Rot().W());

    odometry.mutable_twist()->mutable_twist()->mutable_linear()->set_x(
        gazebo_linear_velocity.X());
    odometry.mutable_twist()->mutable_twist()->mutable_linear()->set_y(
        gazebo_linear_velocity.Y());
    odometry.mutable_twist()->mutable_twist()->mutable_linear()->set_z(
        gazebo_linear_velocity.Z());

    odometry.mutable_twist()->mutable_twist()->mutable_angular()->set_x(
        gazebo_angular_velocity.X());
    odometry.mutable_twist()->mutable_twist()->mutable_angular()->set_y(
        gazebo_angular_velocity.Y());
    odometry.mutable_twist()->mutable_twist()->mutable_angular()->set_z(
        gazebo_angular_velocity.Z());

    if (publish_odometry)
      odometry_queue_.push_back(
          std::make_pair(gazebo_sequence_ + measurement_delay_, odometry));
  }

  // Is it time to publish the front element?
  if (gazebo_sequence_ == odometry_queue_.front().first) {
    // Copy the odometry message that is on the queue
    gz_geometry_msgs::Odometry odometry_msg(odometry_queue_.front().second);

    // Now that we have copied the first element from the queue, remove it.
    odometry_queue_.pop_front();

    // Calculate position distortions.
    Eigen::Vector3d pos_n;
    pos_n << position_n_[0](random_generator_) +
                 position_u_[0](random_generator_),
        position_n_[1](random_generator_) + position_u_[1](random_generator_),
        position_n_[2](random_generator_) + position_u_[2](random_generator_);

    gazebo::msgs::Vector3d* p =
        odometry_msg.mutable_pose()->mutable_pose()->mutable_position();
    p->set_x(p->x() + pos_n[0]);
    p->set_y(p->y() + pos_n[1]);
    p->set_z(p->z() + pos_n[2]);

    // Calculate attitude distortions.
    Eigen::Vector3d theta;
    theta << attitude_n_[0](random_generator_) +
                 attitude_u_[0](random_generator_),
        attitude_n_[1](random_generator_) + attitude_u_[1](random_generator_),
        attitude_n_[2](random_generator_) + attitude_u_[2](random_generator_);
    Eigen::Quaterniond q_n = QuaternionFromSmallAngle(theta);
    q_n.normalize();

    gazebo::msgs::Quaternion* q_W_L =
        odometry_msg.mutable_pose()->mutable_pose()->mutable_orientation();

    Eigen::Quaterniond _q_W_L(q_W_L->w(), q_W_L->x(), q_W_L->y(), q_W_L->z());
    _q_W_L = _q_W_L * q_n;
    q_W_L->set_w(_q_W_L.w());
    q_W_L->set_x(_q_W_L.x());
    q_W_L->set_y(_q_W_L.y());
    q_W_L->set_z(_q_W_L.z());

    // Calculate linear velocity distortions.
    Eigen::Vector3d linear_velocity_n;
    linear_velocity_n << linear_velocity_n_[0](random_generator_) +
                             linear_velocity_u_[0](random_generator_),
        linear_velocity_n_[1](random_generator_) +
            linear_velocity_u_[1](random_generator_),
        linear_velocity_n_[2](random_generator_) +
            linear_velocity_u_[2](random_generator_);

    gazebo::msgs::Vector3d* linear_velocity =
        odometry_msg.mutable_twist()->mutable_twist()->mutable_linear();

    linear_velocity->set_x(linear_velocity->x() + linear_velocity_n[0]);
    linear_velocity->set_y(linear_velocity->y() + linear_velocity_n[1]);
    linear_velocity->set_z(linear_velocity->z() + linear_velocity_n[2]);

    // Calculate angular velocity distortions.
    Eigen::Vector3d angular_velocity_n;
    angular_velocity_n << angular_velocity_n_[0](random_generator_) +
                              angular_velocity_u_[0](random_generator_),
        angular_velocity_n_[1](random_generator_) +
            angular_velocity_u_[1](random_generator_),
        angular_velocity_n_[2](random_generator_) +
            angular_velocity_u_[2](random_generator_);

    gazebo::msgs::Vector3d* angular_velocity =
        odometry_msg.mutable_twist()->mutable_twist()->mutable_angular();

    angular_velocity->set_x(angular_velocity->x() + angular_velocity_n[0]);
    angular_velocity->set_y(angular_velocity->y() + angular_velocity_n[1]);
    angular_velocity->set_z(angular_velocity->z() + angular_velocity_n[2]);

    odometry_msg.mutable_pose()->mutable_covariance()->Clear();
    for (int i = 0; i < pose_covariance_matrix_.size(); i++) {
      odometry_msg.mutable_pose()->mutable_covariance()->Add(
          pose_covariance_matrix_[i]);
    }

    odometry_msg.mutable_twist()->mutable_covariance()->Clear();
    for (int i = 0; i < twist_covariance_matrix_.size(); i++) {
      odometry_msg.mutable_twist()->mutable_covariance()->Add(
          twist_covariance_matrix_[i]);
    }

    // Publish all the topics, for which the topic name is specified.
    if (pose_pub_->HasConnections()) {
      pose_pub_->Publish(odometry_msg.pose().pose());
    }

    if (pose_with_covariance_stamped_pub_->HasConnections()) {
      gz_geometry_msgs::PoseWithCovarianceStamped
          pose_with_covariance_stamped_msg;

      pose_with_covariance_stamped_msg.mutable_header()->CopyFrom(
          odometry_msg.header());
      pose_with_covariance_stamped_msg.mutable_pose_with_covariance()->CopyFrom(
          odometry_msg.pose());

      pose_with_covariance_stamped_pub_->Publish(
          pose_with_covariance_stamped_msg);
    }

    if (position_stamped_pub_->HasConnections()) {
      gz_geometry_msgs::Vector3dStamped position_stamped_msg;
      position_stamped_msg.mutable_header()->CopyFrom(odometry_msg.header());
      position_stamped_msg.mutable_position()->CopyFrom(
          odometry_msg.pose().pose().position());

      position_stamped_pub_->Publish(position_stamped_msg);
    }

    if (transform_stamped_pub_->HasConnections()) {
      gz_geometry_msgs::TransformStamped transform_stamped_msg;

      transform_stamped_msg.mutable_header()->CopyFrom(odometry_msg.header());
      transform_stamped_msg.mutable_transform()->mutable_translation()->set_x(
          p->x());
      transform_stamped_msg.mutable_transform()->mutable_translation()->set_y(
          p->y());
      transform_stamped_msg.mutable_transform()->mutable_translation()->set_z(
          p->z());
      transform_stamped_msg.mutable_transform()->mutable_rotation()->CopyFrom(
          *q_W_L);

      transform_stamped_pub_->Publish(transform_stamped_msg);
    }

    if (odometry_pub_->HasConnections()) {
      // DEBUG
      odometry_pub_->Publish(odometry_msg);
    }

    //==============================================//
    //========= BROADCAST TRANSFORM MSG ============//
    //==============================================//

    gz_geometry_msgs::TransformStampedWithFrameIds
        transform_stamped_with_frame_ids_msg;
    transform_stamped_with_frame_ids_msg.mutable_header()->CopyFrom(
        odometry_msg.header());
    transform_stamped_with_frame_ids_msg.mutable_transform()
        ->mutable_translation()
        ->set_x(p->x());
    transform_stamped_with_frame_ids_msg.mutable_transform()
        ->mutable_translation()
        ->set_y(p->y());
    transform_stamped_with_frame_ids_msg.mutable_transform()
        ->mutable_translation()
        ->set_z(p->z());
    transform_stamped_with_frame_ids_msg.mutable_transform()
        ->mutable_rotation()
        ->CopyFrom(*q_W_L);
    transform_stamped_with_frame_ids_msg.set_parent_frame_id(parent_frame_id_);
    transform_stamped_with_frame_ids_msg.set_child_frame_id(child_frame_id_);

    broadcast_transform_pub_->Publish(transform_stamped_with_frame_ids_msg);

  }  // if (gazebo_sequence_ == odometry_queue_.front().first) {

  ++gazebo_sequence_;
}

void GazeboOdometryPlugin::CreatePubsAndSubs() {
  // Create temporary "ConnectGazeboToRosTopic" publisher and message
  gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
          "~/" + kConnectGazeboToRosSubtopic, 1);

  gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;

  // ============================================ //
  // =============== POSE MSG SETUP ============= //
  // ============================================ //

  pose_pub_ = node_handle_->Advertise<gazebo::msgs::Pose>(
      "~/" + namespace_ + "/" + pose_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   pose_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                pose_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::POSE);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);

  // ============================================ //
  // == POSE WITH COVARIANCE STAMPED MSG SETUP == //
  // ============================================ //

  pose_with_covariance_stamped_pub_ =
      node_handle_->Advertise<gz_geometry_msgs::PoseWithCovarianceStamped>(
          "~/" + namespace_ + "/" + pose_with_covariance_stamped_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic(
      "~/" + namespace_ + "/" + pose_with_covariance_stamped_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(
      namespace_ + "/" + pose_with_covariance_stamped_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::POSE_WITH_COVARIANCE_STAMPED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);

  // ============================================ //
  // ========= POSITION STAMPED MSG SETUP ======= //
  // ============================================ //

  position_stamped_pub_ =
      node_handle_->Advertise<gz_geometry_msgs::Vector3dStamped>(
          "~/" + namespace_ + "/" + position_stamped_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   position_stamped_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                position_stamped_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::VECTOR_3D_STAMPED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);

  // ============================================ //
  // ============= ODOMETRY MSG SETUP =========== //
  // ============================================ //

  odometry_pub_ = node_handle_->Advertise<gz_geometry_msgs::Odometry>(
      "~/" + namespace_ + "/" + odometry_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   odometry_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                odometry_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::ODOMETRY);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);

  // ============================================ //
  // ======== TRANSFORM STAMPED MSG SETUP ======= //
  // ============================================ //

  transform_stamped_pub_ =
      node_handle_->Advertise<gz_geometry_msgs::TransformStamped>(
          "~/" + namespace_ + "/" + transform_stamped_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic(
      "~/" + namespace_ + "/" + transform_stamped_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                transform_stamped_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::TRANSFORM_STAMPED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);

  // ============================================ //
  // ===== "BROADCAST TRANSFORM" MSG SETUP =====  //
  // ============================================ //

  broadcast_transform_pub_ =
      node_handle_->Advertise<gz_geometry_msgs::TransformStampedWithFrameIds>(
          "~/" + kBroadcastTransformSubtopic, 1);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboOdometryPlugin);

}  // namespace gazebo

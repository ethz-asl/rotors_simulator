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


#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_ODOMETRY_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_ODOMETRY_PLUGIN_H

#include <cmath>
#include <deque>
#include <random>
#include <stdio.h>

#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <mav_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/core/core.hpp>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <rotors_gazebo_plugins/common.h>
#include <rotors_gazebo_plugins/sdf_api_wrapper.hpp>
#include <tf/transform_broadcaster.h>

namespace gazebo {
// Default values
static const std::string kDefaultParentFrameId = "world";
static const std::string kDefaultChildFrameId = "odometry_sensor";
static const std::string kDefaultLinkName = "odometry_sensor_link";

static constexpr int kDefaultMeasurementDelay = 0;
static constexpr int kDefaultMeasurementDivisor = 1;
static constexpr int kDefaultGazeboSequence = 0;
static constexpr int kDefaultOdometrySequence = 0;
static constexpr double kDefaultUnknownDelay = 0.0;
static constexpr double kDefaultCovarianceImageScale = 1.0;

class GazeboOdometryPlugin : public ModelPlugin {
 public:
  typedef std::normal_distribution<> NormalDistribution;
  typedef std::uniform_real_distribution<> UniformDistribution;
  typedef std::deque<std::pair<int, nav_msgs::Odometry> > OdometryQueue;

  GazeboOdometryPlugin()
      : ModelPlugin(),
        random_generator_(random_device_()),
        pose_pub_topic_(mav_msgs::default_topics::POSE),
        pose_with_covariance_pub_topic_(mav_msgs::default_topics::POSE_WITH_COVARIANCE),
        position_pub_topic_(mav_msgs::default_topics::POSITION),
        transform_pub_topic_(mav_msgs::default_topics::TRANSFORM),
        odometry_pub_topic_(mav_msgs::default_topics::ODOMETRY),
        parent_frame_id_(kDefaultParentFrameId),
        child_frame_id_(kDefaultChildFrameId),
        link_name_(kDefaultLinkName),
        measurement_delay_(kDefaultMeasurementDelay),
        measurement_divisor_(kDefaultMeasurementDivisor),
        unknown_delay_(kDefaultUnknownDelay),
        gazebo_sequence_(kDefaultGazeboSequence),
        odometry_sequence_(kDefaultOdometrySequence),
        covariance_image_scale_(kDefaultCovarianceImageScale),
        node_handle_(NULL) {}

  ~GazeboOdometryPlugin();

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:
  OdometryQueue odometry_queue_;

  std::string namespace_;
  std::string pose_pub_topic_;
  std::string pose_with_covariance_pub_topic_;
  std::string position_pub_topic_;
  std::string transform_pub_topic_;
  std::string odometry_pub_topic_;
  std::string parent_frame_id_;
  std::string child_frame_id_;
  std::string link_name_;

  NormalDistribution position_n_[3];
  NormalDistribution attitude_n_[3];
  NormalDistribution linear_velocity_n_[3];
  NormalDistribution angular_velocity_n_[3];
  UniformDistribution position_u_[3];
  UniformDistribution attitude_u_[3];
  UniformDistribution linear_velocity_u_[3];
  UniformDistribution angular_velocity_u_[3];

  geometry_msgs::PoseWithCovariance::_covariance_type pose_covariance_matrix_;
  geometry_msgs::TwistWithCovariance::_covariance_type twist_covariance_matrix_;

  int measurement_delay_;
  int measurement_divisor_;
  int gazebo_sequence_;
  int odometry_sequence_;
  double unknown_delay_;
  double covariance_image_scale_;
  cv::Mat covariance_image_;

  std::random_device random_device_;
  std::mt19937 random_generator_;

  ros::NodeHandle* node_handle_;
  ros::Publisher pose_pub_;
  ros::Publisher pose_with_covariance_pub_;
  ros::Publisher position_pub_;
  ros::Publisher transform_pub_;
  ros::Publisher odometry_pub_;

  tf::Transform tf_;
  tf::TransformBroadcaster transform_broadcaster_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  physics::EntityPtr parent_link_;

  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread();
};
}

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_ODOMETRY_PLUGIN_H

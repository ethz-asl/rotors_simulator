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


#ifndef MAV_GAZEBO_PLUGINS_GAZEBO_ODOMETRY_PLUGIN_H
#define MAV_GAZEBO_PLUGINS_GAZEBO_ODOMETRY_PLUGIN_H

#include <cmath>
#include <deque>
#include <random>
#include <stdio.h>

#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <ros/callback_queue.h>

namespace gazebo {
// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultFrameId = "odometry_sensor_link";
static const std::string kDefaultLinkName = "odometry_sensor_link";
static const std::string kDefaultOdometryPubTopic = "odometry";

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
        odometry_pub_topic_(kDefaultOdometryPubTopic),
        frame_id_(kDefaultFrameId),
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
  std::string odometry_pub_topic_;
  std::string frame_id_;
  std::string link_name_;

  NormalDistribution position_n_[3];
  NormalDistribution attitude_n_[3];
  NormalDistribution linear_n_[3];
  NormalDistribution angular_n_[3];
  UniformDistribution position_u_[3];
  UniformDistribution attitude_u_[3];
  UniformDistribution linear_u_[3];
  UniformDistribution angular_u_[3];

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
  ros::Publisher odometry_pub_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;

  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread();
};
}

#endif // MAV_GAZEBO_PLUGINS_GAZEBO_ODOMETRY_PLUGIN_H

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


#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_ODOMETRY_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_ODOMETRY_PLUGIN_H

#include <cmath>
#include <deque>
#include <random>
#include <stdio.h>

#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <opencv2/core/core.hpp>

#include <mav_msgs/default_topics.h>  // This comes from the mav_comm repo

#include "rotors_gazebo_plugins/common.h"
#include "rotors_gazebo_plugins/sdf_api_wrapper.hpp"

#include "Odometry.pb.h"


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
  typedef std::deque<std::pair<int, gz_geometry_msgs::Odometry> > OdometryQueue;
  typedef boost::array<double, 36> CovarianceMatrix;

  GazeboOdometryPlugin()
      : ModelPlugin(),
        random_generator_(random_device_()),
        // DEFAULT TOPICS
        pose_pub_topic_(mav_msgs::default_topics::POSE),
        pose_with_covariance_stamped_pub_topic_(mav_msgs::default_topics::POSE_WITH_COVARIANCE),
        position_stamped_pub_topic_(mav_msgs::default_topics::POSITION),
        transform_stamped_pub_topic_(mav_msgs::default_topics::TRANSFORM),
        odometry_pub_topic_(mav_msgs::default_topics::ODOMETRY),
        //---------------
        parent_frame_id_(kDefaultParentFrameId),
        child_frame_id_(kDefaultChildFrameId),
        link_name_(kDefaultLinkName),
        measurement_delay_(kDefaultMeasurementDelay),
        measurement_divisor_(kDefaultMeasurementDivisor),
        unknown_delay_(kDefaultUnknownDelay),
        gazebo_sequence_(kDefaultGazeboSequence),
        odometry_sequence_(kDefaultOdometrySequence),
        covariance_image_scale_(kDefaultCovarianceImageScale),
        pubs_and_subs_created_(false) {}

  ~GazeboOdometryPlugin();

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:

  /// \brief    Flag that is set to true once CreatePubsAndSubs() is called, used
  ///           to prevent CreatePubsAndSubs() from be called on every OnUpdate().
  bool pubs_and_subs_created_;

  /// \brief    Creates all required publishers and subscribers, incl. routing of messages to/from ROS if required.
  /// \details  Call this once the first time OnUpdate() is called (can't
  ///           be called from Load() because there is no guarantee GazeboRosInterfacePlugin has
  ///           has loaded and listening to ConnectGazeboToRosTopic and ConnectRosToGazeboTopic messages).
  void CreatePubsAndSubs();

  OdometryQueue odometry_queue_;

  std::string namespace_;
  std::string pose_pub_topic_;
  std::string pose_with_covariance_stamped_pub_topic_;
  std::string position_stamped_pub_topic_;
  std::string transform_stamped_pub_topic_;
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

  CovarianceMatrix pose_covariance_matrix_;
  CovarianceMatrix twist_covariance_matrix_;

  int measurement_delay_;
  int measurement_divisor_;
  int gazebo_sequence_;
  int odometry_sequence_;
  double unknown_delay_;
  double covariance_image_scale_;
  cv::Mat covariance_image_;

  std::random_device random_device_;
  std::mt19937 random_generator_;

  gazebo::transport::NodePtr node_handle_;

  gazebo::transport::PublisherPtr pose_pub_;
  gazebo::transport::PublisherPtr pose_with_covariance_stamped_pub_;
  gazebo::transport::PublisherPtr position_stamped_pub_;
  gazebo::transport::PublisherPtr transform_stamped_pub_;
  gazebo::transport::PublisherPtr odometry_pub_;

  /// \brief    Special-case publisher to publish stamped transforms with
  ///           frame IDs. The ROS interface plugin (if present) will
  ///           listen to this publisher and broadcast the transform
  ///           using transform_broadcast().
  gazebo::transport::PublisherPtr broadcast_transform_pub_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  physics::EntityPtr parent_link_;

  /// \brief    Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread();
};

} // namespace gazebo

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_ODOMETRY_PLUGIN_H

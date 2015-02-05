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


#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_POSE_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_POSE_PLUGIN_H

#include <cmath>
#include <deque>
#include <random>
#include <stdio.h>

#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "rotors_gazebo_plugins/common.h"

//#include <rotors_gazebo_plugins/pose_distorter.h>

namespace gazebo {
// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultFrameId = "pose_sensor_link";
static const std::string kDefaultLinkName = "pose_sensor_link";
static const std::string kDefaultPosePubTopic = "pose";

static constexpr int kDefaultMeasurementDelay = 0;
static constexpr int kDefaultMeasurementDivisor = 1;
static constexpr int kDefaultGazeboSequence = 0;
static constexpr int kDefaultPoseSequence = 0;
static constexpr double kDefaultUnknownDelay = 0.0;
static constexpr double kDefaultCovarianceImageScale = 1.0;

class GazeboPosePlugin : public ModelPlugin {
 public:
  typedef std::normal_distribution<> NormalDistribution;
  typedef std::uniform_real_distribution<> UniformDistribution;
  typedef std::deque<std::pair<int, geometry_msgs::PoseStamped> > PoseQueue;

  GazeboPosePlugin()
      : ModelPlugin(),
        gen_(rd_()),
        pose_pub_topic_(kDefaultPosePubTopic),
        frame_id_(kDefaultFrameId),
        link_name_(kDefaultLinkName),
        measurement_delay_(kDefaultMeasurementDelay),
        measurement_divisor_(kDefaultMeasurementDivisor),
        unknown_delay_(kDefaultUnknownDelay),
        gazebo_sequence_(kDefaultGazeboSequence),
        pose_sequence_(kDefaultPoseSequence),
        covariance_image_scale_(kDefaultCovarianceImageScale),
        node_handle_(NULL) {}

  ~GazeboPosePlugin();

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:
  PoseQueue pose_queue_;

  std::string namespace_;
  std::string pose_pub_topic_;
  std::string frame_id_;
  std::string link_name_;

  NormalDistribution pos_n_[3];
  NormalDistribution att_n_[3];
  UniformDistribution pos_u_[3];
  UniformDistribution att_u_[3];

  geometry_msgs::PoseWithCovarianceStamped::_pose_type::_covariance_type covariance_matrix_;

  int measurement_delay_;
  int measurement_divisor_;
  int gazebo_sequence_;
  int pose_sequence_;
  double unknown_delay_;
  double covariance_image_scale_;
  cv::Mat covariance_image_;

  std::random_device rd_;
  std::mt19937 gen_;

  ros::NodeHandle* node_handle_;
  ros::Publisher pose_pub_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
    /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread();
};
}

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_POSE_PLUGIN_H

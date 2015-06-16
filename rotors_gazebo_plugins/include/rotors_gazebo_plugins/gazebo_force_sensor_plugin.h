/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Simone Comari, ASL, ETH Zurich, Switzerland
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

#ifndef ROTORS_GAZEBO_PLUGINS_FORCE_SENSOR_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_FORCE_SENSOR_PLUGIN_H

#include <cmath>
#include <deque>
#include <random>
#include <stdio.h>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>

#include <boost/thread.hpp>


namespace gazebo {
// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultParentFrameId = "world";
static const std::string kDefaultReferenceFrameId = "world";
static const std::string kDefaultLinkName = "f2g_sensor_link";
static const std::string kDefaultForceSensorPubTopic = "force_sensor";
static const std::string kDefaultForceSensorTruthPubTopic = "force_sensor_truth";

static constexpr int kDefaultMeasurementDelay = 0;
static constexpr int kDefaultMeasurementDivisor = 1;
static constexpr int kDefaultGazeboSequence = 0;
static constexpr int kDefaultWrenchSequence = 0;
static constexpr double kDefaultUnknownDelay = 0.0;


class GazeboForceSensorPlugin : public ModelPlugin {

 public:
  typedef std::normal_distribution<> NormalDistribution;
  typedef std::uniform_real_distribution<> UniformDistribution;
  typedef std::deque<std::pair<int, geometry_msgs::WrenchStamped> > WrenchQueue;

  GazeboForceSensorPlugin()
      : ModelPlugin(),
        random_generator_(random_device_()),
        parent_frame_id_(kDefaultParentFrameId),
        reference_frame_id_(kDefaultReferenceFrameId),
        link_name_(kDefaultLinkName),
        force_sensor_pub_topic_(kDefaultForceSensorPubTopic),
        force_sensor_truth_pub_topic_(kDefaultForceSensorTruthPubTopic),
        measurement_delay_(kDefaultMeasurementDelay),
        measurement_divisor_(kDefaultMeasurementDivisor),
        unknown_delay_(kDefaultUnknownDelay),
        gazebo_sequence_(kDefaultGazeboSequence),
        wrench_sequence_(kDefaultWrenchSequence),
        node_handle_(NULL) {}

  ~GazeboForceSensorPlugin();

  void InitializeParams();
  void Publish();


 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& /*_info*/);


 private:
  WrenchQueue wrench_queue_;

  std::string namespace_;
  std::string force_sensor_pub_topic_;
  std::string force_sensor_truth_pub_topic_;
  std::string parent_frame_id_;
  std::string reference_frame_id_;
  std::string link_name_;

  NormalDistribution linear_force_n_[3];
  NormalDistribution torque_n_[3];
  UniformDistribution linear_force_u_[3];
  UniformDistribution torque_u_[3];

  geometry_msgs::WrenchStamped wrench_msg_;

  int measurement_delay_;
  int measurement_divisor_;
  int gazebo_sequence_;
  int wrench_sequence_;
  double unknown_delay_;

  std::random_device random_device_;
  std::mt19937 random_generator_;

  ros::NodeHandle* node_handle_;
  ros::Publisher force_sensor_pub_;
  ros::Publisher force_sensor_truth_pub_;

  tf::Transform tf_;
  tf::TransformBroadcaster transform_broadcaster_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  physics::LinkPtr parent_link_;
  physics::LinkPtr reference_link_;

  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;

  void QueueThread();
};
}

#endif /* ROTORS_GAZEBO_PLUGINS_FORCE_SENSOR_PLUGIN_H */

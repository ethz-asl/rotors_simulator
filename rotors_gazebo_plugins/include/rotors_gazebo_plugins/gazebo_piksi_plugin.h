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


#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_PIKSI_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_PIKSI_PLUGIN_H

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
#include <nav_msgs/Odometry.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>
#include <rotors_comm/PiksiRTKPos.h>


namespace gazebo {
// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultParentFrameId = "world";
static const std::string kDefaultLinkName = "piksi_sensor_link";
static const std::string kDefaultSPPPositionPubTopic = "spp_position";
static const std::string kDefaultRTKPositionPubTopic = "rtk_position";

class GazeboPiksiPlugin : public ModelPlugin {
 public:
  typedef std::normal_distribution<> NormalDistribution;
  typedef std::uniform_real_distribution<> UniformDistribution;

  GazeboPiksiPlugin()
      : ModelPlugin(),
        random_generator_(random_device_()),
        spp_position_pub_topic_(kDefaultSPPPositionPubTopic),
        rtk_position_pub_topic_(kDefaultRTKPositionPubTopic),
        parent_frame_id_(kDefaultParentFrameId),
        link_name_(kDefaultLinkName),
        node_handle_(NULL) {}

  ~GazeboPiksiPlugin();

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:
  double update_rate_ = 10;        // default update rate: 10Hz
  common::Time prev_update_time_;

  double lat_start;
  double lon_start;
  double alt_start;
  double m_to_lat;
  double m_to_lon;
  double lat_to_m;
  double lon_to_m;
  const double rtk_float_start_error_width = 500;  //[m]

  std::string namespace_;
  std::string spp_position_pub_topic_;
  std::string rtk_position_pub_topic_;
  std::string parent_frame_id_;
  std::string link_name_;
  std::string publish_ground_truth_;

  sdf::Vector3 offset_spp_;
  sdf::Vector3 offset_rtk_fixed_;
  sdf::Vector3 gps_start_position_;

  NormalDistribution rtk_position_n_[3];
  NormalDistribution spp_position_n_[3];

  std::random_device random_device_;
  std::mt19937 random_generator_;

  sensor_msgs::NavSatFix sol_spp_;
  sensor_msgs::NavSatFix sol_gt_;
  rotors_comm::PiksiRTKPos sol_rtk_;

  ros::NodeHandle* node_handle_;
  ros::Publisher spp_position_pub_;
  ros::Publisher rtk_position_pub_;
  ros::Publisher ground_truth_pub_;

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

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_PIKSI_PLUGIN_H

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


#ifndef MAV_GAZEBO_PLUGINS_GAZEBO_CAMERA_TRAJECTORY_PLUGIN_H
#define MAV_GAZEBO_PLUGINS_GAZEBO_CAMERA_TRAJECTORY_PLUGIN_H

#include <Eigen/Dense>
#include <random>
#include <cmath>
#include <deque>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <stdio.h>
#include <sensor_msgs/Imu.h>

#include <mav_planning_utils/path_planning.h>
#include <mav_planning_utils/segment_planning.h>
#include <planning_msgs/WayPointArray.h>

namespace gazebo {

class GazeboCameraTrajectoryPlugin : public ModelPlugin {
 public:
  typedef std::normal_distribution<> NormalDistribution;
  typedef std::uniform_real_distribution<> UniformDistribution;

  GazeboCameraTrajectoryPlugin();
  ~GazeboCameraTrajectoryPlugin();

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:
  static const int N_ = 12;
  std::string namespace_;
  std::string imu_topic_;
  std::string path_segments_topic_;
  ros::NodeHandle* node_handle_;
  ros::Publisher imu_pub_;
  ros::Subscriber path_sub_;
  std::string frame_id_;
  std::string link_name_;
  common::Time last_time_;
  common::Time path_received_time_;

  typename mav_planning_utils::path_planning::Segment<N_>::Vector sx_;
  typename mav_planning_utils::path_planning::Segment<N_>::Vector sy_;
  typename mav_planning_utils::path_planning::Segment<N_>::Vector sz_;
  typename mav_planning_utils::path_planning::Segment<N_>::Vector syaw_;

  bool follow_path_;
  int gazebo_seq_;
  int pose_seq_;
  double v_max_;
  double a_max_;
  double yaw_max_;
  double path_time_;

  // Pointer to the world
  physics::WorldPtr world_;
  // Pointer to the model
  physics::ModelPtr model_;
  // Pointer to the link
  physics::LinkPtr link_;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;

  void QueueThread();
  bool createSegments(const planning_msgs::WayPointArray::_waypoints_type& wpts);
  void pathCb(const planning_msgs::WayPointArrayConstPtr& msg);
};
}

#endif // MAV_GAZEBO_PLUGINS_GAZEBO_CAMERA_TRAJECTORY_PLUGIN_H

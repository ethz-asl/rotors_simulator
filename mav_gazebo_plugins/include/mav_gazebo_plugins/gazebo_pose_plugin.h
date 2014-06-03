//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>
// All rights reserved.
//
// TODO(ff): Enter some license
//==============================================================================
#ifndef MAV_GAZEBO_PLUGINS_GAZEBO_POSE_PLUGIN_H
#define MAV_GAZEBO_PLUGINS_GAZEBO_POSE_PLUGIN_H

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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

//#include <mav_gazebo_plugins/pose_distorter.h>

namespace gazebo {

class GazeboPosePlugin : public ModelPlugin {
 public:
  typedef std::normal_distribution<> NormalDistribution;
  typedef std::uniform_real_distribution<> UniformDistribution;
  typedef std::deque<std::pair<int, geometry_msgs::PoseStamped> > PoseQueue;

  GazeboPosePlugin();
  ~GazeboPosePlugin();

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:
  PoseQueue pose_queue_;

  std::string namespace_;
  std::string pose_topic_;
  ros::NodeHandle* node_handle_;
  ros::Publisher pose_pub_;
  std::string frame_id_;
  std::string link_name_;

  std::random_device rd_;
  std::mt19937 gen_;

  NormalDistribution pos_n_[3];
  NormalDistribution att_n_[3];
  UniformDistribution pos_u_[3];
  UniformDistribution att_u_[3];

  int measurement_delay_;
  int measurement_divisor_;
  double noise_normal_q_;
  double noise_normal_p_;
  double noise_uniform_q_;
  double noise_uniform_p_;

  int gazebo_seq_;
  int pose_seq_;

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
};
}

#endif // MAV_GAZEBO_PLUGINS_GAZEBO_POSE_PLUGIN_H

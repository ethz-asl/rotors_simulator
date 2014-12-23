/*
 * gazebo_imu_plugin.h
 *
 *  Created on: Dec 22, 2014
 *      Author: burrimi
 */

#ifndef MAV_GAZEBO_PLUGINS_GAZEBO_IMU_PLUGIN_H
#define MAV_GAZEBO_PLUGINS_GAZEBO_IMU_PLUGIN_H

#include <Eigen/Dense>
#include <random>
#include <cmath>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <stdio.h>
#include <sensor_msgs/Imu.h>



namespace gazebo {

class GazeboImuPlugin : public ModelPlugin {
 public:
  typedef std::normal_distribution<> NormalDistribution;
  typedef std::uniform_real_distribution<> UniformDistribution;

  GazeboImuPlugin();
  ~GazeboImuPlugin();

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  void addNoise(Eigen::Vector3d* linear_acceleration, Eigen::Vector3d* angular_velocity);

  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:
  std::string namespace_;
  std::string imu_topic_;
  ros::NodeHandle* node_handle_;
  ros::Publisher imu_pub_;
  std::string frame_id_;
  std::string link_name_;

  // Pointer to the world
  physics::WorldPtr world_;
  // Pointer to the model
  physics::ModelPtr model_;
  // Pointer to the link
  physics::LinkPtr link_;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection_;

  common::Time last_time_;

  sensor_msgs::Imu imu_msg;

  math::Vector3 gravity_W_;
  math::Vector3 velocity_prev_W_;
};
}

#endif // MAV_GAZEBO_PLUGINS_GAZEBO_IMU_PLUGIN_H

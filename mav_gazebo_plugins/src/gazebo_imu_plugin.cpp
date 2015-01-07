/*
 * gazebo_imu_plugin.cpp
 *
 *  Created on: Dec 22, 2014
 *      Author: burrimi
 */

#include "mav_gazebo_plugins/gazebo_imu_plugin.h"

#include <boost/bind.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

namespace gazebo {

GazeboImuPlugin::GazeboImuPlugin()
    : ModelPlugin(),
      node_handle_(0),
      velocity_prev_W_(0,0,0)
{
}

GazeboImuPlugin::~GazeboImuPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}


// void GazeboImuPlugin::InitializeParams() {};
// void GazeboImuPlugin::Publish() {};

void GazeboImuPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  // default params
  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_pose_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_imu_plugin] Please specify a linkName.\n";
  // Get the pointer to the link
  link_ = this->model_->GetLink(link_name_);
  frame_id_ = link_name_;

  getSdfParam<std::string>(_sdf, "imuTopic", imu_topic_, kDefaultImuTopic);
  getSdfParam<double>(_sdf, "gyroscopeNoiseDensity", imu_parameters_.gyroscope_noise_density,
                      imu_parameters_.gyroscope_noise_density);
  getSdfParam<double>(_sdf, "gyroscopeBiasRandomWalk", imu_parameters_.gyroscope_random_walk,
                      imu_parameters_.gyroscope_random_walk);
  getSdfParam<double>(_sdf, "gyroscopeBiasCorrelationTime", imu_parameters_.gyroscope_bias_correlation_time,
                      imu_parameters_.gyroscope_bias_correlation_time);
  getSdfParam<double>(_sdf, "gyroscopeTurnOnBiasSigma", imu_parameters_.gyroscope_turn_on_bias_sigma,
                      imu_parameters_.gyroscope_turn_on_bias_sigma);
  getSdfParam<double>(_sdf, "accelerometerNoiseDensity", imu_parameters_.accelerometer_noise_density,
                      imu_parameters_.accelerometer_noise_density);
  getSdfParam<double>(_sdf, "accelerometerBiasRandomWalk", imu_parameters_.accelerometer_bias_random_walk,
                      imu_parameters_.accelerometer_bias_random_walk);
  getSdfParam<double>(_sdf, "accelerometerBiasCorrelationTime", imu_parameters_.accelerometer_bias_correlation_time,
                      imu_parameters_.accelerometer_bias_correlation_time);
  getSdfParam<double>(_sdf, "accelerometerTurnOnBiasSigma", imu_parameters_.accelerometer_turn_on_bias_sigma,
                      imu_parameters_.accelerometer_turn_on_bias_sigma);


  last_time_ = world_->GetSimTime();

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboImuPlugin::OnUpdate, this, _1));

  imu_pub_ = node_handle_->advertise<sensor_msgs::Imu>(imu_topic_, 10);

  // Fill imu message.
  imu_msg.header.frame_id = frame_id_;
  // TODO(nikolic): Check this.
  imu_msg.angular_velocity_covariance[0] = imu_parameters_.gyroscope_noise_density *
      imu_parameters_.gyroscope_noise_density;
  imu_msg.angular_velocity_covariance[4] = imu_parameters_.gyroscope_noise_density *
      imu_parameters_.gyroscope_noise_density;
  imu_msg.angular_velocity_covariance[8] = imu_parameters_.gyroscope_noise_density *
      imu_parameters_.gyroscope_noise_density;
  imu_msg.linear_acceleration_covariance[0] = imu_parameters_.accelerometer_noise_density *
      imu_parameters_.accelerometer_noise_density;
  imu_msg.linear_acceleration_covariance[4] = imu_parameters_.accelerometer_noise_density *
      imu_parameters_.accelerometer_noise_density;
  imu_msg.linear_acceleration_covariance[8] = imu_parameters_.accelerometer_noise_density *
      imu_parameters_.accelerometer_noise_density;

  gravity_W_ = world_->GetPhysicsEngine()->GetGravity();
  imu_parameters_.gravity_magnitude = gravity_W_.GetLength();
}

void GazeboImuPlugin::addNoise(const double dt, Eigen::Vector3d* linear_acceleration, Eigen::Vector3d* angular_velocity) {
  assert(linear_acceleration);
  assert(angular_velocity);

  //TODO(nikolic): Add noise.

}

// Called by the world update start event
void GazeboImuPlugin::OnUpdate(const common::UpdateInfo& _info) {
  common::Time current_time  = world_->GetSimTime();
  double dt = (current_time - last_time_).Double();
  last_time_ = current_time;
  double t = current_time.Double();

  math::Pose T_W_I = link_->GetWorldPose(); //TODO(burrimi): Check transformation.
  math::Quaternion C_W_I = T_W_I.rot;

  math::Vector3 velocity_current_W = link_->GetWorldLinearVel();

  // link_->GetRelativeLinearAccel() does not work.
  math::Vector3 acceleration = (velocity_current_W - velocity_prev_W_) / dt;
  math::Vector3 acceleration_I = C_W_I.RotateVectorReverse(acceleration - gravity_W_);
  math::Vector3 angular_vel_I = link_->GetRelativeAngularVel();

  Eigen::Vector3d linear_acceleration_I(acceleration_I.x, acceleration_I.y, acceleration_I.z);
  Eigen::Vector3d angular_velocity_I(angular_vel_I.x, angular_vel_I.y, angular_vel_I.z);

  addNoise(dt, &linear_acceleration_I, &angular_velocity_I);

  // Fill IMU message.
  imu_msg.header.stamp.sec = current_time.sec;
  imu_msg.header.stamp.nsec = current_time.nsec;

  // TODO(burrimi): Add noise to orientation.
  imu_msg.orientation.w = C_W_I.w;
  imu_msg.orientation.x = C_W_I.x;
  imu_msg.orientation.y = C_W_I.y;
  imu_msg.orientation.z = C_W_I.z;

  imu_msg.linear_acceleration.x = linear_acceleration_I(0);
  imu_msg.linear_acceleration.y = linear_acceleration_I(1);
  imu_msg.linear_acceleration.z = linear_acceleration_I(2);
  imu_msg.angular_velocity.x = angular_velocity_I(0);
  imu_msg.angular_velocity.y = angular_velocity_I(1);
  imu_msg.angular_velocity.z = angular_velocity_I(2);

  imu_pub_.publish(imu_msg);

  velocity_prev_W_ = velocity_current_W;
}


GZ_REGISTER_MODEL_PLUGIN(GazeboImuPlugin);
}

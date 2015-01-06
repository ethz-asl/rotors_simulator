/*
 * gazebo_imu_plugin.h
 *
 *  Created on: Dec 22, 2014
 *      Author: burrimi
 */

#ifndef MAV_GAZEBO_PLUGINS_GAZEBO_IMU_PLUGIN_H
#define MAV_GAZEBO_PLUGINS_GAZEBO_IMU_PLUGIN_H

#include <random>

#include <Eigen/Core>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Imu.h>

#include "mav_gazebo_plugins/common.h"

namespace gazebo {

struct ImuParameters {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Gyro noise density. [rad*sqrt(s)/s]
  double gyro_noise_density;
  /// Gyro bias random walk. [rad/s]
  double gyro_bias_random_walk;
  /// Gyro bias correlation time constant [s]
  double gyro_bias_correlation_time;
  /// Gyro turn on bias standart deviation [rad/s]
  double gyro_turn_on_bias_sigma;
  /// Accelerometer noise density. [m*sqrt(s)/s^2]
  double acc_noise_density;
  /// Accelerometer bias random walk. [m/s^2]
  double acc_bias_random_walk;
  /// Accelerometer turn on bias standart deviation [m/s/s]
  double acc_bias_correlation_time;
  /// Accelerometer bias correlation time constant [s]
  double acc_turn_on_bias_sigma;
//  /// Square root of bias convergence time constant
//  double bias_tau_sqrt;
//  /// IMU integration sigma
//  double imu_integration_sigma;
  /// Norm of the Gravitational acceleration. [m/s^2]
  double gravity_magnitude;


  ImuParameters()
      : gyro_noise_density(0.000339198),  // Default values for the ADIS16448 IMU
        gyro_bias_random_walk(0.000038765),
        gyro_bias_correlation_time(360),
        gyro_turn_on_bias_sigma(0),
        acc_noise_density(0.004),
        acc_bias_random_walk(0.006),
        acc_bias_correlation_time(360),
        acc_turn_on_bias_sigma(0),
//        bias_tau_sqrt(189.74),
//        imu_integration_sigma(0.0),
        gravity_magnitude(9.80665) {}
};

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

  void addNoise(const double dt, Eigen::Vector3d* linear_acceleration, Eigen::Vector3d* angular_velocity);

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

  ImuParameters imu_parameters_;
};
}

#endif // MAV_GAZEBO_PLUGINS_GAZEBO_IMU_PLUGIN_H

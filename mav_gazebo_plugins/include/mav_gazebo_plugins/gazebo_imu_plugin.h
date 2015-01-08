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

// Default values for use with ADIS16448 IMU
static constexpr double kDefaultAdisGyroscopeNoiseDensity =
    2.0 * 35.0 / 3600.0 / 180.0 * M_PI;
static constexpr double kDefaultAdisGyroscopeRandomWalk =
    2.0 * 4.0 / 3600.0 / 180.0 * M_PI;
static constexpr double kDefaultAdisGyroscopeBiasCorrelationTime =
    1.0e+3;
static constexpr double kDefaultAdisGyroscopeTurnOnBiasSigma =
    0.5 / 180.0 * M_PI;
static constexpr double kDefaultAdisAccelerometerNoiseDensity =
    2.0 * 2.0e-3;
static constexpr double kDefaultAdisAccelerometerBiasRandomWalk =
    2.0 * 3.0e-3;
static constexpr double kDefaultAdisAccelerometerBiasCorrelationTime =
    300.0;
static constexpr double kDefaultAdisAccelerometerTurnOnBiasSigma =
    20.0e-3 * 9.8;
// Earth's gravity in Zurich (lat=+47.3667degN, lon=+8.5500degE, h=+500m, WGS84)
static constexpr double kDefaultGravityMagnitude = 9.8068;

static const std::string kDefaultImuTopic = "imu";

// A description of the parameters:
// https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model-and-Intrinsics
// TODO(burrimi): Should we have a minimalistic description of the params here?
struct ImuParameters {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Gyroscope noise density (two-sided spectrum) [rad/s/sqrt(Hz)]
  double gyroscope_noise_density;
  /// Gyroscope bias random walk [rad/s/s/sqrt(Hz)]
  double gyroscope_random_walk;
  /// Gyroscope bias correlation time constant [s]
  double gyroscope_bias_correlation_time;
  /// Gyroscope turn on bias standard deviation [rad/s]
  double gyroscope_turn_on_bias_sigma;
  /// Accelerometer noise density (two-sided spectrum) [m/s^2/sqrt(Hz)]
  double accelerometer_noise_density;
  /// Accelerometer bias random walk. [m/s^2/s/sqrt(Hz)]
  double accelerometer_bias_random_walk;
  /// Accelerometer bias correlation time constant [s]
  double accelerometer_bias_correlation_time;
  /// Accelerometer turn on bias standard deviation [m/s^2]
  double accelerometer_turn_on_bias_sigma;
  /// Norm of the gravitational acceleration [m/s^2]
  double gravity_magnitude;


  ImuParameters()
      : gyroscope_noise_density(kDefaultAdisGyroscopeNoiseDensity),
        gyroscope_random_walk(kDefaultAdisGyroscopeRandomWalk),
        gyroscope_bias_correlation_time(
            kDefaultAdisGyroscopeBiasCorrelationTime),
        gyroscope_turn_on_bias_sigma(kDefaultAdisGyroscopeTurnOnBiasSigma),
        accelerometer_noise_density(kDefaultAdisAccelerometerNoiseDensity),
        accelerometer_bias_random_walk(kDefaultAdisAccelerometerBiasRandomWalk),
        accelerometer_bias_correlation_time(
            kDefaultAdisAccelerometerBiasCorrelationTime),
        accelerometer_turn_on_bias_sigma(
            kDefaultAdisAccelerometerTurnOnBiasSigma),
        gravity_magnitude(kDefaultGravityMagnitude) {}
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

  void addNoise(
      const double dt,
      Eigen::Vector3d* linear_acceleration,
      Eigen::Vector3d* angular_velocity);

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

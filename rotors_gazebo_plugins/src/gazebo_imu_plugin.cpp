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

// MODULE HEADER
#include "rotors_gazebo_plugins/gazebo_imu_plugin.h"

// SYSTEM LIBS
#include <stdio.h>
#include <boost/bind.hpp>
#include <chrono>
#include <cmath>
#include <iostream>

// 3RD PARTY
#include "mav_msgs/default_topics.h"

// USER HEADERS
#include "ConnectGazeboToRosTopic.pb.h"

namespace gazebo {

GazeboImuPlugin::GazeboImuPlugin()
    : ModelPlugin(),
      node_handle_(0),
      velocity_prev_W_(0, 0, 0),
      pubs_and_subs_created_(false) {}

GazeboImuPlugin::~GazeboImuPlugin() {
}

void GazeboImuPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  gzdbg << "_model = " << _model->GetName() << std::endl;

  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  // default params
  namespace_.clear();

  //==============================================//
  //========== READ IN PARAMS FROM SDF ===========//
  //==============================================//

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_imu_plugin] Please specify a robotNamespace.\n";

  // Get node handle
  node_handle_ = transport::NodePtr(new transport::Node());

  // Initialise with default namespace (typically /gazebo/default/)
  node_handle_->Init();

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_imu_plugin] Please specify a linkName.\n";
  // Get the pointer to the link
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_imu_plugin] Couldn't find specified link \"" << link_name_
                                                                  << "\".");

  frame_id_ = link_name_;

  getSdfParam<std::string>(_sdf, "imuTopic", imu_topic_,
                           mav_msgs::default_topics::IMU);
  getSdfParam<double>(_sdf, "gyroscopeNoiseDensity",
                      imu_parameters_.gyroscope_noise_density,
                      imu_parameters_.gyroscope_noise_density);
  getSdfParam<double>(_sdf, "gyroscopeBiasRandomWalk",
                      imu_parameters_.gyroscope_random_walk,
                      imu_parameters_.gyroscope_random_walk);
  getSdfParam<double>(_sdf, "gyroscopeBiasCorrelationTime",
                      imu_parameters_.gyroscope_bias_correlation_time,
                      imu_parameters_.gyroscope_bias_correlation_time);
  assert(imu_parameters_.gyroscope_bias_correlation_time > 0.0);
  getSdfParam<double>(_sdf, "gyroscopeTurnOnBiasSigma",
                      imu_parameters_.gyroscope_turn_on_bias_sigma,
                      imu_parameters_.gyroscope_turn_on_bias_sigma);
  getSdfParam<double>(_sdf, "accelerometerNoiseDensity",
                      imu_parameters_.accelerometer_noise_density,
                      imu_parameters_.accelerometer_noise_density);
  getSdfParam<double>(_sdf, "accelerometerRandomWalk",
                      imu_parameters_.accelerometer_random_walk,
                      imu_parameters_.accelerometer_random_walk);
  getSdfParam<double>(_sdf, "accelerometerBiasCorrelationTime",
                      imu_parameters_.accelerometer_bias_correlation_time,
                      imu_parameters_.accelerometer_bias_correlation_time);
  assert(imu_parameters_.accelerometer_bias_correlation_time > 0.0);
  getSdfParam<double>(_sdf, "accelerometerTurnOnBiasSigma",
                      imu_parameters_.accelerometer_turn_on_bias_sigma,
                      imu_parameters_.accelerometer_turn_on_bias_sigma);

  last_time_ = world_->SimTime();

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboImuPlugin::OnUpdate, this, _1));

  //==============================================//
  //====== POPULATE STATIC PARTS OF IMU MSG ======//
  //==============================================//

  //  imu_message_.header.frame_id = frame_id_;
  imu_message_.mutable_header()->set_frame_id(frame_id_);

  // We assume uncorrelated noise on the 3 channels -> only set diagonal
  // elements. Only the broadband noise component is considered, specified as a
  // continuous-time density (two-sided spectrum); not the true covariance of
  // the measurements.

  for (int i = 0; i < 9; i++) {
    switch (i) {
      case 0:
        imu_message_.add_angular_velocity_covariance(
            imu_parameters_.gyroscope_noise_density *
            imu_parameters_.gyroscope_noise_density);

        imu_message_.add_orientation_covariance(-1.0);

        imu_message_.add_linear_acceleration_covariance(
            imu_parameters_.accelerometer_noise_density *
            imu_parameters_.accelerometer_noise_density);
        break;
      case 1:
      case 2:
      case 3:
        imu_message_.add_angular_velocity_covariance(0.0);

        imu_message_.add_orientation_covariance(-1.0);

        imu_message_.add_linear_acceleration_covariance(0.0);
        break;
      case 4:
        imu_message_.add_angular_velocity_covariance(
            imu_parameters_.gyroscope_noise_density *
            imu_parameters_.gyroscope_noise_density);

        imu_message_.add_orientation_covariance(-1.0);

        imu_message_.add_linear_acceleration_covariance(
            imu_parameters_.accelerometer_noise_density *
            imu_parameters_.accelerometer_noise_density);
        break;
      case 5:
      case 6:
      case 7:
        imu_message_.add_angular_velocity_covariance(0.0);

        imu_message_.add_orientation_covariance(-1.0);

        imu_message_.add_linear_acceleration_covariance(0.0);
        break;
      case 8:
        imu_message_.add_angular_velocity_covariance(
            imu_parameters_.gyroscope_noise_density *
            imu_parameters_.gyroscope_noise_density);

        imu_message_.add_orientation_covariance(-1.0);

        imu_message_.add_linear_acceleration_covariance(
            imu_parameters_.accelerometer_noise_density *
            imu_parameters_.accelerometer_noise_density);
        break;
    }
  }

  gravity_W_ = world_->Gravity();
  imu_parameters_.gravity_magnitude = gravity_W_.Length();

  standard_normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);

  double sigma_bon_g = imu_parameters_.gyroscope_turn_on_bias_sigma;
  double sigma_bon_a = imu_parameters_.accelerometer_turn_on_bias_sigma;
  for (int i = 0; i < 3; ++i) {
    gyroscope_turn_on_bias_[i] =
        sigma_bon_g * standard_normal_distribution_(random_generator_);
    accelerometer_turn_on_bias_[i] =
        sigma_bon_a * standard_normal_distribution_(random_generator_);
  }

  // TODO(nikolicj) incorporate steady-state covariance of bias process
  gyroscope_bias_.setZero();
  accelerometer_bias_.setZero();
}

void GazeboImuPlugin::AddNoise(Eigen::Vector3d* linear_acceleration,
                               Eigen::Vector3d* angular_velocity,
                               const double dt) {
  GZ_ASSERT(linear_acceleration != nullptr, "Linear acceleration was null.");
  GZ_ASSERT(angular_velocity != nullptr, "Angular velocity was null.");
  GZ_ASSERT(dt > 0.0, "Change in time must be greater than 0.");

  // Gyrosocpe
  double tau_g = imu_parameters_.gyroscope_bias_correlation_time;
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigma_g_d = 1 / sqrt(dt) * imu_parameters_.gyroscope_noise_density;
  double sigma_b_g = imu_parameters_.gyroscope_random_walk;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_g_d = sqrt(-sigma_b_g * sigma_b_g * tau_g / 2.0 *
                            (exp(-2.0 * dt / tau_g) - 1.0));
  // Compute state-transition.
  double phi_g_d = exp(-1.0 / tau_g * dt);
  // Simulate gyroscope noise processes and add them to the true angular rate.
  for (int i = 0; i < 3; ++i) {
    gyroscope_bias_[i] =
        phi_g_d * gyroscope_bias_[i] +
        sigma_b_g_d * standard_normal_distribution_(random_generator_);
    (*angular_velocity)[i] =
        (*angular_velocity)[i] + gyroscope_bias_[i] +
        sigma_g_d * standard_normal_distribution_(random_generator_) +
        gyroscope_turn_on_bias_[i];
  }

  // Accelerometer
  double tau_a = imu_parameters_.accelerometer_bias_correlation_time;
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigma_a_d = 1 / sqrt(dt) * imu_parameters_.accelerometer_noise_density;
  double sigma_b_a = imu_parameters_.accelerometer_random_walk;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_a_d = sqrt(-sigma_b_a * sigma_b_a * tau_a / 2.0 *
                            (exp(-2.0 * dt / tau_a) - 1.0));
  // Compute state-transition.
  double phi_a_d = exp(-1.0 / tau_a * dt);
  // Simulate accelerometer noise processes and add them to the true linear
  // acceleration.
  for (int i = 0; i < 3; ++i) {
    accelerometer_bias_[i] =
        phi_a_d * accelerometer_bias_[i] +
        sigma_b_a_d * standard_normal_distribution_(random_generator_);
    (*linear_acceleration)[i] =
        (*linear_acceleration)[i] + accelerometer_bias_[i] +
        sigma_a_d * standard_normal_distribution_(random_generator_) +
        accelerometer_turn_on_bias_[i];
  }
}

void GazeboImuPlugin::OnUpdate(const common::UpdateInfo& _info) {
  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  common::Time current_time = world_->SimTime();
  double dt = (current_time - last_time_).Double();
  last_time_ = current_time;
  double t = current_time.Double();

  ignition::math::Pose3d T_W_I = link_->WorldPose();  // TODO(burrimi): Check tf.
  ignition::math::Quaterniond C_W_I = T_W_I.Rot();

  ignition::math::Vector3d acceleration_I =
      link_->RelativeLinearAccel() - C_W_I.RotateVectorReverse(gravity_W_);

  ignition::math::Vector3d angular_vel_I = link_->RelativeAngularVel();

  Eigen::Vector3d linear_acceleration_I(acceleration_I.X(), acceleration_I.Y(),
                                        acceleration_I.Z());
  Eigen::Vector3d angular_velocity_I(angular_vel_I.X(), angular_vel_I.Y(),
                                     angular_vel_I.Z());

  AddNoise(&linear_acceleration_I, &angular_velocity_I, dt);

  // Fill IMU message.
  //  imu_message_.header.stamp.sec = current_time.sec;
  imu_message_.mutable_header()->mutable_stamp()->set_sec(current_time.sec);

  //  imu_message_.header.stamp.nsec = current_time.nsec;
  imu_message_.mutable_header()->mutable_stamp()->set_nsec(current_time.nsec);

  /// \todo(burrimi): Add orientation estimator.
  // NOTE: rotors_simulator used to set the orientation to "0", since it is
  // not raw IMU data but rather a calculation (and could confuse users).
  // However, the orientation is now set as it is used by PX4.
  /*gazebo::msgs::Quaternion* orientation = new gazebo::msgs::Quaternion();
  orientation->set_x(0);
  orientation->set_y(0);
  orientation->set_z(0);
  orientation->set_w(1);
  imu_message_.set_allocated_orientation(orientation);*/

  /// \todo(burrimi): add noise.
  gazebo::msgs::Quaternion* orientation = new gazebo::msgs::Quaternion();
  orientation->set_w(C_W_I.W());
  orientation->set_x(C_W_I.X());
  orientation->set_y(C_W_I.Y());
  orientation->set_z(C_W_I.Z());
  imu_message_.set_allocated_orientation(orientation);

  gazebo::msgs::Vector3d* linear_acceleration = new gazebo::msgs::Vector3d();
  linear_acceleration->set_x(linear_acceleration_I[0]);
  linear_acceleration->set_y(linear_acceleration_I[1]);
  linear_acceleration->set_z(linear_acceleration_I[2]);
  imu_message_.set_allocated_linear_acceleration(linear_acceleration);

  gazebo::msgs::Vector3d* angular_velocity = new gazebo::msgs::Vector3d();
  angular_velocity->set_x(angular_velocity_I[0]);
  angular_velocity->set_y(angular_velocity_I[1]);
  angular_velocity->set_z(angular_velocity_I[2]);
  imu_message_.set_allocated_angular_velocity(angular_velocity);

  // Publish the IMU message
  imu_pub_->Publish(imu_message_);

  // std::cout << "Published IMU message.\n";
}

void GazeboImuPlugin::CreatePubsAndSubs() {
  // Create temporary "ConnectGazeboToRosTopic" publisher and message
  gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
          "~/" + kConnectGazeboToRosSubtopic, 1);

  // ============================================ //
  // =============== IMU MSG SETUP ============== //
  // ============================================ //

  imu_pub_ = node_handle_->Advertise<gz_sensor_msgs::Imu>(
      "~/" + namespace_ + "/" + imu_topic_, 1);

  gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;
  // connect_gazebo_to_ros_topic_msg.set_gazebo_namespace(namespace_);
  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   imu_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" + imu_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::IMU);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboImuPlugin);

}  // namespace gazebo

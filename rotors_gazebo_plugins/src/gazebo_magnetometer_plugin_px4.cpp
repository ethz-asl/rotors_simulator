/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Magnetometer Plugin
 *
 * This plugin simulates magnetometer data
 *
 * @author Elia Tarasov <elias.tarasov@gmail.com>
 */

#include <rotors_gazebo_plugins/gazebo_magnetometer_plugin_px4.h>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(MagnetometerPlugin)

MagnetometerPlugin::MagnetometerPlugin() : ModelPlugin(),
      groundtruth_lat_rad_(0.0),
      groundtruth_lon_rad_(0.0)
{
}

MagnetometerPlugin::~MagnetometerPlugin()
{
  update_connection_->~Connection();
}

void MagnetometerPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  model_ = model;
  world_ = model_->GetWorld();

  namespace_.clear();
  if (sdf->HasElement("robotNamespace")) {
    namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_magnetometer_plugin] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init();

  if (sdf->HasElement("pubRate")) {
    pub_rate_ = sdf->GetElement("pubRate")->Get<unsigned int>();
  } else {
    pub_rate_ = kDefaultPubRate;
    gzwarn << "[gazebo_magnetometer_plugin] Using default publication rate of " << pub_rate_ << " Hz\n";
  }

  if (sdf->HasElement("noiseDensity")) {
    noise_density_ = sdf->GetElement("noiseDensity")->Get<double>();
  } else {
    noise_density_ = kDefaultNoiseDensity;
    gzwarn << "[gazebo_magnetometer_plugin] Using default noise density of " << noise_density_ << " (gauss) / sqrt(hz)\n";
  }

  if (sdf->HasElement("randomWalk")) {
    random_walk_ = sdf->GetElement("randomWalk")->Get<double>();
  } else {
    random_walk_ = kDefaultRandomWalk;
    gzwarn << "[gazebo_magnetometer_plugin] Using default random walk of " << random_walk_ << " (gauss) * sqrt(hz)\n";
  }

  if (sdf->HasElement("biasCorrelationTime")) {
    bias_correlation_time_ = sdf->GetElement("biasCorrelationTime")->Get<double>();
  } else {
    bias_correlation_time_ = kDefaultBiasCorrelationTime;
    gzwarn << "[gazebo_magnetometer_plugin] Using default bias correlation time of " << random_walk_ << " s\n";
  }

  if(sdf->HasElement("magTopic")) {
    mag_topic_ = sdf->GetElement("magTopic")->Get<std::string>();
  } else {
    mag_topic_ = kDefaultMagnetometerTopic;
    gzwarn << "[gazebo_magnetometer_plugin] Using default magnetometer topic " << mag_topic_ << "\n";
  }

  gt_sub_topic_ = "/groundtruth";

#if GAZEBO_MAJOR_VERSION >= 9
  last_time_ = world_->SimTime();
  last_pub_time_ = world_->SimTime();
#else
  last_time_ = world_->GetSimTime();
  last_pub_time_ = world_->GetSimTime();
#endif

  // Listen to the update event. This event is broadcast every simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&MagnetometerPlugin::OnUpdate, this, _1));

  gt_sub_ = node_handle_->Subscribe("~/" + namespace_ + gt_sub_topic_, &MagnetometerPlugin::GroundtruthCallback, this);
  std::cout<<"\33[1m[gazebo_magnetometer_plugin]\33[0m Subscribing to \33[1;34m" << gt_sub_->GetTopic() << "\33[0m gazebo message\n";

  pub_mag_ = node_handle_->Advertise<gz_sensor_msgs::MagneticField>("~/" + namespace_ + mag_topic_, 10);
  std::cout<<"\33[1m[gazebo_magnetometer_plugin]\33[0m Advertising \33[1;32m" << pub_mag_->GetTopic() << "\33[0m gazebo message\n";

  standard_normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);

  bias_.setZero();

  for (auto i = 0; i < 9; ++i) {
    switch (i){
      // principal diagonal = the variance of the random variables
      // = noise_density²
      case 0: case 4: case 8:
        mag_message_.add_magnetic_field_covariance(noise_density_ * noise_density_);
        break;
      default:
        mag_message_.add_magnetic_field_covariance(0.0);
    }
  }
}

void MagnetometerPlugin::GroundtruthCallback(GtPtr& gt_msg) {
  // update groundtruth lat_rad, lon_rad and altitude
  groundtruth_lat_rad_ = gt_msg->latitude_rad();
  groundtruth_lon_rad_ = gt_msg->longitude_rad();
}

void MagnetometerPlugin::addNoise(Eigen::Vector3d* magnetic_field, const double dt) {
  assert(dt > 0.0);

  double tau = bias_correlation_time_;
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigma_d = 1 / sqrt(dt) * noise_density_;
  double sigma_b = random_walk_;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_d = sqrt( - sigma_b * sigma_b * tau / 2.0 * (exp(-2.0 * dt / tau) - 1.0));
  // Compute state-transition.
  double phi_d = exp(-1.0 / tau * dt);
  // Simulate magnetometer noise processes and add them to the true signal.
  for (int i = 0; i < 3; ++i) {
    bias_[i] = phi_d * bias_[i] + sigma_b_d * standard_normal_distribution_(random_generator_);
    (*magnetic_field)[i] = (*magnetic_field)[i] + bias_[i] + sigma_d * standard_normal_distribution_(random_generator_);
  }
}

void MagnetometerPlugin::OnUpdate(const common::UpdateInfo&)
{
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = world_->SimTime();
#else
  common::Time current_time = world_->GetSimTime();
#endif
  double dt = (current_time - last_pub_time_).Double();

  if (dt > 1.0 / pub_rate_||true) {

    // Magnetic field data from WMM2018 (10^5xnanoTesla (N, E D) n-frame )

    // Magnetic declination and inclination (radians)
    float declination_rad = get_mag_declination(groundtruth_lat_rad_ * 180 / M_PI, groundtruth_lon_rad_ * 180 / M_PI) * M_PI / 180;
    float inclination_rad = get_mag_inclination(groundtruth_lat_rad_ * 180 / M_PI, groundtruth_lon_rad_ * 180 / M_PI) * M_PI / 180;

    // Magnetic strength (10^5xnanoTesla)
    float strength_ga = 0.01f * get_mag_strength(groundtruth_lat_rad_ * 180 / M_PI, groundtruth_lon_rad_ * 180 / M_PI);

    // Magnetic filed components are calculated by http://geomag.nrcan.gc.ca/mag_fld/comp-en.php
    float H = strength_ga * cosf(inclination_rad);
    float Z = tanf(inclination_rad) * H;
    float X = H * cosf(declination_rad);
    float Y = H * sinf(declination_rad);

    // Magnetometer noise
    Eigen::Vector3d magnetic_field_I(X, Y, Z);
    addNoise(&magnetic_field_I, dt);

    // Fill magnetometer message
    //mag_message_.set_time_usec(current_time.Double() * 1e6);
    mag_message_.mutable_header()->set_frame_id("gt_link");
    mag_message_.mutable_header()->mutable_stamp()->set_sec(current_time.sec);
    mag_message_.mutable_header()->mutable_stamp()->set_nsec(current_time.nsec);

    //gazebo::msgs::Vector3d* magnetic_field = new gazebo::msgs::Vector3d();
    //magnetic_field->set_x(magnetic_field_I[0]);
    //magnetic_field->set_y(magnetic_field_I[1]);
    //magnetic_field->set_z(magnetic_field_I[2]);
    //mag_message_.set_allocated_magnetic_field(magnetic_field);

    mag_message_.mutable_magnetic_field()->set_x(magnetic_field_I[0]);
    mag_message_.mutable_magnetic_field()->set_y(magnetic_field_I[1]);
    mag_message_.mutable_magnetic_field()->set_z(magnetic_field_I[2]);

    last_pub_time_ = current_time;

    // publish mag msg
    pub_mag_->Publish(mag_message_);
    //std::cout<<"mag pub spam\n";
  }
}

}

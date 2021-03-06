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

#ifndef _GAZEBO_MAGNETOMETER_PLUGIN_HH_
#define _GAZEBO_MAGNETOMETER_PLUGIN_HH_

#include <random>
#include <string>

#include <Eigen/Core>

#include <MagneticField.pb.h>
#include <Groundtruth.pb.h>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <ignition/math.hh>

#include "common.h"
#include "geo_mag_declination_px4.h"

namespace gazebo {

static constexpr auto kDefaultMagnetometerTopic = "/mag";
static constexpr auto kDefaultPubRate = 100.0; // [Hz]. Note: corresponds to most of the mag devices supported in PX4

// Default values for use with ADIS16448 IMU
static constexpr auto kDefaultNoiseDensity = 0.4*1e-3; // [gauss / sqrt(hz)]
static constexpr auto kDefaultRandomWalk = 6.4*1e-6; // [gauss * sqrt(hz)]
static constexpr auto kDefaultBiasCorrelationTime = 6.0e+2; // [s]

typedef const boost::shared_ptr<const sensor_msgs::msgs::Groundtruth> GtPtr;

class MagnetometerPlugin : public ModelPlugin {
public:
  MagnetometerPlugin();
  virtual ~MagnetometerPlugin();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo&);
  void addNoise(Eigen::Vector3d* magnetic_field, const double dt);
  void GroundtruthCallback(GtPtr&);

private:
  std::string namespace_;
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  std::string mag_topic_;
  transport::NodePtr node_handle_;
  transport::PublisherPtr pub_mag_;
  transport::SubscriberPtr gt_sub_;
  std::string gt_sub_topic_;

  double groundtruth_lat_rad_;
  double groundtruth_lon_rad_;

  event::ConnectionPtr update_connection_;
  gz_sensor_msgs::MagneticField mag_message_;

  common::Time last_time_;
  common::Time last_pub_time_;
  unsigned int pub_rate_;
  double noise_density_;
  double random_walk_;
  double bias_correlation_time_;

  Eigen::Vector3d bias_;

  std::default_random_engine random_generator_;
  std::normal_distribution<double> standard_normal_distribution_;
}; // class MagnetometerPlugin
}  // namespace gazebo
#endif // _GAZEBO_MAGNETOMETER_PLUGIN_HH_

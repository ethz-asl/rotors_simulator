/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @brief Barometer Plugin
 *
 * This plugin simulates barometer data
 *
 * @author Elia Tarasov <elias.tarasov@gmail.com>
 */

#include <rotors_gazebo_plugins/gazebo_barometer_plugin_px4.h>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(BarometerPlugin)

BarometerPlugin::BarometerPlugin() : ModelPlugin(),
    baro_rnd_y2_(0.0),
    baro_rnd_use_last_(false),
    baro_drift_pa_(0.0)
{
}

BarometerPlugin::~BarometerPlugin()
{
  update_connection_->~Connection();
}

void BarometerPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  model_ = model;
  world_ = model_->GetWorld();

  namespace_.clear();
  if (sdf->HasElement("robotNamespace")) {
    namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_barometer_plugin] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init();

  const char *env_alt = std::getenv("PX4_HOME_ALT");
  if (env_alt) {
    alt_home_ = std::stod(env_alt);
    gzmsg << "[gazebo_barometer_plugin] Home altitude is set to " << alt_home_ << ".\n";
  } else {
    alt_home_ = kDefaultAltHome;
  }

  std::string link_name;
  if (sdf->HasElement("linkName")) {
      link_name = sdf->GetElement("linkName")->Get<std::string>();
      link_ = boost::dynamic_pointer_cast<physics::Link>(world_->EntityByName(link_name));
      if (link_ == NULL)
          gzerr << "[gazebo_barometer_plugin] Couldn't find specified link \"" << link_name << "\"\n";
  } else {
      gzwarn << "[gazebo_barometer_plugin] Using model frame of reference\n";
  }

  if (sdf->HasElement("pubRate")) {
    pub_rate_ = sdf->GetElement("pubRate")->Get<unsigned int>();
  } else {
    pub_rate_ = kDefaultPubRate;
    gzwarn << "[gazebo_barometer_plugin] Using default publication rate of " << pub_rate_ << " Hz\n";
  }

  if (sdf->HasElement("baroTopic")) {
    baro_topic_ = sdf->GetElement("baroTopic")->Get<std::string>();
  } else {
    baro_topic_ = kDefaultBarometerTopic;
    gzwarn << "[gazebo_barometer_plugin] Using default barometer topic " << baro_topic_ << "\n";
  }

  if (sdf->HasElement("baroDriftPaPerSec")) {
    baro_drift_pa_per_sec_ = sdf->GetElement("baroDriftPaPerSec")->Get<double>();
  } else {
    baro_drift_pa_per_sec_ = 0.0;
  }

#if GAZEBO_MAJOR_VERSION >= 9
  last_time_ = world_->SimTime();
  last_pub_time_ = world_->SimTime();
  if (!link_)
    pose_model_start_ = model_->WorldPose();
  else
    pose_model_start_ = link_->WorldPose();
#else
  last_time_ = world_->GetSimTime();
  last_pub_time_ = world_->GetSimTime();
  if (!link_)
    pose_model_start_ = ignitionFromGazeboMath(model_->GetWorldPose());
  else
    pose_model_start_ = ignitionFromGazeboMath(link_->GetWorldPose());
#endif

  // Listen to the update event. This event is broadcast every simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&BarometerPlugin::OnUpdate, this, _1));

  pub_baro_ = node_handle_->Advertise<sensor_msgs::msgs::Pressure>("~/" + namespace_ + baro_topic_, 10);
  std::cout<<"\33[1m[gazebo_barometer_plugin]\33[0m Advertising \33[1;32m" << pub_baro_->GetTopic() << "\33[0m gazebo message\n";

  standard_normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);
  gravity_W_ = world_->Gravity();
}

void BarometerPlugin::OnUpdate(const common::UpdateInfo&)
{
#if GAZEBO_MAJOR_VERSION >= 9
  const common::Time current_time = world_->SimTime();
#else
  const common::Time current_time = world_->GetSimTime();
#endif
  const double dt = (current_time - last_pub_time_).Double();

  if (dt > 1.0 / pub_rate_) {

    // get pose of the model that the plugin is attached to
    ignition::math::Pose3d pose_model_world;

    if (!link_) {
#if GAZEBO_MAJOR_VERSION >= 9
      pose_model_world = model_->WorldPose();
#else
      pose_model_world = ignitionFromGazeboMath(model_->GetWorldPose());
#endif
    } else {
#if GAZEBO_MAJOR_VERSION >= 9
      pose_model_world = link_->WorldPose();
#else
      pose_model_world = ignitionFromGazeboMath(link_->GetWorldPose());
#endif
    }

    ignition::math::Pose3d pose_model; // Z-component pose in local frame (relative to where it started)
    pose_model.Pos().Z() = pose_model_world.Pos().Z() - pose_model_start_.Pos().Z();

    const float pose_n_z = -pose_model.Pos().Z(); // convert Z-component from ENU to NED

    // calculate abs_pressure using an ISA model for the tropsphere (valid up to 11km above MSL)
    const float lapse_rate = 0.0065f; // reduction in temperature with altitude (Kelvin/m)
    const float temperature_msl = 288.0f; // temperature at MSL (Kelvin)
    const float alt_msl = (float)alt_home_ - pose_n_z;
    const float temperature_local = temperature_msl - lapse_rate * alt_msl;
    const float pressure_ratio = powf(temperature_msl / temperature_local, 5.256f);
    const float pressure_msl = 101325.0f; // pressure at MSL
    const float absolute_pressure = pressure_msl / pressure_ratio;
    //std::cout<<"alt_msl: "<<alt_msl<<"\n";

    // generate Gaussian noise sequence using polar form of Box-Muller transformation
    double y1;
    {
      double x1, x2, w;
      if (!baro_rnd_use_last_) {
        do {
          x1 = 2.0 * standard_normal_distribution_(random_generator_) - 1.0;
          x2 = 2.0 * standard_normal_distribution_(random_generator_) - 1.0;
          w = x1 * x1 + x2 * x2;
        } while ( w >= 1.0 );
        w = sqrt( (-2.0 * log( w ) ) / w );
        // calculate two values - the second value can be used next time because it is uncorrelated
        y1 = x1 * w;
        baro_rnd_y2_ = x2 * w;
        baro_rnd_use_last_ = true;
      } else {
        // no need to repeat the calculation - use the second value from last update
        y1 = baro_rnd_y2_;
        baro_rnd_use_last_ = false;
      }
    }

    // Apply noise and drift
    const float abs_pressure_noise = 1.0f * (float)y1;  // 1 Pa RMS noise
    baro_drift_pa_ += baro_drift_pa_per_sec_ * dt;
    const float absolute_pressure_noisy = absolute_pressure + abs_pressure_noise + baro_drift_pa_;

    // convert to hPa
    const float absolute_pressure_noisy_hpa = absolute_pressure_noisy * 0.01f;
    baro_msg_.set_absolute_pressure(absolute_pressure_noisy_hpa);

    // calculate density using an ISA model for the tropsphere (valid up to 11km above MSL)
    const float density_ratio = powf(temperature_msl / temperature_local, 4.256f);
    const float rho = 1.225f / density_ratio;

    // calculate pressure altitude including effect of pressure noise
    baro_msg_.set_pressure_altitude(alt_msl -
                                    (abs_pressure_noise + baro_drift_pa_) /
                                        (gravity_W_.Length() * rho));

    // calculate temperature in Celsius
    baro_msg_.set_temperature(temperature_local - 273.0f);

    // Fill baro msg
    baro_msg_.set_time_usec(current_time.Double() * 1e6);

    last_pub_time_ = current_time;

    // Publish baro msg
    pub_baro_->Publish(baro_msg_);
  }
}
}

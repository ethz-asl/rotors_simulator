/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @brief Airspeed Plugin
 *
 * This plugin publishes Airspeed
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 */

#include <rotors_gazebo_plugins/gazebo_airspeed_plugin_px4.h>

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(AirspeedPlugin)

AirspeedPlugin::AirspeedPlugin() : ModelPlugin(),
  diff_pressure_stddev_(0.01f),
  temperature_(20.0f)
{ }

AirspeedPlugin::~AirspeedPlugin()
{
    if (updateConnection_)
      updateConnection_->~Connection();
}

void AirspeedPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model.
  model_ = _model;
  world_ = model_->GetWorld();

  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_airspeed_plugin] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init();

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_airspeed_plugin] Please specify a linkName.\n";
  // Get the pointer to the link
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_airspeed_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  if(_sdf->HasElement("windTopic")) {
      wind_.wind_topic = _sdf->Get<std::string>("windTopic");
  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&AirspeedPlugin::OnUpdate, this, _1));

  if (!wind_.wind_topic.empty()) {
    wind_.wind_sub_ = node_handle_->Subscribe("~/" + namespace_ + wind_.wind_topic, &AirspeedPlugin::Wind::Callback, &wind_);
    std::cout<<"\33[1m[gazebo_airspeed_plugin]\33[0m Subscribing to \33[1;34m" << wind_.wind_sub_->GetTopic() << "\33[0m gazebo message\n";
  }

  airspeed_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Airspeed>("~/" + namespace_ + "/airspeed", 10);
  std::cout<<"\33[1m[gazebo_airspeed_plugin]\33[0m Advertising \33[1;32m" << airspeed_pub_->GetTopic() << "\33[0m gazebo message\n";

  //wind_sub_ = node_handle_->Subscribe("~/world_wind", &AirspeedPlugin::WindVelocityCallback, this);

  getSdfParam<float>(_sdf, "DiffPressureStdev", diff_pressure_stddev_, diff_pressure_stddev_);
  getSdfParam<float>(_sdf, "Temperature", temperature_, temperature_);
}

void AirspeedPlugin::OnUpdate(const common::UpdateInfo&){
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = world_->SimTime();
#else
  common::Time current_time = world_->GetSimTime();
#endif
  const float temperature_msl = 288.0f; // temperature at MSL (Kelvin)
  float temperature_local = temperature_ + 273.0f;
  const float density_ratio = powf((temperature_msl/temperature_local) , 4.256f);
  float rho = 1.225f / density_ratio;

  const float diff_pressure_noise = standard_normal_distribution_(random_generator_) * diff_pressure_stddev_;

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d T_W_I = link_->WorldPose();
#else
  ignition::math::Pose3d T_W_I = ignitionFromGazeboMath(link_->GetWorldPose());
#endif
  ignition::math::Quaterniond C_W_I = T_W_I.Rot();

  ignition::math::Vector3d cp = T_W_I.Pos(); // airspeed sensor position in world frame

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d vel_a = link_->RelativeLinearVel() - C_W_I.RotateVectorReverse(wind_.GetWind(cp));
#else
  ignition::math::Vector3d vel_a = ignitionFromGazeboMath(link_->GetRelativeLinearVel()) - C_W_I.RotateVectorReverse(wind_.GetWind(cp));
#endif
  double diff_pressure = 0.005f * rho * vel_a.X() * vel_a.X() + diff_pressure_noise;

  // calculate differential pressure in hPa
  sensor_msgs::msgs::Airspeed airspeed_msg;
  airspeed_msg.set_time_usec(current_time.Double() * 1e6);
  airspeed_msg.set_diff_pressure(diff_pressure);
  airspeed_pub_->Publish(airspeed_msg);

  last_time_ = current_time;
}

} // namespace gazebo

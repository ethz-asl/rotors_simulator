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
 * @brief Groundtruth Plugin
 *
 * This plugin gets and publishes ground-truth data
 *
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <rotors_gazebo_plugins/gazebo_groundtruth_plugin_px4.h>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GroundtruthPlugin)

GroundtruthPlugin::GroundtruthPlugin() : ModelPlugin()
{ }

GroundtruthPlugin::~GroundtruthPlugin()
{
  if (updateConnection_)
    updateConnection_->~Connection();
  world_->Reset();
}

void GroundtruthPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;
  world_ = model_->GetWorld();

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_groundtruth_plugin] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init();

  // Use environment variables if set for home position.
  const char *env_lat = std::getenv("PX4_HOME_LAT");
  const char *env_lon = std::getenv("PX4_HOME_LON");
  const char *env_alt = std::getenv("PX4_HOME_ALT");

  // Check if the world spherical coordinates are set and use them in that case
  const bool world_has_origin = checkWorldHomePosition(world_, world_latitude_, world_longitude_, world_altitude_);

  if (env_lat) {
    lat_home_ = std::stod(env_lat) * M_PI / 180.0;
    gzmsg << "[gazebo_groundtruth_plugin] Home latitude is set to " << std::stod(env_lat) << ".\n";
  } else if (world_has_origin) {
    lat_home_ = world_latitude_;
    gzmsg << "[gazebo_groundtruth_plugin] Home latitude is set to " << lat_home_ << ".\n";
  } else if(_sdf->HasElement("homeLatitude")) {
    double latitude;
    getSdfParam<double>(_sdf, "homeLatitude", latitude, lat_home_);
    lat_home_ = latitude * M_PI / 180.0;
  }

  if (env_lon) {
    lon_home_ = std::stod(env_lon) * M_PI / 180.0;
    gzmsg << "[gazebo_groundtruth_plugin] Home longitude is set to " << std::stod(env_lon) << ".\n";
  } else if (world_has_origin) {
    lon_home_ = world_longitude_;
    gzmsg << "[gazebo_groundtruth_plugin] Home longitude is set to " << lon_home_ << ".\n";
  } else if(_sdf->HasElement("homeLongitude")) {
    double longitude;
    getSdfParam<double>(_sdf, "homeLongitude", longitude, lon_home_);
    lon_home_ = longitude * M_PI / 180.0;
  }

  if (env_alt) {
    alt_home_ = std::stod(env_alt);
    gzmsg << "[gazebo_groundtruth_plugin] Home altitude is set to " << alt_home_ << ".\n";
  } else if (world_has_origin) {
    alt_home_ = world_altitude_;
    gzmsg << "[gazebo_groundtruth_plugin] Home altitude is set to " << alt_home_ << ".\n";
  } else if(_sdf->HasElement("homeAltitude")) {
    getSdfParam<double>(_sdf, "homeAltitude", alt_home_, alt_home_);
  }

  std::string link_name;
  if (_sdf->HasElement("linkName")) {
      link_name = _sdf->GetElement("linkName")->Get<std::string>();
      link_ = boost::dynamic_pointer_cast<physics::Link>(world_->EntityByName(link_name));
      if (link_ == NULL)
          gzerr << "[gazebo_groundtruth_plugin] Couldn't find specified link \"" << link_name << "\"\n";
  } else {
      gzwarn << "[gazebo_groundtruth_plugin] Using model frame of reference\n";
  }

  // Listen to the update event. This event is broadcast every simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GroundtruthPlugin::OnUpdate, this, _1));

  gt_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Groundtruth>("~/" + namespace_ + "/groundtruth", 10);
  std::cout<<"\33[1m[gazebo_groundtruth_plugin]\33[0m Advertising \33[1;32m" << gt_pub_->GetTopic() << "\33[0m gazebo message\n";
}

void GroundtruthPlugin::OnUpdate(const common::UpdateInfo&)
{
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = world_->SimTime();
#else
  common::Time current_time = world_->GetSimTime();
#endif

  ignition::math::Pose3d T_W_I;
  ignition::math::Vector3d velocity_current_W;

  if (!link_) {
#if GAZEBO_MAJOR_VERSION >= 9
    T_W_I = model_->WorldPose();
    velocity_current_W = model_->WorldLinearVel();
#else
    T_W_I = ignitionFromGazeboMath(model_->GetWorldPose());
    velocity_current_W = ignitionFromGazeboMath(model_->GetWorldLinearVel());
#endif
  } else {
#if GAZEBO_MAJOR_VERSION >= 9
    T_W_I = link_->WorldPose();
    velocity_current_W = link_->WorldLinearVel();
#else
    T_W_I = ignitionFromGazeboMath(link_->GetWorldPose());
    velocity_current_W = ignitionFromGazeboMath(link_->GetWorldLinearVel());
#endif
  }

  // Use the models world position and attitude for groundtruth
  ignition::math::Vector3d& pos_W_I = T_W_I.Pos();
  ignition::math::Quaterniond& att_W_I = T_W_I.Rot();

  // reproject position into geographic coordinates
  auto latlon_gt = reproject(pos_W_I, lat_home_, lon_home_, alt_home_);

  ignition::math::Vector3d velocity_current_W_xy = velocity_current_W;
  velocity_current_W_xy.Z() = 0;

  // fill Groundtruth msg
  sensor_msgs::msgs::Groundtruth groundtruth_msg;

  groundtruth_msg.set_time_usec(current_time.Double() * 1e6);
  groundtruth_msg.set_latitude_rad(latlon_gt.first);
  groundtruth_msg.set_longitude_rad(latlon_gt.second);
  groundtruth_msg.set_altitude(pos_W_I.Z() + alt_home_);
  groundtruth_msg.set_velocity_east(velocity_current_W.X());
  groundtruth_msg.set_velocity_north(velocity_current_W.Y());
  groundtruth_msg.set_velocity_up(velocity_current_W.Z());
  groundtruth_msg.set_attitude_q_w(att_W_I.W());
  groundtruth_msg.set_attitude_q_x(att_W_I.X());
  groundtruth_msg.set_attitude_q_y(att_W_I.Y());
  groundtruth_msg.set_attitude_q_z(att_W_I.Z());

  // publish Groundtruth msg at full sim rate
  gt_pub_->Publish(groundtruth_msg);
}

} // namespace gazebo

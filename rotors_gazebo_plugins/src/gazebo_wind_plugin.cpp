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

#include "rotors_gazebo_plugins/gazebo_wind_plugin.h"

#include "ConnectGazeboToRosTopic.pb.h"

namespace gazebo {

GazeboWindPlugin::~GazeboWindPlugin() {
  event::Events::DisconnectWorldUpdateBegin(update_connection_);
}

void GazeboWindPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  // Store the pointer to the model.
  model_ = _model;
  world_ = model_->GetWorld();

  double wind_gust_start = kDefaultWindGustStart;
  double wind_gust_duration = kDefaultWindGustDuration;

  //==============================================//
  //========== READ IN PARAMS FROM SDF ===========//
  //==============================================//

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_wind_plugin] Please specify a robotNamespace.\n";

  // Create Gazebo Node.
  node_handle_ = gazebo::transport::NodePtr(new transport::Node());

  // Initialise with default namespace (typically /gazebo/default/).
  node_handle_->Init();

  if (_sdf->HasElement("xyzOffset"))
    xyz_offset_ = _sdf->GetElement("xyzOffset")->Get<math::Vector3>();
  else
    gzerr << "[gazebo_wind_plugin] Please specify a xyzOffset.\n";

  getSdfParam<std::string>(_sdf, "windForcePubTopic", wind_force_pub_topic_,
                           wind_force_pub_topic_);
  getSdfParam<std::string>(_sdf, "windSpeedPubTopic", wind_speed_pub_topic_,
                           wind_speed_pub_topic_);
  getSdfParam<std::string>(_sdf, "frameId", frame_id_, frame_id_);
  getSdfParam<std::string>(_sdf, "linkName", link_name_, link_name_);
  // Get the wind params from SDF.
  getSdfParam<double>(_sdf, "windForceMean", wind_force_mean_,
                      wind_force_mean_);
  getSdfParam<double>(_sdf, "windForceVariance", wind_force_variance_,
                      wind_force_variance_);
  getSdfParam<math::Vector3>(_sdf, "windDirection", wind_direction_,
                             wind_direction_);
  // Get the wind gust params from SDF.
  getSdfParam<double>(_sdf, "windGustStart", wind_gust_start, wind_gust_start);
  getSdfParam<double>(_sdf, "windGustDuration", wind_gust_duration,
                      wind_gust_duration);
  getSdfParam<double>(_sdf, "windGustForceMean", wind_gust_force_mean_,
                      wind_gust_force_mean_);
  getSdfParam<double>(_sdf, "windGustForceVariance", wind_gust_force_variance_,
                      wind_gust_force_variance_);
  getSdfParam<math::Vector3>(_sdf, "windGustDirection", wind_gust_direction_,
                             wind_gust_direction_);
  // Get the wind speed params from SDF.
  getSdfParam<double>(_sdf, "windSpeedMean", wind_speed_mean_,
                      wind_speed_mean_);
  getSdfParam<double>(_sdf, "windSpeedVariance", wind_speed_variance_,
                      wind_speed_variance_);

  wind_direction_.Normalize();
  wind_gust_direction_.Normalize();
  wind_gust_start_ = common::Time(wind_gust_start);
  wind_gust_end_ = common::Time(wind_gust_start + wind_gust_duration);

  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_wind_plugin] Couldn't find specified link \"" << link_name_
                                                                   << "\".");

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboWindPlugin::OnUpdate, this, _1));
}

// This gets called by the world update start event.
void GazeboWindPlugin::OnUpdate(const common::UpdateInfo& _info) {
  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  // Get the current simulation time.
  common::Time now = world_->GetSimTime();

  // Calculate the wind force.
  double wind_strength = wind_force_mean_;
  math::Vector3 wind = wind_strength * wind_direction_;
  // Apply a force from the constant wind to the link.
  link_->AddForceAtRelativePosition(wind, xyz_offset_);

  math::Vector3 wind_gust(0, 0, 0);
  // Calculate the wind gust force.
  if (now >= wind_gust_start_ && now < wind_gust_end_) {
    double wind_gust_strength = wind_gust_force_mean_;
    wind_gust = wind_gust_strength * wind_gust_direction_;
    // Apply a force from the wind gust to the link.
    link_->AddForceAtRelativePosition(wind_gust, xyz_offset_);
  }

  wrench_stamped_msg_.mutable_header()->set_frame_id(frame_id_);
  wrench_stamped_msg_.mutable_header()->mutable_stamp()->set_sec(now.sec);
  wrench_stamped_msg_.mutable_header()->mutable_stamp()->set_nsec(now.nsec);

  wrench_stamped_msg_.mutable_wrench()->mutable_force()->set_x(wind.x +
                                                               wind_gust.x);
  wrench_stamped_msg_.mutable_wrench()->mutable_force()->set_y(wind.y +
                                                               wind_gust.y);
  wrench_stamped_msg_.mutable_wrench()->mutable_force()->set_z(wind.z +
                                                               wind_gust.z);

  // No torque due to wind, set x,y and z to 0.
  wrench_stamped_msg_.mutable_wrench()->mutable_torque()->set_x(0);
  wrench_stamped_msg_.mutable_wrench()->mutable_torque()->set_y(0);
  wrench_stamped_msg_.mutable_wrench()->mutable_torque()->set_z(0);

  wind_force_pub_->Publish(wrench_stamped_msg_);

  // Calculate the wind speed.
  double wind_speed = wind_speed_mean_;
  math::Vector3 wind_velocity = wind_speed * wind_direction_;

  wind_speed_msg_.mutable_header()->set_frame_id(frame_id_);
  wind_speed_msg_.mutable_header()->mutable_stamp()->set_sec(now.sec);
  wind_speed_msg_.mutable_header()->mutable_stamp()->set_nsec(now.nsec);

  wind_speed_msg_.mutable_velocity()->set_x(wind_velocity.x);
  wind_speed_msg_.mutable_velocity()->set_y(wind_velocity.y);
  wind_speed_msg_.mutable_velocity()->set_z(wind_velocity.z);

  wind_speed_pub_->Publish(wind_speed_msg_);
}

void GazeboWindPlugin::CreatePubsAndSubs() {
  // Create temporary "ConnectGazeboToRosTopic" publisher and message.
  gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
          "~/" + kConnectGazeboToRosSubtopic, 1);

  gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;

  // ============================================ //
  // ========= WRENCH STAMPED MSG SETUP ========= //
  // ============================================ //
  wind_force_pub_ = node_handle_->Advertise<gz_geometry_msgs::WrenchStamped>(
      "~/" + namespace_ + "/" + wind_force_pub_topic_, 1);

  // connect_gazebo_to_ros_topic_msg.set_gazebo_namespace(namespace_);
  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   wind_force_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                wind_force_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::WRENCH_STAMPED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);

  // ============================================ //
  // ========== WIND SPEED MSG SETUP ============ //
  // ============================================ //
  wind_speed_pub_ = node_handle_->Advertise<gz_mav_msgs::WindSpeed>(
      "~/" + namespace_ + "/" + wind_speed_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   wind_speed_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                wind_speed_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::WIND_SPEED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboWindPlugin);

}  // namespace gazebo

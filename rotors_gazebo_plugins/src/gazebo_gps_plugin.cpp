/*
 * Copyright 2016 Pavel Vechersky, ASL, ETH Zurich, Switzerland
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

// MODULE
#include "rotors_gazebo_plugins/gazebo_gps_plugin.h"

// 3RD PARTY
#include "mav_msgs/default_topics.h"

// USER
#include "ConnectGazeboToRosTopic.pb.h"

namespace gazebo {

GazeboGpsPlugin::GazeboGpsPlugin()
    : SensorPlugin(),
      // node_handle_(0),
      random_generator_(random_device_()),
      pubs_and_subs_created_(false) {}

GazeboGpsPlugin::~GazeboGpsPlugin() {
}

void GazeboGpsPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

// Store the pointer to the parent sensor.
  parent_sensor_ = std::dynamic_pointer_cast<sensors::GpsSensor>(_sensor);
  world_ = physics::get_world(parent_sensor_->WorldName());

  //==============================================//
  //========== READ IN PARAMS FROM SDF ===========//
  //==============================================//

  std::string link_name;

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_gps_plugin] Please specify a robotNamespace.\n";

  node_handle_ = gazebo::transport::NodePtr(new transport::Node());

  // Initialise with default namespace (typically /gazebo/default/)
  node_handle_->Init();

  if (_sdf->HasElement("linkName"))
    link_name = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_gps_plugin] Please specify a linkName.\n";

  std::string frame_id = link_name;

  // Get the pointer to the link that holds the sensor.
  link_ =
      boost::dynamic_pointer_cast<physics::Link>(world_->EntityByName(link_name));
  if (link_ == NULL)
    gzerr << "[gazebo_gps_plugin] Couldn't find specified link \"" << link_name
          << "\"\n";

  // Retrieve the rest of the SDF parameters.
  double hor_pos_std_dev;
  double ver_pos_std_dev;
  double hor_vel_std_dev;
  double ver_vel_std_dev;

  getSdfParam<std::string>(_sdf, "gpsTopic", gps_topic_,
                           mav_msgs::default_topics::GPS);

  getSdfParam<std::string>(_sdf, "groundSpeedTopic", ground_speed_topic_,
                           mav_msgs::default_topics::GROUND_SPEED);

  getSdfParam<double>(_sdf, "horPosStdDev", hor_pos_std_dev,
                      kDefaultHorPosStdDev);
  getSdfParam<double>(_sdf, "verPosStdDev", ver_pos_std_dev,
                      kDefaultVerPosStdDev);
  getSdfParam<double>(_sdf, "horVelStdDev", hor_vel_std_dev,
                      kDefaultHorVelStdDev);
  getSdfParam<double>(_sdf, "verVelStdDev", ver_vel_std_dev,
                      kDefaultVerVelStdDev);

  // Connect to the sensor update event.
  this->updateConnection_ = this->parent_sensor_->ConnectUpdated(
      boost::bind(&GazeboGpsPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  parent_sensor_->SetActive(true);

  // Initialize the normal distributions for ground speed.
  ground_speed_n_[0] = NormalDistribution(0, hor_vel_std_dev);
  ground_speed_n_[1] = NormalDistribution(0, hor_vel_std_dev);
  ground_speed_n_[2] = NormalDistribution(0, ver_vel_std_dev);

  // ============================================ //
  // ======= POPULATE STATIC PARTS OF MSGS ====== //
  // ============================================ //

  // Fill the static parts of the GPS message.
  gz_gps_message_.mutable_header()->set_frame_id(frame_id);
  gz_gps_message_.set_service(gz_sensor_msgs::NavSatFix::SERVICE_GPS);
  gz_gps_message_.set_status(gz_sensor_msgs::NavSatFix::STATUS_FIX);
  gz_gps_message_.set_position_covariance_type(
      gz_sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN);

  for (int i = 0; i < 9; i++) {
    switch (i) {
      case 0:
        gz_gps_message_.add_position_covariance(hor_pos_std_dev *
                                                hor_pos_std_dev);
        break;
      case 1:
      case 2:
      case 3:
        gz_gps_message_.add_position_covariance(0);
        break;
      case 4:
        gz_gps_message_.add_position_covariance(hor_pos_std_dev *
                                                hor_pos_std_dev);
        break;
      case 5:
      case 6:
      case 7:
        gz_gps_message_.add_position_covariance(0);
        break;
      case 8:
        gz_gps_message_.add_position_covariance(ver_pos_std_dev *
                                                ver_pos_std_dev);
        break;
    }
  }

  // Fill the static parts of the ground speed message.
  gz_ground_speed_message_.mutable_header()->set_frame_id(frame_id);
  gz_ground_speed_message_.mutable_twist()->mutable_angular()->set_x(0.0);
  gz_ground_speed_message_.mutable_twist()->mutable_angular()->set_y(0.0);
  gz_ground_speed_message_.mutable_twist()->mutable_angular()->set_z(0.0);
}

void GazeboGpsPlugin::OnUpdate() {
  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  // Get the time of the last measurement.
  common::Time current_time;

  // Get the linear velocity in the world frame.
  ignition::math::Vector3d W_ground_speed_W_L = link_->WorldLinearVel();

  // Apply noise to ground speed.
  W_ground_speed_W_L += ignition::math::Vector3d (ground_speed_n_[0](random_generator_),
                                      ground_speed_n_[1](random_generator_),
                                      ground_speed_n_[2](random_generator_));

  // Fill the GPS message.
  current_time = parent_sensor_->LastMeasurementTime();

  gz_gps_message_.set_latitude(parent_sensor_->Latitude().Degree());
  gz_gps_message_.set_longitude(parent_sensor_->Longitude().Degree());
  gz_gps_message_.set_altitude(parent_sensor_->Altitude());

  gz_gps_message_.mutable_header()->mutable_stamp()->set_sec(current_time.sec);
  gz_gps_message_.mutable_header()->mutable_stamp()->set_nsec(
      current_time.nsec);

  // Fill the ground speed message.
  gz_ground_speed_message_.mutable_twist()->mutable_linear()->set_x(
      W_ground_speed_W_L.X());
  gz_ground_speed_message_.mutable_twist()->mutable_linear()->set_y(
      W_ground_speed_W_L.Y());
  gz_ground_speed_message_.mutable_twist()->mutable_linear()->set_z(
      W_ground_speed_W_L.Z());
  gz_ground_speed_message_.mutable_header()->mutable_stamp()->set_sec(
      current_time.sec);
  gz_ground_speed_message_.mutable_header()->mutable_stamp()->set_nsec(
      current_time.nsec);

  // Publish the GPS message.
  gz_gps_pub_->Publish(gz_gps_message_);

  // Publish the ground speed message.
  gz_ground_speed_pub_->Publish(gz_ground_speed_message_);
}

void GazeboGpsPlugin::CreatePubsAndSubs() {
  // Create temporary "ConnectGazeboToRosTopic" publisher and message
  gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
          "~/" + kConnectGazeboToRosSubtopic, 1);

  gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;

  // ============================================ //
  // =========== NAV SAT FIX MSG SETUP ========== //
  // ============================================ //
  gz_gps_pub_ = node_handle_->Advertise<gz_sensor_msgs::NavSatFix>(
      "~/" + namespace_ + "/" + gps_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   gps_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" + gps_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::NAV_SAT_FIX);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);

  // ============================================ //
  // == GROUND SPEED (TWIST STAMPED) MSG SETUP == //
  // ============================================ //
  gz_ground_speed_pub_ =
      node_handle_->Advertise<gz_geometry_msgs::TwistStamped>(
          "~/" + namespace_ + "/" + ground_speed_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   ground_speed_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                ground_speed_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::TWIST_STAMPED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg);
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboGpsPlugin);
}

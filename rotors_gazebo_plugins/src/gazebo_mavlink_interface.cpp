/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#include "rotors_gazebo_plugins/gazebo_mavlink_interface.h"

namespace gazebo {

GazeboMavlinkInterface::GazeboMavlinkInterface()
    : ModelPlugin(),
      node_handle_(0) {}

GazeboMavlinkInterface::~GazeboMavlinkInterface() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboMavlinkInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model.
  model_ = _model;
  world_ = model_->GetWorld();

  // Default params
  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_mavlink_interface] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  double gps_freq;
  double ref_mag_north;
  double ref_mag_east;
  double ref_mag_down;

  getSdfParam<std::string>(_sdf, "imuSubTopic", imu_sub_topic_,
                           mav_msgs::default_topics::IMU);
  getSdfParam<std::string>(_sdf, "mavlinkHilSensorPubTopic",
                           hil_sensor_mavlink_pub_topic_,
                           kDefaultMavlinkHilSensorPubTopic);
  getSdfParam<double>(_sdf, "gpsUpdateFreq", gps_freq, kDefaultGpsUpdateFreq);
  getSdfParam<double>(_sdf, "referenceMagNorth", ref_mag_north, kDefaultRefMagNorth);
  getSdfParam<double>(_sdf, "referenceMagEast", ref_mag_east, kDefaultRefMagEast);
  getSdfParam<double>(_sdf, "referenceMagDown", ref_mag_down, kDefaultRefMagDown);
  getSdfParam<double>(_sdf, "referenceLatitude", ref_lat_, kDefaultRefLat);
  getSdfParam<double>(_sdf, "referenceLongitude", ref_lon_, kDefaultRefLon);
  getSdfParam<double>(_sdf, "referenceAltitude", ref_alt_, kDefaultRefAlt);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboMavlinkInterface::OnUpdate, this, _1));

  imu_sub_ = node_handle_->subscribe(imu_sub_topic_, 10, &GazeboMavlinkInterface::ImuCallback, this);

  hil_sensor_pub_ = node_handle_->advertise<mavros_msgs::Mavlink>(hil_sensor_mavlink_pub_topic_, 10);

  last_time_ = world_->GetSimTime();
  last_gps_time_ = world_->GetSimTime();
  gps_update_interval_ = 1000000000.0 / gps_freq;  // Nanoseconds

  gravity_W_ = world_->GetPhysicsEngine()->GetGravity();
  mag_W_ = math::Vector3(ref_mag_north, ref_mag_east, ref_mag_down);
}

// This gets called by the world update start event.
void GazeboMavlinkInterface::OnUpdate(const common::UpdateInfo& _info) {
  common::Time current_time = world_->GetSimTime();

  last_time_ = current_time;

  math::Pose T_W_I = model_->GetWorldPose(); //TODO(burrimi): Check tf.
  math::Vector3 pos_W_I = T_W_I.pos;  // Use the models' world position for GPS and pressure alt.

  math::Vector3 velocity_current_W = model_->GetWorldLinearVel();  // Use the models' world position for GPS velocity.
  math::Vector3 velocity_current_W_xy = velocity_current_W;
  velocity_current_W_xy.z = 0.0;
  
  common::Time gps_update(gps_update_interval_);

  if (current_time - last_gps_time_ > gps_update) {
    mavlink_message_t gps_mmsg;

    hil_gps_msg_.time_usec = current_time.nsec * 1000;
    hil_gps_msg_.fix_type = 3;
    hil_gps_msg_.lat = (ref_lat_ + (pos_W_I.x / kEarthRadius) * 180 / M_PI) * 10000000;
    hil_gps_msg_.lon = (ref_lon_ + (-pos_W_I.y / kEarthRadius) * 180 / M_PI) * 10000000;
    hil_gps_msg_.alt = (ref_alt_ + pos_W_I.z) * 1000;
    hil_gps_msg_.eph = 100;
    hil_gps_msg_.epv = 100;
    hil_gps_msg_.vel = velocity_current_W_xy.GetLength() * 100;
    hil_gps_msg_.vn = velocity_current_W.x * 100;
    hil_gps_msg_.ve = -velocity_current_W.y * 100;
    hil_gps_msg_.vd = -velocity_current_W.z * 100;
    hil_gps_msg_.cog = atan2(hil_gps_msg_.ve, hil_gps_msg_.vn) * 180.0 / M_PI * 100.0;
    hil_gps_msg_.satellites_visible = 10;
           
    mavlink_hil_gps_t* hil_gps_msg = &hil_gps_msg_;
    mavlink_msg_hil_gps_encode(1, 240, &gps_mmsg, hil_gps_msg);
    mavlink_message_t* gps_msg = &gps_mmsg;
    mavros_msgs::MavlinkPtr gps_rmsg = boost::make_shared<mavros_msgs::Mavlink>();
    gps_rmsg->header.stamp = ros::Time::now();
    mavros_msgs::mavlink::convert(*gps_msg, *gps_rmsg);
    
    hil_sensor_pub_.publish(gps_rmsg);

    last_gps_time_ = current_time;
  }
}

void GazeboMavlinkInterface::ImuCallback(const sensor_msgs::ImuConstPtr& imu_message) {
  mavlink_message_t mmsg;

  math::Pose T_W_I = model_->GetWorldPose();
  math::Vector3 pos_W_I = T_W_I.pos;  // Use the models'world position for GPS and pressure alt.

  math::Quaternion C_W_I;
  C_W_I.w = imu_message->orientation.w;
  C_W_I.x = imu_message->orientation.x;
  C_W_I.y = imu_message->orientation.y;
  C_W_I.z = imu_message->orientation.z;

  math::Vector3 mag_I = C_W_I.RotateVectorReverse(mag_W_); // TODO: Add noise based on bais and variance like for imu and gyro

  hil_sensor_msg_.time_usec = imu_message->header.stamp.nsec*1000;
  hil_sensor_msg_.xacc = imu_message->linear_acceleration.x;
  hil_sensor_msg_.yacc = imu_message->linear_acceleration.y;
  hil_sensor_msg_.zacc = imu_message->linear_acceleration.z;
  hil_sensor_msg_.xgyro = imu_message->angular_velocity.x;
  hil_sensor_msg_.ygyro = imu_message->angular_velocity.y;
  hil_sensor_msg_.zgyro = imu_message->angular_velocity.z;
  hil_sensor_msg_.xmag = mag_I.x;
  hil_sensor_msg_.ymag = mag_I.y;
  hil_sensor_msg_.zmag = mag_I.z;
  hil_sensor_msg_.abs_pressure = 0.0;
  hil_sensor_msg_.diff_pressure = 0.0;
  hil_sensor_msg_.pressure_alt = pos_W_I.z;
  hil_sensor_msg_.temperature = 0.0;
  hil_sensor_msg_.fields_updated = 4095;  // 0b1111111111111 (All updated since new data with new noise added always)

  mavlink_hil_sensor_t* hil_msg = &hil_sensor_msg_;
  mavlink_msg_hil_sensor_encode(1, 240, &mmsg, hil_msg);
  mavlink_message_t* msg = &mmsg;
  
  mavros_msgs::MavlinkPtr rmsg = boost::make_shared<mavros_msgs::Mavlink>();
  rmsg->header.stamp = ros::Time::now();
  mavros_msgs::mavlink::convert(*msg, *rmsg);

  hil_sensor_pub_.publish(rmsg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMavlinkInterface);
}

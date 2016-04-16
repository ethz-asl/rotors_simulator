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

  std::string mavlink_control_sub_topic;
  std::string imu_sub_topic;
  std::string hil_sensor_mavlink_pub_topic;
  std::string motor_speeds_pub_topic;
  double gps_freq;
  double ref_mag_north;
  double ref_mag_east;
  double ref_mag_down;

  getSdfParam<std::string>(_sdf, "mavlinkControlSubTopic", mavlink_control_sub_topic,
                           kDefaultMavlinkControlSubTopic);
  getSdfParam<std::string>(_sdf, "imuSubTopic", imu_sub_topic,
                           mav_msgs::default_topics::IMU);
  getSdfParam<std::string>(_sdf, "mavlinkHilSensorPubTopic", hil_sensor_mavlink_pub_topic,
                           kDefaultMavlinkHilSensorPubTopic);
  getSdfParam<std::string>(_sdf, "motorSpeedsPubTopic", motor_speeds_pub_topic,
                           kDefaultMotorSpeedsPubTopic);
  getSdfParam<double>(_sdf, "gpsUpdateFreq", gps_freq, kDefaultGpsUpdateFreq);
  getSdfParam<int>(_sdf, "rotorCount", rotor_count_, kDefaultRotorCount);
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

  mav_control_sub_ = node_handle_->subscribe(mavlink_control_sub_topic, 10,
                                             &GazeboMavlinkInterface::MavlinkControlCallback,
                                             this);

  imu_sub_ = node_handle_->subscribe(imu_sub_topic, 10, &GazeboMavlinkInterface::ImuCallback, this);

  hil_sensor_pub_ = node_handle_->advertise<mavros_msgs::Mavlink>(hil_sensor_mavlink_pub_topic, 10);

  motor_speeds_pub_ = node_handle_->advertise<mav_msgs::Actuators>(motor_speeds_pub_topic, 10);

  last_time_ = world_->GetSimTime();
  last_gps_time_ = world_->GetSimTime();
  gps_update_interval_ = 1000000000.0 / gps_freq;  // GPS update interval, in nanoseconds

  gravity_W_ = world_->GetPhysicsEngine()->GetGravity();
  mag_W_ = math::Vector3(ref_mag_north, ref_mag_east, ref_mag_down);
}

// This gets called by the world update start event.
void GazeboMavlinkInterface::OnUpdate(const common::UpdateInfo& _info) {
  common::Time current_time = world_->GetSimTime();

  last_time_ = current_time;

  // Use the models' world position for GPS, velocity, and pressure alt.
  math::Pose T_W_I = model_->GetWorldPose();
  math::Vector3 pos_W_I = T_W_I.pos;
  math::Vector3 velocity_current_W = model_->GetWorldLinearVel();
  math::Vector3 velocity_current_W_xy = velocity_current_W;
  velocity_current_W_xy.z = 0.0;
  
  common::Time gps_update(gps_update_interval_);

  if (current_time - last_gps_time_ > gps_update) {
    mavlink_message_t gps_mmsg;

    /* mavlink_hil_gps_t message specifications for clarifying the conversions:
      uint64_t time_usec < Timestamp (microseconds since UNIX epoch or microseconds since system boot)
      int32_t lat < Latitude (WGS84), in degrees * 1E7
      int32_t lon < Longitude (WGS84), in degrees * 1E7
      int32_t alt < Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
      uint16_t eph < GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
      uint16_t epv < GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
      uint16_t vel < GPS ground speed (m/s * 100). If unknown, set to: 65535
      int16_t vn < GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
      int16_t ve < GPS velocity in cm/s in EAST direction in earth-fixed NED frame
      int16_t vd < GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
      uint16_t cog < Course over ground (NOT heading, but direction of movement) in degrees * 100. If unknown, set to: 65535
      uint8_t fix_type < 0-1: no fix, 2: 2D fix, 3: 3D fix
      uint8_t satellites_visible < Number of satellites visible. If unknown, set to 255
    */

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

void GazeboMavlinkInterface::MavlinkControlCallback(const mavros_msgs::Mavlink::ConstPtr &rmsg) {
  if (rmsg->msgid == MAVLINK_MSG_ID_HIL_CONTROLS) {
    mavlink_message_t* mmsg(new mavlink_message_t);

    if (mavros_msgs::mavlink::convert(*rmsg, *mmsg)) {
      mavlink_hil_controls_t act_msg;
      mavlink_msg_hil_controls_decode(mmsg, &act_msg);

      float inputs[8];
      inputs[0] = act_msg.roll_ailerons;
      inputs[1] = act_msg.pitch_elevator;
      inputs[2] = act_msg.yaw_rudder;
      inputs[3] = act_msg.throttle;
      inputs[4] = act_msg.aux1;
      inputs[5] = act_msg.aux2;
      inputs[6] = act_msg.aux3;
      inputs[7] = act_msg.aux4;

      mav_msgs::Actuators motor_speeds_msg;
      ros::Time current_time = ros::Time::now();

      for (int i = 0; i < rotor_count_; i++) {
        motor_speeds_msg.angular_velocities.push_back(inputs[i] * kMotorSpeedScaling + kMotorSpeedOffset);
      }

      motor_speeds_msg.header.stamp.sec = current_time.sec;
      motor_speeds_msg.header.stamp.nsec = current_time.nsec;

      motor_speeds_pub_.publish(motor_speeds_msg);
    }
    else {
      gzerr << "[gazebo_mavlink_interface] Incorrect mavlink data.\n";
    }

    delete mmsg;
  }
}

void GazeboMavlinkInterface::ImuCallback(const sensor_msgs::ImuConstPtr& imu_message) {
  mavlink_message_t mmsg;

  // Use the models'world position for pressure alt.
  math::Pose T_W_I = model_->GetWorldPose();
  math::Vector3 pos_W_I = T_W_I.pos;

  math::Quaternion C_W_I;
  C_W_I.w = imu_message->orientation.w;
  C_W_I.x = imu_message->orientation.x;
  C_W_I.y = imu_message->orientation.y;
  C_W_I.z = imu_message->orientation.z;

  math::Vector3 mag_I = C_W_I.RotateVectorReverse(mag_W_);

  hil_sensor_msg_.time_usec = imu_message->header.stamp.nsec * 1000;
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
  hil_sensor_msg_.fields_updated = kAllFieldsUpdated;

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

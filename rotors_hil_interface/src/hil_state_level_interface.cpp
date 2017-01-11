/*
 * Copyright 2016 Pavel Vechersky, ASL, ETH Zurich, Switzerland
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

#include "rotors_hil_interface/hil_interface.h"

namespace rotors_hil {

HilStateLevelInterface::HilStateLevelInterface(const Eigen::Quaterniond& q_S_B) {
  ros::NodeHandle pnh("~");

  // Retrieve the necessary parameters.
  std::string air_speed_sub_topic;
  std::string gps_sub_topic;
  std::string ground_speed_sub_topic;
  std::string imu_sub_topic;

  pnh.param("air_speed_topic", air_speed_sub_topic, std::string(mav_msgs::default_topics::AIR_SPEED));
  pnh.param("gps_topic", gps_sub_topic, std::string(mav_msgs::default_topics::GPS));
  pnh.param("ground_speed_topic", ground_speed_sub_topic, std::string(mav_msgs::default_topics::GROUND_SPEED));
  pnh.param("imu_topic", imu_sub_topic, std::string(mav_msgs::default_topics::IMU));

  // Compute the rotation matrix to rotate data into NED frame
  q_S_B_ = q_S_B;
  R_S_B_ = q_S_B_.matrix().cast<float>();

  // Initialize the subscribers.
  air_speed_sub_ =
      nh_.subscribe<geometry_msgs::TwistStamped>(
          air_speed_sub_topic, 1, boost::bind(
              &HilListeners::AirSpeedCallback, &hil_listeners_, _1, &hil_data_));

  gps_sub_ =
      nh_.subscribe<sensor_msgs::NavSatFix>(
          gps_sub_topic, 1, boost::bind(
              &HilListeners::GpsCallback, &hil_listeners_, _1, &hil_data_));

  ground_speed_sub_ =
      nh_.subscribe<geometry_msgs::TwistStamped>(
          ground_speed_sub_topic, 1, boost::bind(
              &HilListeners::GroundSpeedCallback, &hil_listeners_, _1, &hil_data_));

  imu_sub_ =
      nh_.subscribe<sensor_msgs::Imu>(
          imu_sub_topic, 1, boost::bind(
              &HilListeners::ImuCallback, &hil_listeners_, _1, &hil_data_));
}

HilStateLevelInterface::~HilStateLevelInterface() {
}

std::vector<mavros_msgs::Mavlink> HilStateLevelInterface::CollectData() {
  boost::mutex::scoped_lock lock(mtx_);

  ros::Time current_time = ros::Time::now();
  uint64_t time_usec = RosTimeToMicroseconds(current_time);

  mavlink_message_t mmsg;
  std::vector<mavros_msgs::Mavlink> hil_msgs;

  // Rotate the attitude into NED frame
  Eigen::Quaterniond att = q_S_B_ * hil_data_.att;

  // Rotate gyroscope, accelerometer, and ground speed data into NED frame
  Eigen::Vector3f gyro = R_S_B_ * hil_data_.gyro_rad_per_s;
  Eigen::Vector3f acc = R_S_B_ * hil_data_.acc_m_per_s2;
  Eigen::Vector3i gps_vel = (R_S_B_ * hil_data_.gps_vel_cm_per_s.cast<float>()).cast<int>();

  // Fill in a MAVLINK HIL_STATE_QUATERNION message and convert it to MAVROS format.
  hil_state_qtrn_msg_.time_usec = time_usec;
  hil_state_qtrn_msg_.attitude_quaternion[0] = att.w();
  hil_state_qtrn_msg_.attitude_quaternion[1] = att.x();
  hil_state_qtrn_msg_.attitude_quaternion[2] = att.y();
  hil_state_qtrn_msg_.attitude_quaternion[3] = att.z();
  hil_state_qtrn_msg_.rollspeed = gyro.x();
  hil_state_qtrn_msg_.pitchspeed = gyro.y();
  hil_state_qtrn_msg_.yawspeed = gyro.z();
  hil_state_qtrn_msg_.lat = hil_data_.lat_1e7deg;
  hil_state_qtrn_msg_.lon = hil_data_.lon_1e7deg;
  hil_state_qtrn_msg_.alt = hil_data_.alt_mm;
  hil_state_qtrn_msg_.vx = gps_vel.x();
  hil_state_qtrn_msg_.vy = gps_vel.y();
  hil_state_qtrn_msg_.vz = gps_vel.z();
  hil_state_qtrn_msg_.ind_airspeed = hil_data_.ind_airspeed_1e2m_per_s;
  hil_state_qtrn_msg_.true_airspeed = hil_data_.true_airspeed_1e2m_per_s;
  hil_state_qtrn_msg_.xacc = acc.x() * kMetersToMm / kGravityMagnitude_m_per_s2;
  hil_state_qtrn_msg_.yacc = acc.y() * kMetersToMm / kGravityMagnitude_m_per_s2;
  hil_state_qtrn_msg_.zacc = acc.z() * kMetersToMm / kGravityMagnitude_m_per_s2;

  mavlink_hil_state_quaternion_t* hil_state_qtrn_msg_ptr = &hil_state_qtrn_msg_;
  mavlink_msg_hil_state_quaternion_encode(1, 0, &mmsg, hil_state_qtrn_msg_ptr);

  mavros_msgs::MavlinkPtr rmsg_hil_state_qtrn = boost::make_shared<mavros_msgs::Mavlink>();
  rmsg_hil_state_qtrn->header.stamp.sec = current_time.sec;
  rmsg_hil_state_qtrn->header.stamp.nsec = current_time.nsec;
  mavros_msgs::mavlink::convert(mmsg, *rmsg_hil_state_qtrn);

  hil_msgs.push_back(*rmsg_hil_state_qtrn);

  return hil_msgs;
}

}

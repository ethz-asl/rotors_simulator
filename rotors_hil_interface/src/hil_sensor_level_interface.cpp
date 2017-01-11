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

HilSensorLevelInterface::HilSensorLevelInterface(const Eigen::Quaterniond& q_S_B) {
  ros::NodeHandle pnh("~");

  // Retrieve the necessary parameters.
  double gps_freq;
  std::string air_speed_sub_topic;
  std::string gps_sub_topic;
  std::string ground_speed_sub_topic;
  std::string imu_sub_topic;
  std::string mag_sub_topic;
  std::string pressure_sub_topic;

  pnh.param("gps_frequency", gps_freq, kDefaultGpsFrequency);
  pnh.param("air_speed_topic", air_speed_sub_topic, std::string(mav_msgs::default_topics::AIR_SPEED));
  pnh.param("gps_topic", gps_sub_topic, std::string(mav_msgs::default_topics::GPS));
  pnh.param("ground_speed_topic", ground_speed_sub_topic, std::string(mav_msgs::default_topics::GROUND_SPEED));
  pnh.param("imu_topic", imu_sub_topic, std::string(mav_msgs::default_topics::IMU));
  pnh.param("mag_topic", mag_sub_topic, std::string(mav_msgs::default_topics::MAGNETIC_FIELD));
  pnh.param("pressure_topic", pressure_sub_topic, kDefaultPressureSubTopic);

  // Compute the desired interval between published GPS messages.
  gps_interval_nsec_ = static_cast<uint64_t>(kSecToNsec / gps_freq);

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

  mag_sub_ =
      nh_.subscribe<sensor_msgs::MagneticField>(
          mag_sub_topic, 1, boost::bind(
              &HilListeners::MagCallback, &hil_listeners_, _1, &hil_data_));

  pressure_sub_ =
      nh_.subscribe<sensor_msgs::FluidPressure>(
          pressure_sub_topic, 1, boost::bind(
              &HilListeners::PressureCallback, &hil_listeners_, _1, &hil_data_));
}

HilSensorLevelInterface::~HilSensorLevelInterface() {
}

std::vector<mavros_msgs::Mavlink> HilSensorLevelInterface::CollectData() {
  boost::mutex::scoped_lock lock(mtx_);

  ros::Time current_time = ros::Time::now();
  uint64_t time_usec = RosTimeToMicroseconds(current_time);

  mavlink_message_t mmsg;
  std::vector<mavros_msgs::Mavlink> hil_msgs;

  // Rotate gyroscope, accelerometer, and magnetometer data into NED frame
  Eigen::Vector3f gyro = R_S_B_ * hil_data_.gyro_rad_per_s;
  Eigen::Vector3f acc = R_S_B_ * hil_data_.acc_m_per_s2;
  Eigen::Vector3f mag = R_S_B_ * hil_data_.mag_G;

  // Check if we need to publish a HIL_GPS message.
  if ((current_time.nsec - last_gps_pub_time_nsec_) >= gps_interval_nsec_) {
    last_gps_pub_time_nsec_ = current_time.nsec;

    // Rotate ground speed data into NED frame
    Eigen::Vector3i gps_vel = (R_S_B_ * hil_data_.gps_vel_cm_per_s.cast<float>()).cast<int>();

    // Fill in a MAVLINK HIL_GPS message and convert it to MAVROS format.
    hil_gps_msg_.time_usec = time_usec;
    hil_gps_msg_.fix_type = hil_data_.fix_type;
    hil_gps_msg_.lat = hil_data_.lat_1e7deg;
    hil_gps_msg_.lon = hil_data_.lon_1e7deg;
    hil_gps_msg_.alt = hil_data_.alt_mm;
    hil_gps_msg_.eph = hil_data_.eph_cm;
    hil_gps_msg_.epv = hil_data_.epv_cm;
    hil_gps_msg_.vel = hil_data_.vel_1e2m_per_s;
    hil_gps_msg_.vn = gps_vel.x();
    hil_gps_msg_.ve = gps_vel.y();
    hil_gps_msg_.vd = gps_vel.z();
    hil_gps_msg_.cog = hil_data_.cog_1e2deg;
    hil_gps_msg_.satellites_visible = hil_data_.satellites_visible;

    mavlink_hil_gps_t* hil_gps_msg_ptr = &hil_gps_msg_;
    mavlink_msg_hil_gps_encode(1, 0, &mmsg, hil_gps_msg_ptr);

    mavros_msgs::MavlinkPtr rmsg_hil_gps = boost::make_shared<mavros_msgs::Mavlink>();
    rmsg_hil_gps->header.stamp.sec = current_time.sec;
    rmsg_hil_gps->header.stamp.nsec = current_time.nsec;
    mavros_msgs::mavlink::convert(mmsg, *rmsg_hil_gps);

    hil_msgs.push_back(*rmsg_hil_gps);
  }

  // Fill in a MAVLINK HIL_SENSOR message and convert it to MAVROS format.
  hil_sensor_msg_.time_usec = time_usec;
  hil_sensor_msg_.xacc = acc.x();
  hil_sensor_msg_.yacc = acc.y();
  hil_sensor_msg_.zacc = acc.z();
  hil_sensor_msg_.xgyro = gyro.x();
  hil_sensor_msg_.ygyro = gyro.y();
  hil_sensor_msg_.zgyro = gyro.z();
  hil_sensor_msg_.xmag = mag.x();
  hil_sensor_msg_.ymag = mag.y();
  hil_sensor_msg_.zmag = mag.z();
  hil_sensor_msg_.abs_pressure = hil_data_.pressure_abs_mBar;
  hil_sensor_msg_.diff_pressure = hil_data_.pressure_diff_mBar;
  hil_sensor_msg_.pressure_alt = hil_data_.pressure_alt;
  hil_sensor_msg_.temperature = hil_data_.temperature_degC;
  hil_sensor_msg_.fields_updated = kAllFieldsUpdated;

  mavlink_hil_sensor_t* hil_sensor_msg_ptr = &hil_sensor_msg_;
  mavlink_msg_hil_sensor_encode(1, 0, &mmsg, hil_sensor_msg_ptr);

  mavros_msgs::MavlinkPtr rmsg_hil_sensor = boost::make_shared<mavros_msgs::Mavlink>();
  rmsg_hil_sensor->header.stamp.sec = current_time.sec;
  rmsg_hil_sensor->header.stamp.nsec = current_time.nsec;
  mavros_msgs::mavlink::convert(mmsg, *rmsg_hil_sensor);

  hil_msgs.push_back(*rmsg_hil_sensor);

  return hil_msgs;
}

}

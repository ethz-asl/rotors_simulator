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
HilStateLevelInterface::HilStateLevelInterface() {
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

HilStateLevelInterface::~HilStateLevelInterface() {}

mavros_msgs::HilStateQuaternion HilStateLevelInterface::CollectStateData() {
	boost::mutex::scoped_lock lock(mtx_);

	mavros_msgs::HilStateQuaternionPtr hil_state_msg = boost::make_shared<mavros_msgs::HilStateQuaternion>();

	// Fill in a MAVROS HIL_STATE_QUATERNION message.
	hil_state_msg->header.stamp = ros::Time::now();
	hil_state_msg->orientation.w = hil_data_.att.w();
	hil_state_msg->orientation.x = hil_data_.att.x();
	hil_state_msg->orientation.y = hil_data_.att.y();
	hil_state_msg->orientation.z = hil_data_.att.z();
	hil_state_msg->angular_velocity.x = hil_data_.gyro.x();
	hil_state_msg->angular_velocity.y = hil_data_.gyro.y();
	hil_state_msg->angular_velocity.z = hil_data_.gyro.z();
	hil_state_msg->geo.latitude = hil_data_.lat;
	hil_state_msg->geo.longitude = hil_data_.lon;
	hil_state_msg->geo.altitude = hil_data_.alt;
	hil_state_msg->linear_velocity.x = hil_data_.gps_vel.x();
	hil_state_msg->linear_velocity.y = hil_data_.gps_vel.y();
	hil_state_msg->linear_velocity.z = hil_data_.gps_vel.z();
	hil_state_msg->ind_airspeed = hil_data_.ind_airspeed;
	hil_state_msg->true_airspeed = hil_data_.true_airspeed;
	hil_state_msg->linear_acceleration.x = hil_data_.acc.x() * kGravityMagnitude_m_per_s2;
	hil_state_msg->linear_acceleration.y = hil_data_.acc.y() * kGravityMagnitude_m_per_s2;
	hil_state_msg->linear_acceleration.z = hil_data_.acc.z() * kGravityMagnitude_m_per_s2;

	return *hil_state_msg;
}
}	// namespace rotors_hil

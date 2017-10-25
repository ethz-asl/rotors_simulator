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
HilSensorLevelInterface::HilSensorLevelInterface() {
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

HilSensorLevelInterface::~HilSensorLevelInterface() {}

mavros_msgs::HilGPS HilSensorLevelInterface::CollectGPSData() {
	boost::mutex::scoped_lock lock(mtx_);

	mavros_msgs::HilGPSPtr hil_gps_msg = boost::make_shared<mavros_msgs::HilGPS>();

	// Check if we need to publish a HIL_GPS message.
	if ((ros::Time::now().toNSec() - last_gps_pub_time_nsec_) >= gps_interval_nsec_) {
		last_gps_pub_time_nsec_ = ros::Time::now().toNSec();

		// Fill in a MAVROS HIL_GPS message.
		hil_gps_msg->header.stamp = ros::Time::now();
		hil_gps_msg->fix_type = hil_data_.fix_type;
		hil_gps_msg->geo.latitude = hil_data_.lat;
		hil_gps_msg->geo.longitude = hil_data_.lon;
		hil_gps_msg->geo.altitude = hil_data_.alt;
		hil_gps_msg->eph = hil_data_.eph;
		hil_gps_msg->epv = hil_data_.epv;
		hil_gps_msg->vel = hil_data_.vel;
		hil_gps_msg->vn = hil_data_.gps_vel.x();
		hil_gps_msg->ve = hil_data_.gps_vel.y();
		hil_gps_msg->vd = hil_data_.gps_vel.z();
		hil_gps_msg->cog = hil_data_.cog;
		hil_gps_msg->satellites_visible = hil_data_.satellites_visible;

		return *hil_gps_msg;
	}
}

mavros_msgs::HilSensor HilSensorLevelInterface::CollectSensorData() {
	boost::mutex::scoped_lock lock(mtx_);

	mavros_msgs::HilSensorPtr hil_sensor_msg = boost::make_shared<mavros_msgs::HilSensor>();

	// Fill in a ROS HIL_SENSOR message.
	hil_sensor_msg->header.stamp = ros::Time::now();
	hil_sensor_msg->acc.x = hil_data_.acc.x();
	hil_sensor_msg->acc.y = hil_data_.acc.y();
	hil_sensor_msg->acc.z = hil_data_.acc.z();
	hil_sensor_msg->gyro.x = hil_data_.gyro.x();
	hil_sensor_msg->gyro.y = hil_data_.gyro.y();
	hil_sensor_msg->gyro.z = hil_data_.gyro.z();
	hil_sensor_msg->mag.x = hil_data_.mag.x();
	hil_sensor_msg->mag.y = hil_data_.mag.y();
	hil_sensor_msg->mag.z = hil_data_.mag.z();
	hil_sensor_msg->abs_pressure = hil_data_.pressure_abs;
	hil_sensor_msg->diff_pressure = hil_data_.pressure_diff;
	hil_sensor_msg->pressure_alt = hil_data_.pressure_alt;
	hil_sensor_msg->temperature = hil_data_.temperature;
	hil_sensor_msg->fields_updated = kAllFieldsUpdated;

	return *hil_sensor_msg;
}
}	// namespace rotors_hil

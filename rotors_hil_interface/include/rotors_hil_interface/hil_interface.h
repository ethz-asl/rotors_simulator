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

#ifndef ROTORS_HIL_INTERFACE_H_
#define ROTORS_HIL_INTERFACE_H_

#include <mav_msgs/Actuators.h>
#include <mav_msgs/default_topics.h>
#include <mavros_msgs/HilControls.h>
#include <mavros_msgs/HilGPS.h>
#include <mavros_msgs/HilSensor.h>
#include <mavros_msgs/HilStateQuaternion.h>

#include <rotors_hil_interface/hil_listeners.h>

namespace rotors_hil {
// Constants
static constexpr int kAllFieldsUpdated = 4095;

// Default values
static constexpr double kDefaultGpsFrequency = 5.0;
static const std::string kDefaultPressureSubTopic = "air_pressure";


class HilInterface {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/// \brief Destructor
	virtual ~HilInterface() {};

	/// \brief Gather data collected from ROS messages into MAVROS HilGPS message.
	/// \return HilSensor message to be publised.
	virtual mavros_msgs::HilGPS CollectGPSData() {};

	/// \brief Gather data collected from ROS messages into MAVROS HilSensor message.
	/// \return MAVROS HilSensor message to be publised.
	virtual mavros_msgs::HilSensor CollectSensorData() {};

	/// \brief Gather data collected from ROS messages into MAVROS HilSensor message.
	/// \return MAVROS HilStateQuaternion message to be publised.
	virtual mavros_msgs::HilStateQuaternion CollectStateData() {};

protected:
	/// ROS node handle.
	ros::NodeHandle nh_;

	/// ROS air speed subscriber.
	ros::Subscriber air_speed_sub_;

	/// ROS GPS subscriber.
	ros::Subscriber gps_sub_;

	/// ROS ground speed subscriber.
	ros::Subscriber ground_speed_sub_;

	/// ROS IMU subscriber.
	ros::Subscriber imu_sub_;

	/// ROS magnetometer subscriber.
	ros::Subscriber mag_sub_;

	/// ROS air pressure subscriber.
	ros::Subscriber pressure_sub_;

	/// Object for storing the latest data.
	HilData hil_data_;

	/// Object with callbacks for receiving data.
	HilListeners hil_listeners_;

	/// Mutex lock for thread safety of reading hil data.
	boost::mutex mtx_;
};

class HilSensorLevelInterface : public HilInterface {
public:
	/// \brief Constructor
	HilSensorLevelInterface();

	/// \brief Destructor
	virtual ~HilSensorLevelInterface();

	mavros_msgs::HilGPS CollectGPSData();
	mavros_msgs::HilSensor CollectSensorData();

private:
	/// Interval between outgoing HIL_GPS messages.
	uint64_t gps_interval_nsec_;

	/// Nanosecond portion of the last HIL_GPS message timestamp.
	uint64_t last_gps_pub_time_nsec_;
};

class HilStateLevelInterface : public HilInterface {
public:
	/// \brief Constructor
	HilStateLevelInterface();

	/// \brief Destructor
	virtual ~HilStateLevelInterface();

	mavros_msgs::HilStateQuaternion CollectStateData();
};
}	// namespace rotors_hil

#endif	// ROTORS_HIL_INTERFACE_H_

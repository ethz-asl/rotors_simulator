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

#ifndef ROTORS_HIL_INTERFACE_NODE_H_
#define ROTORS_HIL_INTERFACE_NODE_H_

#include <memory>

#include <rotors_hil_interface/hil_interface.h>

namespace rotors_hil {
// Default values
static constexpr bool kDefaultSensorLevelHil = true;
static constexpr double kDefaultHilFrequency = 100.0;
static const std::string kDefaultHilGPSPubTopic = "mavros/hil/gps";
static const std::string kDefaultHilSensorPubTopic = "mavros/hil/imu_ned";
static const std::string kDefaultHilStatePubTopic = "mavros/hil/state";
static const std::string kDefaultHilControlsSubTopic = "mavros/hil/controls";

class HilInterfaceNode {
public:
	HilInterfaceNode();
	virtual ~HilInterfaceNode();

	/// \brief Main execution loop.
	void MainTask();

	/// \brief Callback for handling HilControls messages.
	/// \param[in] hil_controls_msg A HilControls message.
	void HilControlsCallback(const mavros_msgs::HilControlsConstPtr& hil_controls_msg);

private:
	/// ROS node handle.
	ros::NodeHandle nh_;

	/// \brief Choose the interface level
	bool sensor_level_hil;

	/// ROS publisher for sending actuator commands.
	ros::Publisher actuators_pub_;

	/// ROS publisher for sending MAVROS HilGPS messages.
	ros::Publisher hil_gps_pub_;

	/// ROS publisher for sending MAVROS HilSensor messages.
	ros::Publisher hil_sensor_pub_;

	/// ROS publisher for sending MAVROS HilStateQuaternion messages.
	ros::Publisher hil_state_pub_;

	/// ROS subscriber for handling HilControls messages.
	ros::Subscriber hil_controls_sub_;

	/// Object for spinning.
	ros::Rate rate_;

	/// Pointer to the HIL interface object.
	std::shared_ptr<HilInterface> hil_interface_;
};
}

#endif	// ROTORS_HIL_INTERFACE_NODE_H_

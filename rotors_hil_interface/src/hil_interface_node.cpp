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

#include "rotors_hil_interface/hil_interface_node.h"

namespace rotors_hil {
HilInterfaceNode::HilInterfaceNode() :
	rate_(kDefaultHilFrequency) {
	ros::NodeHandle pnh("~");

	double hil_frequency;
	std::string actuators_pub_topic;
	std::string hil_gps_pub_topic;
	std::string hil_sensor_pub_topic;
	std::string hil_state_pub_topic;
	std::string hil_controls_sub_topic;

	pnh.param("sensor_level_hil", sensor_level_hil, kDefaultSensorLevelHil);
	pnh.param("hil_frequency", hil_frequency, kDefaultHilFrequency);
	pnh.param("actuators_pub_topic", actuators_pub_topic, std::string(mav_msgs::default_topics::COMMAND_ACTUATORS));
	pnh.param("hil_gps_pub_topic", hil_gps_pub_topic, kDefaultHilGPSPubTopic);
	pnh.param("hil_sensor_pub_topic", hil_sensor_pub_topic, kDefaultHilSensorPubTopic);
	pnh.param("hil_state_pub_topic", hil_state_pub_topic, kDefaultHilStatePubTopic);
	pnh.param("hil_controls_sub_topic", hil_controls_sub_topic, kDefaultHilControlsSubTopic);

	if (sensor_level_hil)
		hil_interface_ = std::shared_ptr<HilSensorLevelInterface>(new HilSensorLevelInterface());
	else
		hil_interface_ = std::shared_ptr<HilStateLevelInterface>(new HilStateLevelInterface());

	rate_ = ros::Rate(hil_frequency);

	// Publishers
	actuators_pub_ = nh_.advertise<mav_msgs::Actuators>(actuators_pub_topic, 1);
	hil_gps_pub_ = nh_.advertise<mavros_msgs::HilGPS>(hil_gps_pub_topic, 10);
	hil_sensor_pub_ = nh_.advertise<mavros_msgs::HilSensor>(hil_sensor_pub_topic, 10);
	hil_state_pub_ = nh_.advertise<mavros_msgs::HilStateQuaternion>(hil_state_pub_topic, 10);

	// Subscribers
	hil_controls_sub_ = nh_.subscribe(hil_controls_sub_topic, 1,
			&HilInterfaceNode::HilControlsCallback, this);
}

HilInterfaceNode::~HilInterfaceNode() {}

void HilInterfaceNode::MainTask() {
	while (ros::ok()) {
		if (sensor_level_hil) {
			mavros_msgs::HilGPS hil_gps_msg = hil_interface_->CollectGPSData();
			mavros_msgs::HilSensor hil_sensor_msg = hil_interface_->CollectSensorData();

			hil_gps_pub_.publish(hil_gps_msg);
			hil_sensor_pub_.publish(hil_sensor_msg);
		}
		else {
			mavros_msgs::HilStateQuaternion hil_state_msg = hil_interface_->CollectStateData();

			hil_state_pub_.publish(hil_state_msg);
		}

		ros::spinOnce();
		rate_.sleep();
	}
}

void HilInterfaceNode::HilControlsCallback(const mavros_msgs::HilControlsConstPtr& hil_controls_msg) {
	mav_msgs::Actuators act_msg;

	ros::Time current_time = ros::Time::now();

	act_msg.normalized.push_back(hil_controls_msg->roll_ailerons);
	act_msg.normalized.push_back(hil_controls_msg->pitch_elevator);
	act_msg.normalized.push_back(hil_controls_msg->yaw_rudder);
	act_msg.normalized.push_back(hil_controls_msg->aux1);
	act_msg.normalized.push_back(hil_controls_msg->aux2);
	act_msg.normalized.push_back(hil_controls_msg->throttle);

	act_msg.header.stamp.sec = current_time.sec;
	act_msg.header.stamp.nsec = current_time.nsec;

	actuators_pub_.publish(act_msg);
}
}	// namespace rotors_hil

int main(int argc, char** argv) {
	ros::init(argc, argv, "rotors_hil_interface_node");
	rotors_hil::HilInterfaceNode hil_interface_node;

	hil_interface_node.MainTask();

	return 0;
}

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

#include "roll_pitch_yawrate_thrust_controller_node.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control {

RollPitchYawrateThrustControllerNode::RollPitchYawrateThrustControllerNode() {
  InitializeParams();

  ros::NodeHandle nh;

  cmd_roll_pitch_yawrate_thrust_sub_ = nh.subscribe(kDefaultCommandRollPitchYawrateThrustTopic, 1,
                                     &RollPitchYawrateThrustControllerNode::RollPitchYawrateThrustCallback, this);
  odometry_sub_ = nh.subscribe(kDefaultOdometryTopic, 1,
                               &RollPitchYawrateThrustControllerNode::OdometryCallback, this);

  motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(
      kDefaultCommandMotorSpeedTopic, 1);
}

RollPitchYawrateThrustControllerNode::~RollPitchYawrateThrustControllerNode() { }

void RollPitchYawrateThrustControllerNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  // Read parameters from rosparam.
  GetRosParameter(pnh, "attitude_gain/x",
                  roll_pitch_yawrate_thrust_controller_.controller_parameters_.attitude_gain_.x(),
                  &roll_pitch_yawrate_thrust_controller_.controller_parameters_.attitude_gain_.x());
  GetRosParameter(pnh, "attitude_gain/y",
                  roll_pitch_yawrate_thrust_controller_.controller_parameters_.attitude_gain_.y(),
                  &roll_pitch_yawrate_thrust_controller_.controller_parameters_.attitude_gain_.y());
  GetRosParameter(pnh, "attitude_gain/z",
                  roll_pitch_yawrate_thrust_controller_.controller_parameters_.attitude_gain_.z(),
                  &roll_pitch_yawrate_thrust_controller_.controller_parameters_.attitude_gain_.z());
  GetRosParameter(pnh, "angular_rate_gain/x",
                  roll_pitch_yawrate_thrust_controller_.controller_parameters_.angular_rate_gain_.x(),
                  &roll_pitch_yawrate_thrust_controller_.controller_parameters_.angular_rate_gain_.x());
  GetRosParameter(pnh, "angular_rate_gain/y",
                  roll_pitch_yawrate_thrust_controller_.controller_parameters_.angular_rate_gain_.y(),
                  &roll_pitch_yawrate_thrust_controller_.controller_parameters_.angular_rate_gain_.y());
  GetRosParameter(pnh, "angular_rate_gain/z",
                  roll_pitch_yawrate_thrust_controller_.controller_parameters_.angular_rate_gain_.z(),
                  &roll_pitch_yawrate_thrust_controller_.controller_parameters_.angular_rate_gain_.z());
  GetVehicleParameters(pnh, &roll_pitch_yawrate_thrust_controller_.vehicle_parameters_);
  roll_pitch_yawrate_thrust_controller_.InitializeParameters();
}
void RollPitchYawrateThrustControllerNode::Publish() {
}

void RollPitchYawrateThrustControllerNode::RollPitchYawrateThrustCallback(
    const mav_msgs::RollPitchYawrateThrustConstPtr& roll_pitch_yawrate_thrust_reference_msg) {
  mav_msgs::EigenRollPitchYawrateThrust roll_pitch_yawrate_thrust;
  mav_msgs::eigenRollPitchYawrateThrustFromMsg(*roll_pitch_yawrate_thrust_reference_msg, &roll_pitch_yawrate_thrust);
  roll_pitch_yawrate_thrust_controller_.SetRollPitchYawrateThrust(roll_pitch_yawrate_thrust);
}


void RollPitchYawrateThrustControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  ROS_INFO_ONCE("RollPitchYawrateThrustController got first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  roll_pitch_yawrate_thrust_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
  roll_pitch_yawrate_thrust_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  actuator_msg->header.stamp = odometry_msg->header.stamp;

  motor_velocity_reference_pub_.publish(actuator_msg);
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "roll_pitch_yawrate_thrust_controller_node");

  rotors_control::RollPitchYawrateThrustControllerNode roll_pitch_yawrate_thrust_controller_node;

  ros::spin();

  return 0;
}

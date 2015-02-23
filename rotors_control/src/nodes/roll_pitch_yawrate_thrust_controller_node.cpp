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
  google::InitGoogleLogging("rotors_control_glogger");
  InitializeParams();

  ros::NodeHandle nh;

  cmd_roll_pitch_yawrate_thrust_sub_ = nh.subscribe(kDefaultCommandRollPitchYawrateThrustTopic, 10,
                                     &RollPitchYawrateThrustControllerNode::CommandRollPitchYawrateThrustCallback, this);
  odometry_sub_ = nh.subscribe(kDefaultOdometryTopic, 10,
                               &RollPitchYawrateThrustControllerNode::OdometryCallback, this);

  motor_velocity_reference_pub_ = nh.advertise<mav_msgs::CommandMotorSpeed>(
      kDefaultMotorSpeedTopic, 10);
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
  GetRosParameter(pnh, "mass",
                  roll_pitch_yawrate_thrust_controller_.vehicle_parameters_.mass_,
                  &roll_pitch_yawrate_thrust_controller_.vehicle_parameters_.mass_);
  GetRosParameter(pnh, "inertia/xx",
                  roll_pitch_yawrate_thrust_controller_.vehicle_parameters_.inertia_(0, 0),
                  &roll_pitch_yawrate_thrust_controller_.vehicle_parameters_.inertia_(0, 0));
  GetRosParameter(pnh, "inertia/xy",
                  roll_pitch_yawrate_thrust_controller_.vehicle_parameters_.inertia_(0, 1),
                  &roll_pitch_yawrate_thrust_controller_.vehicle_parameters_.inertia_(0, 1));
  roll_pitch_yawrate_thrust_controller_.vehicle_parameters_.inertia_(1, 0) =
      roll_pitch_yawrate_thrust_controller_.vehicle_parameters_.inertia_(0, 1);
  GetRosParameter(pnh, "inertia/xz",
                  roll_pitch_yawrate_thrust_controller_.vehicle_parameters_.inertia_(0, 2),
                  &roll_pitch_yawrate_thrust_controller_.vehicle_parameters_.inertia_(0, 2));
  roll_pitch_yawrate_thrust_controller_.vehicle_parameters_.inertia_(2, 0) =
      roll_pitch_yawrate_thrust_controller_.vehicle_parameters_.inertia_(0, 2);
  GetRosParameter(pnh, "inertia/yy",
                  roll_pitch_yawrate_thrust_controller_.vehicle_parameters_.inertia_(1, 1),
                  &roll_pitch_yawrate_thrust_controller_.vehicle_parameters_.inertia_(1, 1));
  GetRosParameter(pnh, "inertia/yz",
                  roll_pitch_yawrate_thrust_controller_.vehicle_parameters_.inertia_(1, 2),
                  &roll_pitch_yawrate_thrust_controller_.vehicle_parameters_.inertia_(1, 2));
  roll_pitch_yawrate_thrust_controller_.vehicle_parameters_.inertia_(2, 1) =
      roll_pitch_yawrate_thrust_controller_.vehicle_parameters_.inertia_(1, 2);
  GetRosParameter(pnh, "inertia/zz",
                  roll_pitch_yawrate_thrust_controller_.vehicle_parameters_.inertia_(2, 2),
                  &roll_pitch_yawrate_thrust_controller_.vehicle_parameters_.inertia_(2, 2));

  // Get the rotor configuration.
  std::map<std::string, double> single_rotor;
  std::string rotor_configuration_string = "rotor_configuration/";
  unsigned int i = 0;
  while (pnh.getParam(rotor_configuration_string + std::to_string(i), single_rotor)) {
    if (i == 0) {
      roll_pitch_yawrate_thrust_controller_.vehicle_parameters_.rotor_configuration_.rotors.clear();
    }
    Rotor rotor;
    pnh.getParam(rotor_configuration_string + std::to_string(i) + "/angle",
                 rotor.angle);
    pnh.getParam(rotor_configuration_string + std::to_string(i) + "/arm_length",
                 rotor.arm_length);
    pnh.getParam(rotor_configuration_string + std::to_string(i) + "/rotor_force_constant",
                 rotor.rotor_force_constant);
    pnh.getParam(rotor_configuration_string + std::to_string(i) + "/rotor_moment_constant",
                 rotor.rotor_moment_constant);
    pnh.getParam(rotor_configuration_string + std::to_string(i) + "/direction",
                 rotor.direction);
    roll_pitch_yawrate_thrust_controller_.vehicle_parameters_.rotor_configuration_.rotors.push_back(rotor);
    ++i;
  }
  roll_pitch_yawrate_thrust_controller_.InitializeParameters();
}
void RollPitchYawrateThrustControllerNode::Publish() {
}

void RollPitchYawrateThrustControllerNode::CommandRollPitchYawrateThrustCallback(
    const mav_msgs::CommandRollPitchYawrateThrustConstPtr& roll_pitch_yawrate_thrust_reference_msg) {
  mav_msgs::EigenCommandRollPitchYawrateThrust roll_pitch_yawrate_thrust;
  mav_msgs::eigenCommandRollPitchYawrateThrustFromMsg(*roll_pitch_yawrate_thrust_reference_msg, &roll_pitch_yawrate_thrust);
  roll_pitch_yawrate_thrust_controller_.SetCommandRollPitchYawrateThrust(roll_pitch_yawrate_thrust);
}


void RollPitchYawrateThrustControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  ROS_INFO_ONCE("RollPitchYawrateThrustController got first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  roll_pitch_yawrate_thrust_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
  roll_pitch_yawrate_thrust_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::CommandMotorSpeedPtr turning_velocities_msg(new mav_msgs::CommandMotorSpeed);

  turning_velocities_msg->motor_speed.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    turning_velocities_msg->motor_speed.push_back(ref_rotor_velocities[i]);
  turning_velocities_msg->header.stamp = odometry_msg->header.stamp;

  motor_velocity_reference_pub_.publish(turning_velocities_msg);
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "roll_pitch_yawrate_thrust_controller_node");

  rotors_control::RollPitchYawrateThrustControllerNode roll_pitch_yawrate_thrust_controller_node;

  ros::spin();

  return 0;
}

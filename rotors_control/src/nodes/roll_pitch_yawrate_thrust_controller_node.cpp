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

#include "roll_pitch_yawrate_thrust_controller_node.hpp"

#include "rotors_control/parameters_ros.hpp"

namespace rotors_control {

    RollPitchYawrateThrustControllerNode::RollPitchYawrateThrustControllerNode() : Node("roll_pitch_yawrate_thrust_controller_node"), node_handle(std::shared_ptr<RollPitchYawrateThrustControllerNode>(this, [](auto *) {})) {

        InitializeParams();

        cmd_roll_pitch_yawrate_thrust_sub_= this->create_subscription<mav_msgs::msg::RollPitchYawrateThrust>(kDefaultCommandRollPitchYawrateThrustTopic, 1, std::bind(&RollPitchYawrateThrustControllerNode::RollPitchYawrateThrustCallback, this, std::placeholders::_1));
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(kDefaultOdometryTopic, 1, std::bind(&RollPitchYawrateThrustControllerNode::OdometryCallback, this, std::placeholders::_1));

        motor_velocity_reference_pub_ = this->create_publisher<actuator_msgs::msg::Actuators>(kDefaultCommandMotorSpeedTopic, 1);
    }

    RollPitchYawrateThrustControllerNode::~RollPitchYawrateThrustControllerNode() {}

    void RollPitchYawrateThrustControllerNode::InitializeParams() {

        GetRosParameter(node_handle, "attitude_gain/x",
                        roll_pitch_yawrate_thrust_controller_.controller_parameters_.attitude_gain_.x(),
                        &roll_pitch_yawrate_thrust_controller_.controller_parameters_.attitude_gain_.x());
        GetRosParameter(node_handle, "attitude_gain/y",
                        roll_pitch_yawrate_thrust_controller_.controller_parameters_.attitude_gain_.y(),
                        &roll_pitch_yawrate_thrust_controller_.controller_parameters_.attitude_gain_.y());
        GetRosParameter(node_handle, "attitude_gain/z",
                        roll_pitch_yawrate_thrust_controller_.controller_parameters_.attitude_gain_.z(),
                        &roll_pitch_yawrate_thrust_controller_.controller_parameters_.attitude_gain_.z());
        GetRosParameter(node_handle, "angular_rate_gain/x",
                        roll_pitch_yawrate_thrust_controller_.controller_parameters_.angular_rate_gain_.x(),
                        &roll_pitch_yawrate_thrust_controller_.controller_parameters_.angular_rate_gain_.x());
        GetRosParameter(node_handle, "angular_rate_gain/y",
                        roll_pitch_yawrate_thrust_controller_.controller_parameters_.angular_rate_gain_.y(),
                        &roll_pitch_yawrate_thrust_controller_.controller_parameters_.angular_rate_gain_.y());
        GetRosParameter(node_handle, "angular_rate_gain/z",
                        roll_pitch_yawrate_thrust_controller_.controller_parameters_.angular_rate_gain_.z(),
                        &roll_pitch_yawrate_thrust_controller_.controller_parameters_.angular_rate_gain_.z());
        GetVehicleParameters(node_handle, &roll_pitch_yawrate_thrust_controller_.vehicle_parameters_);
        roll_pitch_yawrate_thrust_controller_.InitializeParameters();
    }

    void RollPitchYawrateThrustControllerNode::Publish() {}

    void RollPitchYawrateThrustControllerNode::RollPitchYawrateThrustCallback(const mav_msgs::msg::RollPitchYawrateThrust::ConstSharedPtr & roll_pitch_yawrate_thrust_reference_msg)
    {
        mav_msgs::EigenRollPitchYawrateThrust roll_pitch_yawrate_thrust;
        mav_msgs::eigenRollPitchYawrateThrustFromMsg(*roll_pitch_yawrate_thrust_reference_msg, &roll_pitch_yawrate_thrust);
        roll_pitch_yawrate_thrust_controller_.SetRollPitchYawrateThrust(roll_pitch_yawrate_thrust);
    }

    void RollPitchYawrateThrustControllerNode::OdometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "RollPitchYawrateThrustController got first odometry message.");

        EigenOdometry odometry;
        eigenOdometryFromMsg(odometry_msg, &odometry);
        roll_pitch_yawrate_thrust_controller_.SetOdometry(odometry);

        Eigen::VectorXd ref_rotor_velocities;
        roll_pitch_yawrate_thrust_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

        actuator_msgs::msg::Actuators::SharedPtr actuator_msg(new actuator_msgs::msg::Actuators);

        actuator_msg->velocity.clear();
        for (int i = 0; i < ref_rotor_velocities.size(); i++)
            actuator_msg->velocity.push_back(ref_rotor_velocities[i]);
        actuator_msg->header.stamp = odometry_msg->header.stamp;

        motor_velocity_reference_pub_->publish(*actuator_msg);  //check

    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rotors_control::RollPitchYawrateThrustControllerNode>());
    rclcpp::shutdown();
    return 0;
}
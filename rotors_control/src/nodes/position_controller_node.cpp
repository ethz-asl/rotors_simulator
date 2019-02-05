/*
 * Copyright 2018 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2018 Luigi Iannelli, University of Sannio in Benevento, Italy
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

#include "position_controller_node.h"

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <ros/console.h> 
#include <sensor_msgs/Imu.h>
#include <time.h>
#include <chrono>

#include "rotors_control/parameters_ros.h"
#include "rotors_control/stabilizer_types.h"
#include "rotors_control/crazyflie_complementary_filter.h"

#define ATTITUDE_UPDATE_DT 0.004  /* ATTITUDE UPDATE RATE [s] - 500Hz */
#define RATE_UPDATE_DT 0.002      /* RATE UPDATE RATE [s] - 250Hz */
#define SAMPLING_TIME  0.01       /* SAMPLING CONTROLLER TIME [s] - 100Hz */

namespace rotors_control {

PositionControllerNode::PositionControllerNode() {

    ROS_INFO_ONCE("Started position controller");

    InitializeParams();

    ros::NodeHandle nh;
    ros::NodeHandle pnh_node("~");

    cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,  &PositionControllerNode::MultiDofJointTrajectoryCallback, this);

    odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &PositionControllerNode::OdometryCallback, this);

    motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

    // The subscription to the IMU topic is made only if it is available, when simulating the Crazyflie dynamics considering the also IMU values
    if(pnh_node.getParam("enable_state_estimator", enable_state_estimator_)){
      ROS_INFO("Got param 'enable_state_estimator': %d");
    }

    if (enable_state_estimator_){
        imu_sub_ = nh.subscribe(mav_msgs::default_topics::IMU, 1, &PositionControllerNode::IMUCallback, this);

        //Timers allow to set up the working frequency of the control system
        timer_Attitude_ = n_.createTimer(ros::Duration(ATTITUDE_UPDATE_DT), &PositionControllerNode::CallbackAttitudeEstimation, this, false, true);

        timer_highLevelControl = n_.createTimer(ros::Duration(SAMPLING_TIME), &PositionControllerNode::CallbackHightLevelControl, this, false, true);

        timer_IMUUpdate = n_.createTimer(ros::Duration(RATE_UPDATE_DT), &PositionControllerNode::CallbackIMUUpdate, this, false, true);

    }


}

PositionControllerNode::~PositionControllerNode(){}


void PositionControllerNode::MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference);

  // We can trigger the first command immediately.
  position_controller_.SetTrajectoryPoint(eigen_reference);
  commands_.pop_front();

  if (n_commands >= 1) {
    waypointHasBeenPublished_ = true;
    ROS_INFO("PositionController got first MultiDOFJointTrajectory message.");
  }
}


void PositionControllerNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  // Parameters reading from rosparam.
  GetRosParameter(pnh, "xy_gain_kp/x",
                  position_controller_.controller_parameters_.xy_gain_kp_.x(),
                  &position_controller_.controller_parameters_.xy_gain_kp_.x());
  GetRosParameter(pnh, "xy_gain_kp/y",
                  position_controller_.controller_parameters_.xy_gain_kp_.y(),
                  &position_controller_.controller_parameters_.xy_gain_kp_.y());
  GetRosParameter(pnh, "xy_gain_ki/x",
                  position_controller_.controller_parameters_.xy_gain_ki_.x(),
                  &position_controller_.controller_parameters_.xy_gain_ki_.x());
  GetRosParameter(pnh, "xy_gain_ki/y",
                  position_controller_.controller_parameters_.xy_gain_ki_.y(),
                  &position_controller_.controller_parameters_.xy_gain_ki_.y());

  GetRosParameter(pnh, "attitude_gain_kp/phi",
                  position_controller_.controller_parameters_.attitude_gain_kp_.x(),
                  &position_controller_.controller_parameters_.attitude_gain_kp_.x());
  GetRosParameter(pnh, "attitude_gain_kp/phi",
                  position_controller_.controller_parameters_.attitude_gain_kp_.y(),
                  &position_controller_.controller_parameters_.attitude_gain_kp_.y());
  GetRosParameter(pnh, "attitude_gain_ki/theta",
                  position_controller_.controller_parameters_.attitude_gain_ki_.x(),
                  &position_controller_.controller_parameters_.attitude_gain_ki_.x());
  GetRosParameter(pnh, "attitude_gain_ki/phi",
                  position_controller_.controller_parameters_.attitude_gain_ki_.y(),
                  &position_controller_.controller_parameters_.attitude_gain_ki_.y());

  GetRosParameter(pnh, "rate_gain_kp/p",
                  position_controller_.controller_parameters_.rate_gain_kp_.x(),
                  &position_controller_.controller_parameters_.rate_gain_kp_.x());
  GetRosParameter(pnh, "rate_gain_kp/q",
                  position_controller_.controller_parameters_.rate_gain_kp_.y(),
                  &position_controller_.controller_parameters_.rate_gain_kp_.y());
  GetRosParameter(pnh, "rate_gain_kp/r",
                  position_controller_.controller_parameters_.rate_gain_kp_.z(),
                  &position_controller_.controller_parameters_.rate_gain_kp_.z());
  GetRosParameter(pnh, "rate_gain_ki/p",
                  position_controller_.controller_parameters_.rate_gain_ki_.x(),
                  &position_controller_.controller_parameters_.rate_gain_ki_.x());
  GetRosParameter(pnh, "rate_gain_ki/q",
                  position_controller_.controller_parameters_.rate_gain_ki_.y(),
                  &position_controller_.controller_parameters_.rate_gain_ki_.y());
  GetRosParameter(pnh, "rate_gain_ki/r",
                  position_controller_.controller_parameters_.rate_gain_ki_.z(),
                  &position_controller_.controller_parameters_.rate_gain_ki_.z());

  GetRosParameter(pnh, "yaw_gain_kp/yaw",
                  position_controller_.controller_parameters_.yaw_gain_kp_,
                  &position_controller_.controller_parameters_.yaw_gain_kp_);
  GetRosParameter(pnh, "yaw_gain_ki/yaw",
                  position_controller_.controller_parameters_.yaw_gain_ki_,
                  &position_controller_.controller_parameters_.yaw_gain_ki_);

  GetRosParameter(pnh, "hovering_gain_kp/z",
                  position_controller_.controller_parameters_.hovering_gain_kp_,
                  &position_controller_.controller_parameters_.hovering_gain_kp_);
  GetRosParameter(pnh, "hovering_gain_ki/z",
                  position_controller_.controller_parameters_.hovering_gain_ki_,
                  &position_controller_.controller_parameters_.hovering_gain_ki_);
  GetRosParameter(pnh, "hovering_gain_kd/z",
                  position_controller_.controller_parameters_.hovering_gain_kd_,
                  &position_controller_.controller_parameters_.hovering_gain_kd_);

  position_controller_.SetControllerGains();
  if (enable_state_estimator_)
    position_controller_.crazyflie_onboard_controller_.SetControllerGains(position_controller_.controller_parameters_);

}

void PositionControllerNode::Publish(){
}

void PositionControllerNode::IMUCallback(const sensor_msgs::ImuConstPtr& imu_msg) {

    ROS_INFO_ONCE("PositionController got first imu message.");
    
    // Angular velocities data
    sensors_.gyro.x = imu_msg->angular_velocity.x;
    sensors_.gyro.y = imu_msg->angular_velocity.y;
    sensors_.gyro.z = imu_msg->angular_velocity.z;
    
    // Linear acceleration data
    sensors_.acc.x = imu_msg->linear_acceleration.x;
    sensors_.acc.y = imu_msg->linear_acceleration.y;
    sensors_.acc.z = imu_msg->linear_acceleration.z;

    imu_msg_head_stamp_ = imu_msg->header.stamp;	

}

void PositionControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

    ROS_INFO_ONCE("PositionController got first odometry message.");

    if (waypointHasBeenPublished_ && enable_state_estimator_){

	    //This functions allows us to put the odometry message into the odometry variable--> _position,
 	    //_orientation,_velocit_body,_angular_velocity
	    EigenOdometry odometry;
	    eigenOdometryFromMsg(odometry_msg, &odometry);
	    position_controller_.SetOdometryWithStateEstimator(odometry);

    }

    if(waypointHasBeenPublished_){

      //This functions allows us to put the odometry message into the odometry variable--> _position,
      //_orientation,_velocit_body,_angular_velocity
      EigenOdometry odometry;
      eigenOdometryFromMsg(odometry_msg, &odometry);
      position_controller_.SetOdometryWithoutStateEstimator(odometry);

      Eigen::Vector4d ref_rotor_velocities;
      position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

      //creating a new mav message. actuator_msg is used to send the velocities of the propellers.
      mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

      //we use clear because we later want to be sure that we used the previously calculated velocity.
      actuator_msg->angular_velocities.clear();
      //for all propellers, we put them into actuator_msg so they will later be used to control the crazyflie.
      for (int i = 0; i < ref_rotor_velocities.size(); i++)
         actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
      actuator_msg->header.stamp = odometry_msg->header.stamp;

      motor_velocity_reference_pub_.publish(actuator_msg);

    }
   
}

// The attitude is estimated only if the waypoint has been published
void PositionControllerNode::CallbackAttitudeEstimation(const ros::TimerEvent& event){

    if (waypointHasBeenPublished_)
            position_controller_.CallbackAttitudeEstimation();

}

// The high level control is run only if the waypoint has been published
void PositionControllerNode::CallbackHightLevelControl(const ros::TimerEvent& event){

    if (waypointHasBeenPublished_)
            position_controller_.CallbackHightLevelControl();

}

// IMU messages are sent to the controller with a frequency of 500Hz. In other words, with a sampling time of 0.002 seconds
void PositionControllerNode::CallbackIMUUpdate(const ros::TimerEvent& event){

    position_controller_.SetSensorData(sensors_);

    ROS_INFO_ONCE("IMU Message sent to position controller");

    if (waypointHasBeenPublished_){

	    Eigen::Vector4d ref_rotor_velocities;
	    position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

	    // A new mav message, actuator_msg, is used to send to Gazebo the propellers angular velocities.
	    mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

	    // The clear method makes sure the actuator_msg is empty (there are no previous values of the propellers angular velocities).
	    actuator_msg->angular_velocities.clear();
	    // for all propellers, we put them into actuator_msg so they will later be used to control the crazyflie.
	    for (int i = 0; i < ref_rotor_velocities.size(); i++)
	       actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
	    actuator_msg->header.stamp = imu_msg_head_stamp_;
	    
	    motor_velocity_reference_pub_.publish(actuator_msg);

    }

}


}

int main(int argc, char** argv){
    ros::init(argc, argv, "position_controller_node_with_stateEstimator");
    
    ros::NodeHandle nh2;
    
    rotors_control::PositionControllerNode position_controller_node;

    ros::spin();

    return 0;
}

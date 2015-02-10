/*
 * common.h
 *
 *  Created on: Feb 10, 2015
 *      Author: burrimi
 */

#ifndef INCLUDE_ROTORS_CONTROL_COMMON_H_
#define INCLUDE_ROTORS_CONTROL_COMMON_H_

#include <nav_msgs/Odometry.h>
#include <mav_msgs/conversions.h>

namespace rotors_control {

// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultMotorVelocityReferencePubTopic = "motor_velocity_reference";
static const std::string kDefaultCommandTrajectoryTopic = "command/trajectory";
static const std::string kDefaultImuSubTopic = "imu";
static const std::string kDefaultOdometrySubTopic = "odometry";



struct EigenOdometry {
  EigenOdometry()
      : position(0.0, 0.0, 0.0),
        orientation(Eigen::Quaterniond::Identity()),
        velocity(0.0, 0.0, 0.0),
        angular_velocity(0.0, 0.0, 0.0) {};

  EigenOdometry(const Eigen::Vector3d& _position,
                         const Eigen::Quaterniond& _orientation,
                         const Eigen::Vector3d& _velocity,
                         const Eigen::Vector3d& _angular_velocity) {
    position = _position;
    orientation = _orientation;
    velocity = _velocity;
    angular_velocity = _angular_velocity;
  };

  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d velocity; // Velocity in expressed in the Body frame!
  Eigen::Vector3d angular_velocity;
};


void eigenOdometryFromMsg(const nav_msgs::OdometryConstPtr& msg,
                          EigenOdometry* odometry) {
  odometry->position = mav_msgs::vector3FromPointMsg(msg->pose.pose.position);
  odometry->orientation = mav_msgs::quaternionFromMsg(msg->pose.pose.orientation);
  odometry->velocity = mav_msgs::vector3FromMsg(msg->twist.twist.linear);
  odometry->angular_velocity = mav_msgs::vector3FromMsg(msg->twist.twist.angular);
}

}


#endif /* INCLUDE_ROTORS_CONTROL_COMMON_H_ */

/*
 * Copyright 2018 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2018 Emanuele Aucone, University of Sannio in Benevento, Italy
 * Copyright 2018 Benjamin Rodriguez, MIT, USA
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

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"
#include "rotors_control/common.h"

#include <nav_msgs/Odometry.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/conversions.h>

// Here I use global publisher and subscriber, since I want to access the
// publisher in the function MsgCallback:
ros::Publisher rpy_publisher;
ros::Subscriber quat_subscriber;

// Function for conversion of quaternion to roll pitch and yaw. The angles
// are published here too.
void MsgCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{

    EigenOdometry odometry;
    eigenOdometryFromMsg(odometry_msg, &odometry);

    // the incoming geometry_msgs::PoseStamped is transformed to a tf::Quaterion
    tf::Quaternion q(odometry.orientation.x(), odometry.orientation.y(), odometry.orientation.z(), odometry.orientation.w());
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    roll = -1 * roll;

    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll * 180.0 / M_PI;
    rpy.y = pitch * 180.0 / M_PI;
    rpy.z = yaw * 180.0 / M_PI;

    // this Vector is then published:
    rpy_publisher.publish(rpy);
    ROS_DEBUG("published rpy angles: roll=%f pitch=%f yaw=%f", rpy.x, rpy.y, rpy.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quaternion_to_rpy");
    ros::NodeHandle n;
    rpy_publisher = n.advertise<geometry_msgs::Vector3>("orientation_rpy", 1);
    quat_subscriber = n.subscribe("odometry_sensor1/odometry", 1, MsgCallback);

    // check for incoming quaternions untill ctrl+c is pressed
    ROS_DEBUG("waiting for quaternion");
    ros::spin();
    return 0;
}


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

#ifndef ROTORS_CONTROL_POSITION_CONTROLLER_NODE_H
#define ROTORS_CONTROL_POSITION_CONTROLLER_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "rotors_control/common.h"
#include "rotors_control/position_controller.h"

namespace rotors_control {

    class PositionControllerNode{
        public:
            PositionControllerNode();
            ~PositionControllerNode();
             
            void InitializeParams();
            void Publish();

        private:

            PositionController position_controller_;

            std::string namespace_;

            //subscribers
            ros::Subscriber cmd_trajectory_sub_;
            ros::Subscriber cmd_multi_dof_joint_trajectory_sub_;
            ros::Subscriber cmd_pose_sub_;
            ros::Subscriber odometry_sub_;

            //publisher
            ros::Publisher motor_velocity_reference_pub_;

            mav_msgs::EigenTrajectoryPointDeque commands_;
            std::deque<ros::Duration> command_waiting_times_;
            ros::Timer command_timer_;

            void TimedCommandCallback(const ros::TimerEvent& e);

            void MultiDofJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);

            void CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg);

            void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);

    };
}

#endif // ROTORS_CONTROL_POSITION_CONTROLLER_NODE_H

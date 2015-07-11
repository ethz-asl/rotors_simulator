/*
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
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

#ifndef ROTORS_GAZEBO_PLUGINS_SERVO_MOTOR_H
#define ROTORS_GAZEBO_PLUGINS_SERVO_MOTOR_H

#include <stdio.h>

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <manipulator_msgs/CommandPositionServoMotor.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include "rotors_gazebo_plugins/common.h"

namespace gazebo {
// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultCommandSubTopic = "command/servo_position";
static const double kDefaultKp = 10.0;
static const double kDefaultKd = 0.05;
static const double kDefaultKi = 0.3;
static const double kDefaultMaxAngle = 1e+16;
static const double kDefaultMinAngle = -1e+16;
static const double kDefaultMaxAngleErrorIntegral = 1.0;

class GazeboServoMotor : public ModelPlugin
{
 public:
  GazeboServoMotor()
      : ModelPlugin(),
        command_position_sub_topic_(kDefaultCommandSubTopic),
        kp_(kDefaultKp),
        kd_(kDefaultKd),
        ki_(kDefaultKi),
        max_angle_(kDefaultMaxAngle),
        min_angle_(kDefaultMinAngle),
        angle_error_integral_(0.0),
        max_angle_error_integral_(kDefaultMaxAngleErrorIntegral),
        received_first_command_(false)
  {
  }

  virtual ~GazeboServoMotor();

  virtual void InitializeParams();
  virtual void Publish();

 protected:
  virtual void UpdatePosition();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

 private:
  std::string command_position_sub_topic_;
  std::string joint_name_;
  std::string namespace_;
  std::string motor_model_;

  ros::NodeHandle* node_handle_;
  ros::Subscriber position_command_sub_;

  double sampling_time_;

  // motor parameters
  double max_torque_;
  double max_angle_error_integral_;
  double no_load_speed_;
  double max_angle_;
  double min_angle_;

  //controller parameters
  double kp_;
  double kd_;
  double ki_;

  bool received_first_command_;

  math::Angle angle_reference_;

  double angle_error_integral_;

  physics::ModelPtr model_;
  physics::JointPtr joint_;
  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread();
  void PositionCommandCallback(const manipulator_msgs::CommandPositionServoMotorConstPtr& msg);

};
}

#endif // ROTORS_GAZEBO_PLUGINS_SERVO_MOTOR_H

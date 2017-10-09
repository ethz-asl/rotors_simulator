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
#include <sensor_msgs/JointState.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "rotors_gazebo_plugins/common.h"

namespace gazebo {
// Default values
static const std::string kDefaultCommandSubTopic = "command/servo";
static const std::string kDefaultJointStatePubTopic = "joint_state";
static const double kDefaultKp = 10.0;
static const double kDefaultKd = 0.05;
static const double kDefaultKi = 0.3;
static const double kDefaultMaxAngle = 1e+16;
static const double kDefaultMinAngle = -1e+16;
static const double kDefaultMaxAngleErrorIntegral = 1.0;
static constexpr int kDefaultMeasurementDelay = 0;
static constexpr int kDefaultMeasurementDivisor = 1;
static constexpr int kDefaultGazeboSequence = 0;
static constexpr double kDefaultUnknownDelay = 0.0;


class GazeboServoMotor : public ModelPlugin
{
 public:
  typedef std::normal_distribution<> NormalDistribution;
  typedef std::uniform_real_distribution<> UniformDistribution;
  typedef std::deque<std::pair<int, sensor_msgs::JointState> > JointStateQueue;

  GazeboServoMotor()
      : ModelPlugin(),
        random_generator_(random_device_()),
        command_sub_topic_(kDefaultCommandSubTopic),
        joint_state_pub_topic_(kDefaultJointStatePubTopic),
        kp_(kDefaultKp),
        kd_(kDefaultKd),
        ki_(kDefaultKi),
        max_angle_(kDefaultMaxAngle),
        min_angle_(kDefaultMinAngle),
        measurement_delay_(kDefaultMeasurementDelay),
        measurement_divisor_(kDefaultMeasurementDivisor),
        unknown_delay_(kDefaultUnknownDelay),
        gazebo_sequence_(kDefaultGazeboSequence),
        angle_error_integral_(0.0),
        sampling_time_(0.0),
        no_load_speed_(0.0),
        max_torque_(0.0),
        max_angle_error_integral_(kDefaultMaxAngleErrorIntegral),
        received_first_command_(false),
        node_handle_(NULL) {}

  virtual ~GazeboServoMotor();

  virtual void InitializeParams();
  virtual void Publish();

 protected:
  virtual void RunController();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

 private:
  std::string command_sub_topic_;
  std::string joint_state_pub_topic_;
  std::string joint_name_;
  std::string namespace_;
  std::string motor_name_;

  ros::NodeHandle* node_handle_;
  ros::Subscriber command_sub_;
  ros::Publisher joint_state_pub_;

  double sampling_time_;

  std::random_device random_device_;
  std::mt19937 random_generator_;

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

  // measurements parameters
  int measurement_delay_;
  int measurement_divisor_;
  int gazebo_sequence_;
  int motor_sequence_;
  double unknown_delay_;

  NormalDistribution position_n_;
  NormalDistribution velocity_n_;
  NormalDistribution effort_n_;
  UniformDistribution position_u_;
  UniformDistribution velocity_u_;
  UniformDistribution effort_u_;

  JointStateQueue joint_state_queue_;

  bool received_first_command_;

  math::Angle angle_reference_;
  double angular_velocity_reference_;

  double angle_error_integral_;

  physics::ModelPtr model_;
  physics::JointPtr joint_;
  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread();
  void CommandCallback(const trajectory_msgs::JointTrajectoryPtr& msg);
};
}

#endif // ROTORS_GAZEBO_PLUGINS_SERVO_MOTOR_H

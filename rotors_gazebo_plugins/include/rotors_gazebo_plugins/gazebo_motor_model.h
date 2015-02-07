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


#ifndef ROTORS_GAZEBO_PLUGINS_MOTOR_MODELS_H
#define ROTORS_GAZEBO_PLUGINS_MOTOR_MODELS_H

#include <stdio.h>

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <rotors_model/motor_model.hpp>
#include <mav_msgs/MotorSpeed.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "rotors_gazebo_plugins/common.h"

namespace turning_direction {
const static int CCW = 1;
const static int CW = -1;
}

namespace gazebo {
// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultCommandSubTopic = "command/motors";
static const std::string kDefaultMotorVelocityPubTopic = "motor_velocity";

static constexpr double kDefaultMotorConstant = 8.54858e-06;
static constexpr double kDefaultMomentConstant = 0.016;
static constexpr double kDefaultTimeConstantUp = 1.0 / 80.0;
static constexpr double kDefaultTimeConstantDown = 1.0 / 40.0;
static constexpr double kDefaulMaxRotVelocity = 838.0;
static constexpr double kDefaultRotorDragCoefficient = 1.0e-4;
static constexpr double kDefaultRollingMomentCoefficient = 1.0e-6;
static constexpr double kDefaultRotorVelocitySlowdownSim = 10.0;



class GazeboMotorModel : public MotorModel, public ModelPlugin {
 public:
  GazeboMotorModel()
      : ModelPlugin(),
        MotorModel(),
        command_sub_topic_(),
        motor_velocity_pub_topic_(),
        motor_constant_(kDefaultMotorConstant),
        moment_constant_(kDefaultMomentConstant),
        time_constant_up_(kDefaultTimeConstantUp),
        time_constant_down_(kDefaultTimeConstantDown),
        max_rot_velocity_(kDefaulMaxRotVelocity),
        rotor_drag_coefficient_(kDefaultRotorDragCoefficient),
        rolling_moment_coefficient_(kDefaultRollingMomentCoefficient),
        rotor_velocity_slowdown_sim_(kDefaultRotorVelocitySlowdownSim),
        node_handle_(NULL) {
    InitGlogHelper::instance().initGlog();
  }

  virtual ~GazeboMotorModel();

  virtual void InitializeParams();
  virtual void Publish();

 protected:
  virtual void UpdateForcesAndMoments();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

 private:
  std::string namespace_;
  std::string joint_name_;
  std::string link_name_;
  std::string command_sub_topic_;
  std::string motor_velocity_pub_topic_;
  int motor_number_;
  int turning_direction_;
  double motor_constant_;
  double moment_constant_;
  double time_constant_up_;
  double time_constant_down_;
  double max_rot_velocity_;
  double viscous_friction_coefficient_;
  double inertia_;
  double rotor_drag_coefficient_;
  double rolling_moment_coefficient_;
  double rotor_velocity_slowdown_sim_;

  ros::NodeHandle* node_handle_;
  ros::Publisher motor_velocity_pub_;
  ros::Subscriber command_sub_;

  physics::ModelPtr model_;
  physics::JointPtr joint_;
  physics::LinkPtr link_;
  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread();
  std_msgs::Float32 turning_velocity_msg_;
  void VelocityCallback(const mav_msgs::MotorSpeedPtr& rot_velocities);
  std::unique_ptr<FirstOrderFilter<double>>  rotor_velocity_filter_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_MOTOR_MODELS_H

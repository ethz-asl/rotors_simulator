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

// SYSTEM
#include <stdio.h>

// 3RD PARTY
#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <mav_msgs/default_topics.h>  // This comes from the mav_comm repo

// USER
#include "rotors_gazebo_plugins/common.h"
#include "rotors_gazebo_plugins/motor_model.hpp"
#include "Float32.pb.h"
#include "CommandMotorSpeed.pb.h"
#include "WindSpeed.pb.h"

namespace turning_direction {
const static int CCW = 1;
const static int CW = -1;
} // namespace turning_direction

namespace gazebo {

// Default values
//static const std::string kDefaultCommandSubTopic = "gazebo/command/motor_speed";
//static const std::string kDefaultWindSpeedSubTopic = "gazebo/wind_speed";

typedef const boost::shared_ptr<const gz_mav_msgs::CommandMotorSpeed> GzCommandMotorSpeedMsgPtr;
typedef const boost::shared_ptr<const gz_mav_msgs::WindSpeed> GzWindSpeedMsgPtr;

// Set the max_force_ to the max double value. The limitations get handled by the FirstOrderFilter.
static constexpr double kDefaultMaxForce = std::numeric_limits<double>::max();
static constexpr double kDefaultMotorConstant = 8.54858e-06;
static constexpr double kDefaultMomentConstant = 0.016;
static constexpr double kDefaultTimeConstantUp = 1.0 / 80.0;
static constexpr double kDefaultTimeConstantDown = 1.0 / 40.0;
static constexpr double kDefaulMaxRotVelocity = 838.0;
static constexpr double kDefaultRotorDragCoefficient = 1.0e-4;
static constexpr double kDefaultRollingMomentCoefficient = 1.0e-6;

class GazeboMotorModel : public MotorModel, public ModelPlugin {

 public:
  GazeboMotorModel()
      : ModelPlugin(),
        MotorModel(),
        command_sub_topic_(mav_msgs::default_topics::COMMAND_ACTUATORS),
        wind_speed_sub_topic_(mav_msgs::default_topics::WIND_SPEED),
        motor_speed_pub_topic_(mav_msgs::default_topics::MOTOR_MEASUREMENT),
        motor_number_(0),
        turning_direction_(turning_direction::CW),
        max_force_(kDefaultMaxForce),
        max_rot_velocity_(kDefaulMaxRotVelocity),
        moment_constant_(kDefaultMomentConstant),
        motor_constant_(kDefaultMotorConstant),
        ref_motor_rot_vel_(0.0),
        rolling_moment_coefficient_(kDefaultRollingMomentCoefficient),
        rotor_drag_coefficient_(kDefaultRotorDragCoefficient),
        rotor_velocity_slowdown_sim_(kDefaultRotorVelocitySlowdownSim),
        time_constant_down_(kDefaultTimeConstantDown),
        time_constant_up_(kDefaultTimeConstantUp),
        node_handle_(nullptr),
        wind_speed_W_(0, 0, 0),
        pubs_and_subs_created_(false) {}

  virtual ~GazeboMotorModel();

  virtual void InitializeParams();
  virtual void Publish();

 protected:
  virtual void UpdateForcesAndMoments();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

 private:

  /// \brief    Flag that is set to true once CreatePubsAndSubs() is called, used
  ///           to prevent CreatePubsAndSubs() from be called on every OnUpdate().
  bool pubs_and_subs_created_;

  /// \brief    Creates all required publishers and subscribers, incl. routing of messages to/from ROS if required.
  /// \details  Call this once the first time OnUpdate() is called (can't
  ///           be called from Load() because there is no guarantee GazeboRosInterfacePlugin has
  ///           has loaded and listening to ConnectGazeboToRosTopic and ConnectRosToGazeboTopic messages).
  void CreatePubsAndSubs();

  std::string command_sub_topic_;
  std::string wind_speed_sub_topic_;
  std::string joint_name_;
  std::string link_name_;
  std::string motor_speed_pub_topic_;
  std::string namespace_;

  int motor_number_;
  int turning_direction_;

  double max_force_;
  double max_rot_velocity_;
  double moment_constant_;
  double motor_constant_;
  double ref_motor_rot_vel_;
  double rolling_moment_coefficient_;
  double rotor_drag_coefficient_;
  double rotor_velocity_slowdown_sim_;
  double time_constant_down_;
  double time_constant_up_;

  gazebo::transport::NodePtr node_handle_;

  gazebo::transport::PublisherPtr motor_velocity_pub_;

  gazebo::transport::SubscriberPtr command_sub_;

  gazebo::transport::SubscriberPtr wind_speed_sub_;

  physics::ModelPtr model_;
  physics::JointPtr joint_;
  physics::LinkPtr link_;

  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;

  void QueueThread();

  gz_std_msgs::Float32 turning_velocity_msg_;

  void ControlVelocityCallback(GzCommandMotorSpeedMsgPtr& command_motor_speed_msg);

  void WindSpeedCallback(GzWindSpeedMsgPtr& wind_speed_msg);

  std::unique_ptr<FirstOrderFilter<double>> rotor_velocity_filter_;
  math::Vector3 wind_speed_W_;
};

} // namespace gazebo {

#endif // ROTORS_GAZEBO_PLUGINS_MOTOR_MODELS_H

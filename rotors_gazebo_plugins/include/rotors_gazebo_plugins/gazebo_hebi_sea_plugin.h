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

#ifndef ROTORS_GAZEBO_PLUGINS_HEBI_SEA_H
#define ROTORS_GAZEBO_PLUGINS_HEBI_SEA_H

#include <cmath>
#include <deque>
#include <random>
#include <stdio.h>

#include <rotors_gazebo_plugins/common.h>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <limits>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {
// Default values
static const std::string kDefaultCommandSubTopic = "command/trajectory";
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
static constexpr double kDefaultVoltage = 24.0;
static constexpr double kDefaultStatorResistance = 2.0;
static constexpr double kDefaultMotorConstant = 2.9;  //torque = Km*I

enum HebiControlStrategies
{
  Off,
  Strategy1,
  Strategy2,
  Strategy3,
  Strategy4
};

typedef struct HebiSEAControlParameters
{
  HebiSEAControlParameters()
      : k_p_(1),
        k_d_(0),
        k_i_(0),
        feed_forward_(0),
        deadzone_(0),
        I_clamp_(std::numeric_limits<double>::infinity()),
        punch_(0),
        min_target_(-std::numeric_limits<double>::infinity()),
        max_target_(std::numeric_limits<double>::infinity()),
        min_output_(-1),
        max_output_(1),
        target_lp_(1),
        output_lp_(1),
        D_on_error_(true)
  {

  }

  double k_p_;
  double k_d_;
  double k_i_;
  double feed_forward_;
  double deadzone_;
  double I_clamp_;
  double punch_;
  double min_target_;
  double max_target_;
  double min_output_;
  double max_output_;
  double target_lp_;
  double output_lp_;
  bool D_on_error_;

} HebiSEAControlParameters;

typedef struct HebiSEAParameters
{

  HebiSEAParameters()
      : control_strategy_(HebiControlStrategies::Off)
  {
  }
  HebiSEAControlParameters position_control_parameters_;
  HebiSEAControlParameters velocity_control_parameters_;
  HebiSEAControlParameters effort_control_parameters_;
  HebiControlStrategies control_strategy_;

} HebiSEAParameters;

class PIDController
{
 public:
  PIDController()
      : previous_error_(0.0),
        error_integral_(0.0),
        previous_measurment_(0.0),
        first_call_(true),
        parameters_set_(false)
  {

  }

  void setParameters(const HebiSEAControlParameters &parameters)
  {
    parameters_ = parameters;
    parameters_set_ = true;
  }

  double run(double dt, double target, double measurement)
  {
    if (!parameters_set_) {
      return 0;
    }

    if (first_call_) {
      previous_measurment_ = measurement;
      previous_target_ = target;
      previous_output_ = 0.0;
      first_call_ = false;
    }

    target = parameters_.target_lp_ * target + (1 - parameters_.target_lp_) * previous_target_;
    target = limit(target, parameters_.max_target_, parameters_.min_target_);
    double error = target - measurement;
    // Proportional term
    double Pout = parameters_.k_p_ * error;
    // Integral term
    error_integral_ += error * dt;
    double Iout = parameters_.k_i_ * error_integral_;
    // Derivative term
    double derivative = 0.0;
    if (parameters_.D_on_error_ == true) {
      derivative = (error - previous_error_) / dt;
    } else {
      derivative = (measurement - previous_measurment_) / dt;
      // std::cout << "derivative: " << derivative << std::endl;
    }
    derivative = limit(derivative, 100.0, -100.0);
    double Dout = parameters_.k_d_ * derivative;

    double output = Pout + Iout + Dout;
    output = parameters_.output_lp_ * output + (1 - parameters_.output_lp_) * previous_output_;
    output = limit(output, parameters_.max_output_, parameters_.min_output_);
    previous_error_ = error;
    previous_measurment_ = measurement;
    previous_target_ = target;
    previous_output_ = output;

    return output;
  }
 private:
  HebiSEAControlParameters parameters_;
  double previous_error_;
  double error_integral_;
  double previous_measurment_;
  bool first_call_;
  bool parameters_set_;
  double previous_target_;
  double previous_output_;
};

class GazeboHebiSEA : public ModelPlugin
{
 public:
  typedef std::normal_distribution<> NormalDistribution;
  typedef std::uniform_real_distribution<> UniformDistribution;
  typedef std::deque<std::pair<int, sensor_msgs::JointState> > JointStateQueue;

  GazeboHebiSEA()
      : ModelPlugin(),
        random_generator_(random_device_()),
        command_sub_topic_(kDefaultCommandSubTopic),
        joint_state_pub_topic_(kDefaultJointStatePubTopic),
        measurement_delay_(kDefaultMeasurementDelay),
        measurement_divisor_(kDefaultMeasurementDivisor),
        unknown_delay_(kDefaultUnknownDelay),
        motor_sequence_(0),
        position_reference_(0),
        velocity_reference_(0),
        effort_reference_(0),
        gazebo_sequence_(kDefaultGazeboSequence),
        angle_error_integral_(0.0),
        sampling_time_(0.0),
        received_first_command_(false),
        node_handle_(NULL)
  {
  }

  virtual ~GazeboHebiSEA();

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
  HebiSEAParameters actuator_parameters_;

  ros::NodeHandle* node_handle_;
  ros::Subscriber command_sub_;
  ros::Publisher joint_state_pub_;

  double sampling_time_;

  std::random_device random_device_;
  std::mt19937 random_generator_;

  //controllers
  PIDController position_pid_;
  PIDController velocity_pid_;
  PIDController effort_pid_;

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

  ignition::math::Angle position_reference_;
  double velocity_reference_;
  double effort_reference_;

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

#endif // ROTORS_GAZEBO_PLUGINS_HEBI_SEA_H

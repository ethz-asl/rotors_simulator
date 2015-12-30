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

#include "rotors_gazebo_plugins/gazebo_servo_motor_plugin.h"
#include <chrono>

namespace gazebo {

GazeboServoMotor::~GazeboServoMotor()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}


void GazeboServoMotor::InitializeParams()
{
}


void GazeboServoMotor::Publish()
{
  sensor_msgs::JointStatePtr joint_state(new sensor_msgs::JointState);

  // Get latest joint state
  joint_state->header = joint_state_queue_.front().second.header;
  joint_state->name = joint_state_queue_.front().second.name;
  joint_state->position = joint_state_queue_.front().second.position;
  joint_state->velocity = joint_state_queue_.front().second.velocity;
  joint_state->effort = joint_state_queue_.front().second.effort;

  joint_state_queue_.pop_front();

  // Apply distortions
  joint_state->position[0] += position_n_(random_generator_) + position_u_(random_generator_);
  joint_state->velocity[0] += velocity_n_(random_generator_) + velocity_u_(random_generator_);
  joint_state->effort[0] += effort_n_(random_generator_) + effort_u_(random_generator_);

  if (joint_state_pub_.getNumSubscribers() > 0)
    joint_state_pub_.publish(joint_state);
}


void GazeboServoMotor::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  double noise_normal_angle;
  double noise_normal_angular_velocity;
  double noise_normal_torque;
  double noise_uniform_angle;
  double noise_uniform_angular_velocity;
  double noise_uniform_torque;

  model_ = _model;

  namespace_.clear();

  joint_state_queue_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_servo_motor] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("jointName"))
    joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
  else
    gzerr << "[gazebo_servo_motor] Please specify a jointName, where the rotor is attached.\n";
  // Get the pointer to the joint.
  joint_ = model_->GetJoint(joint_name_);
  if (joint_ == NULL)
    gzthrow("[gazebo_servo_motor] Couldn't find specified joint \"" << joint_name_ << "\".");

  if (_sdf->HasElement("motorModel"))
    motor_model_ = _sdf->GetElement("motorModel")->Get<std::string>();
  else
    gzerr << "[gazebo_servo_motor] Please specify a motorModel.\n";

  if (_sdf->HasElement("maxTorque"))
    max_torque_ = _sdf->GetElement("maxTorque")->Get<double>();
  else
    gzerr << "[gazebo_servo_motor] Please specify a maxTorque.\n";

  if (_sdf->HasElement("noLoadSpeed"))
    no_load_speed_ = _sdf->GetElement("noLoadSpeed")->Get<double>();
  else
    gzerr << "[gazebo_servo_motor] Please specify a noLoadSpeed.\n";

  if (_sdf->HasElement("randomEngineSeed")) {
    random_generator_.seed(_sdf->GetElement("randomEngineSeed")->Get<unsigned int>());
  }
  else {
    random_generator_.seed(std::chrono::system_clock::now().time_since_epoch().count());
  }

  getSdfParam<double>(_sdf, "maxAngleErrorIntegral", max_angle_error_integral_, max_angle_error_integral_);
  getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic_, command_sub_topic_);
  getSdfParam<std::string>(_sdf, "jointStatePubTopic", joint_state_pub_topic_, joint_state_pub_topic_);
  getSdfParam<double>(_sdf, "Kp", kp_, kp_);
  getSdfParam<double>(_sdf, "Kd", kd_, kd_);
  getSdfParam<double>(_sdf, "Ki", ki_, ki_);
  getSdfParam<double>(_sdf, "noiseNormalAngle", noise_normal_angle, 0.0);
  getSdfParam<double>(_sdf, "noiseNormalAngularVelocity", noise_normal_angular_velocity, 0.0);
  getSdfParam<double>(_sdf, "noiseNormalTorque", noise_normal_torque, 0.0);
  getSdfParam<double>(_sdf, "noiseUniformAngle", noise_uniform_angle, 0.0);
  getSdfParam<double>(_sdf, "noiseUniformAngularVelocity", noise_uniform_angular_velocity, 0.0);
  getSdfParam<double>(_sdf, "noiseUniformTorque", noise_uniform_torque, 0.0);
  getSdfParam<int>(_sdf, "measurementDelay", measurement_delay_, measurement_delay_);
  getSdfParam<int>(_sdf, "measurementDivisor", measurement_divisor_, measurement_divisor_);
  getSdfParam<double>(_sdf, "unknownDelay", unknown_delay_, unknown_delay_);
  getSdfParam<double>(_sdf, "maxAngle", max_angle_, max_angle_);
  getSdfParam<double>(_sdf, "minAngle", min_angle_, min_angle_);

  position_n_ = NormalDistribution(0, noise_normal_angle);
  velocity_n_ = NormalDistribution(0, noise_normal_angular_velocity);
  effort_n_ = NormalDistribution(0, noise_normal_torque);

  position_u_ = UniformDistribution(-noise_uniform_angle, noise_uniform_angle);
  velocity_u_ = UniformDistribution(-noise_uniform_angular_velocity, noise_uniform_angular_velocity);
  effort_u_ = UniformDistribution(-noise_uniform_torque, noise_uniform_torque);

  joint_->SetLowerLimit(0, min_angle_);
  joint_->SetUpperLimit(0, max_angle_);

  // Listen to the update event. This event is broadcast every simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboServoMotor::OnUpdate, this, _1));

  command_position_sub_topic_ = command_sub_topic_+"_position";
  command_torque_sub_topic_ = command_sub_topic_+"_torque";

  position_command_sub_ = node_handle_->subscribe(command_position_sub_topic_, 100,
                                                  &GazeboServoMotor::PositionCommandCallback, this);
  torque_command_sub_ = node_handle_->subscribe(command_torque_sub_topic_, 100,
                                                  &GazeboServoMotor::TorqueCommandCallback, this);

  joint_state_pub_ = node_handle_->advertise<sensor_msgs::JointState>(joint_state_pub_topic_, 10);
}


// This gets called by the world update start event.
void GazeboServoMotor::OnUpdate(const common::UpdateInfo& _info)
{
  static double prev_sim_time = _info.simTime.Double();
  sampling_time_ = _info.simTime.Double() - prev_sim_time;
  prev_sim_time = _info.simTime.Double();
  sampling_time_ = limit(sampling_time_, 1.0, 0.001);

  if (received_first_command_)
    if (position_control_)
      UpdatePosition();
    else
      UpdateTorque();

  if (gazebo_sequence_ % measurement_divisor_ == 0) {
    // Get the current simulation time.
    common::Time now = model_->GetWorld()->GetSimTime();
    sensor_msgs::JointState joint_state;

    joint_state.header.frame_id  = joint_->GetParent()->GetScopedName();
    joint_state.header.seq = motor_sequence_++;
    joint_state.header.stamp.sec = now.sec + ros::Duration(unknown_delay_).sec;
    joint_state.header.stamp.nsec = now.nsec + ros::Duration(unknown_delay_).nsec;
    joint_state.name.push_back(joint_name_);
    joint_state.position.push_back(joint_->GetAngle(0).Radian());
    joint_state.velocity.push_back(joint_->GetVelocity(0));
    joint_state.effort.push_back(joint_->GetForce(0));

    joint_state_queue_.push_back(std::make_pair(gazebo_sequence_ + measurement_delay_, joint_state));
  }

  // Is it time to publish the front element?
  if (gazebo_sequence_ == joint_state_queue_.front().first) {
    Publish();
  }

  ++gazebo_sequence_;
}


void GazeboServoMotor::PositionCommandCallback(
    const manipulator_msgs::CommandPositionServoMotorConstPtr& msg)
{
  double ref = msg->motor_angle;
  angle_reference_.SetFromRadian(ref);
  received_first_command_ = true;
  position_control_ = true;
}


void GazeboServoMotor::TorqueCommandCallback(
    const manipulator_msgs::CommandTorqueServoMotorConstPtr& msg)
{
  torque_reference_ = msg->torque;
  received_first_command_ = true;
  position_control_ = false;
}


void GazeboServoMotor::UpdatePosition()
{
  double angle_error = (angle_reference_ - joint_->GetAngle(0)).Radian();
  double omega = joint_->GetVelocity(0);
  angle_error_integral_ += angle_error * sampling_time_;
  angle_error_integral_ = limit(angle_error_integral_, max_angle_error_integral_,
                                -max_angle_error_integral_);
  double torque = kp_ * angle_error - kd_ * omega + ki_ * angle_error_integral_;
  torque = limit(torque, max_torque_, -max_torque_);

  ///debug
//  std::cout << "joint name = " << joint_->GetName() << std::endl;
//  std::cout << "reference = " << angle_reference_.Degree() << std::endl;
//  std::cout << "angle = " << joint_->GetAngle(0).Degree() << std::endl;
//  std::cout << "torque = \t" << torque <<
//      "\nangle error = \t" << angle_error <<
//      "\nomega = \t" << omega <<
//      "\nintegral_error = \t" << angle_error_integral_ << std::endl;
//  std::cout << "===============================" << std::endl;

  joint_->SetForce(0, torque);
}


void GazeboServoMotor::UpdateTorque()
{
  double torque = limit(torque_reference_, max_torque_, -max_torque_);
  joint_->SetForce(0, torque);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboServoMotor);
}

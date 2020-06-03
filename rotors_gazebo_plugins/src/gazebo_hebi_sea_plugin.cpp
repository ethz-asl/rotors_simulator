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

#include "rotors_gazebo_plugins/gazebo_hebi_sea_plugin.h"
#include <chrono>

namespace gazebo {

GazeboHebiSEA::~GazeboHebiSEA()
{
  updateConnection_.reset();
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboHebiSEA::InitializeParams()
{
}

void GazeboHebiSEA::Publish()
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

void GazeboHebiSEA::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  double noise_normal_angle;
  double noise_normal_velocity;
  double noise_normal_torque;
  double noise_uniform_angle;
  double noise_uniform_velocity;
  double noise_uniform_torque;

  model_ = _model;

  namespace_.clear();

  joint_state_queue_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_hebi_sea] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("jointName"))
    joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
  else
    gzerr << "[gazebo_hebi_sea] Please specify a jointName, where the rotor is attached.\n";
  // Get the pointer to the joint.
  joint_ = model_->GetJoint(joint_name_);
  if (joint_ == NULL)
    gzthrow("[gazebo_hebi_sea] Couldn't find specified joint \"" << joint_name_ << "\".");

  if (_sdf->HasElement("motorName"))
    motor_name_ = _sdf->GetElement("motorName")->Get<std::string>();
  else
    gzerr << "[gazebo_hebi_sea] Please specify a motorModel.\n";

  if (_sdf->HasElement("randomEngineSeed")) {
    random_generator_.seed(_sdf->GetElement("randomEngineSeed")->Get<unsigned int>());
  } else {
    random_generator_.seed(std::chrono::system_clock::now().time_since_epoch().count());
  }

  //control strategy
  if (_sdf->HasElement("controlStrategy")) {
    actuator_parameters_.control_strategy_ = (HebiControlStrategies) _sdf->GetElement(
        "controlStrategy")->Get<unsigned int>();
  } else {
    actuator_parameters_.control_strategy_ = HebiControlStrategies::Strategy3;
  }

  //position controller parameters
  getSdfParam<double>(_sdf, "position_kp", actuator_parameters_.position_control_parameters_.k_p_,
                      actuator_parameters_.position_control_parameters_.k_p_);
  getSdfParam<double>(_sdf, "position_kd", actuator_parameters_.position_control_parameters_.k_d_,
                      actuator_parameters_.position_control_parameters_.k_d_);
  getSdfParam<double>(_sdf, "position_ki", actuator_parameters_.position_control_parameters_.k_i_,
                      actuator_parameters_.position_control_parameters_.k_i_);
  getSdfParam<double>(_sdf, "position_feed_forward",
                      actuator_parameters_.position_control_parameters_.feed_forward_,
                      actuator_parameters_.position_control_parameters_.feed_forward_);
  getSdfParam<double>(_sdf, "position_min_target",
                      actuator_parameters_.position_control_parameters_.min_target_,
                      actuator_parameters_.position_control_parameters_.min_target_);
  getSdfParam<double>(_sdf, "position_max_target",
                      actuator_parameters_.position_control_parameters_.max_target_,
                      actuator_parameters_.position_control_parameters_.max_target_);
  getSdfParam<double>(_sdf, "position_min_output",
                      actuator_parameters_.position_control_parameters_.min_output_,
                      actuator_parameters_.position_control_parameters_.min_output_);
  getSdfParam<double>(_sdf, "position_max_output",
                      actuator_parameters_.position_control_parameters_.max_output_,
                      actuator_parameters_.position_control_parameters_.max_output_);
  getSdfParam<double>(_sdf, "position_target_lp",
                      actuator_parameters_.position_control_parameters_.target_lp_,
                      actuator_parameters_.position_control_parameters_.target_lp_);
  getSdfParam<double>(_sdf, "position_output_lp",
                      actuator_parameters_.position_control_parameters_.output_lp_,
                      actuator_parameters_.position_control_parameters_.output_lp_);
  getSdfParam<bool>(_sdf, "position_D_on_error",
                      actuator_parameters_.position_control_parameters_.D_on_error_,
                      actuator_parameters_.position_control_parameters_.D_on_error_);

  //velocity controller parameters
  getSdfParam<double>(_sdf, "velocity_kp", actuator_parameters_.velocity_control_parameters_.k_p_,
                      actuator_parameters_.velocity_control_parameters_.k_p_);
  getSdfParam<double>(_sdf, "velocity_kd", actuator_parameters_.velocity_control_parameters_.k_d_,
                      actuator_parameters_.velocity_control_parameters_.k_d_);
  getSdfParam<double>(_sdf, "velocity_ki", actuator_parameters_.velocity_control_parameters_.k_i_,
                      actuator_parameters_.velocity_control_parameters_.k_i_);
  getSdfParam<double>(_sdf, "velocity_feed_forward",
                      actuator_parameters_.velocity_control_parameters_.feed_forward_,
                      actuator_parameters_.velocity_control_parameters_.feed_forward_);
  getSdfParam<double>(_sdf, "velocity_min_target",
                      actuator_parameters_.velocity_control_parameters_.min_target_,
                      actuator_parameters_.velocity_control_parameters_.min_target_);
  getSdfParam<double>(_sdf, "velocity_max_target",
                      actuator_parameters_.velocity_control_parameters_.max_target_,
                      actuator_parameters_.velocity_control_parameters_.max_target_);
  getSdfParam<double>(_sdf, "velocity_min_output",
                      actuator_parameters_.velocity_control_parameters_.min_output_,
                      actuator_parameters_.velocity_control_parameters_.min_output_);
  getSdfParam<double>(_sdf, "velocity_max_output",
                      actuator_parameters_.velocity_control_parameters_.max_output_,
                      actuator_parameters_.velocity_control_parameters_.max_output_);
  getSdfParam<double>(_sdf, "velocity_target_lp",
                      actuator_parameters_.velocity_control_parameters_.target_lp_,
                      actuator_parameters_.velocity_control_parameters_.target_lp_);
  getSdfParam<double>(_sdf, "velocity_output_lp",
                      actuator_parameters_.velocity_control_parameters_.output_lp_,
                      actuator_parameters_.velocity_control_parameters_.output_lp_);
  getSdfParam<bool>(_sdf, "velocity_D_on_error",
                      actuator_parameters_.velocity_control_parameters_.D_on_error_,
                      actuator_parameters_.velocity_control_parameters_.D_on_error_);

  //effort controller parameters
  getSdfParam<double>(_sdf, "effort_kp", actuator_parameters_.effort_control_parameters_.k_p_,
                      actuator_parameters_.effort_control_parameters_.k_p_);
  getSdfParam<double>(_sdf, "effort_kd", actuator_parameters_.effort_control_parameters_.k_d_,
                      actuator_parameters_.effort_control_parameters_.k_d_);
  getSdfParam<double>(_sdf, "effort_ki", actuator_parameters_.effort_control_parameters_.k_i_,
                      actuator_parameters_.effort_control_parameters_.k_i_);
  getSdfParam<double>(_sdf, "effort_feed_forward",
                      actuator_parameters_.effort_control_parameters_.feed_forward_,
                      actuator_parameters_.effort_control_parameters_.feed_forward_);
  getSdfParam<double>(_sdf, "effort_min_target",
                      actuator_parameters_.effort_control_parameters_.min_target_,
                      actuator_parameters_.effort_control_parameters_.min_target_);
  getSdfParam<double>(_sdf, "effort_max_target",
                      actuator_parameters_.effort_control_parameters_.max_target_,
                      actuator_parameters_.effort_control_parameters_.max_target_);
  getSdfParam<double>(_sdf, "effort_min_output",
                      actuator_parameters_.effort_control_parameters_.min_output_,
                      actuator_parameters_.effort_control_parameters_.min_output_);
  getSdfParam<double>(_sdf, "effort_max_output",
                      actuator_parameters_.effort_control_parameters_.max_output_,
                      actuator_parameters_.effort_control_parameters_.max_output_);
  getSdfParam<double>(_sdf, "effort_target_lp",
                      actuator_parameters_.effort_control_parameters_.target_lp_,
                      actuator_parameters_.effort_control_parameters_.target_lp_);
  getSdfParam<double>(_sdf, "effort_output_lp",
                      actuator_parameters_.effort_control_parameters_.output_lp_,
                      actuator_parameters_.effort_control_parameters_.output_lp_);
  getSdfParam<bool>(_sdf, "effort_D_on_error",
                      actuator_parameters_.effort_control_parameters_.D_on_error_,
                      actuator_parameters_.effort_control_parameters_.D_on_error_);

  position_pid_.setParameters(actuator_parameters_.position_control_parameters_);
  velocity_pid_.setParameters(actuator_parameters_.velocity_control_parameters_);
  effort_pid_.setParameters(actuator_parameters_.effort_control_parameters_);

  getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic_, command_sub_topic_);
  getSdfParam<std::string>(_sdf, "jointStatePubTopic", joint_state_pub_topic_,
                           joint_state_pub_topic_);
  // erase all dashes from topic name
  joint_state_pub_topic_.erase(
      remove(joint_state_pub_topic_.begin(), joint_state_pub_topic_.end(), '-'),
      joint_state_pub_topic_.end() );

  getSdfParam<double>(_sdf, "noiseNormalAngle", noise_normal_angle, 0.0);
  getSdfParam<double>(_sdf, "noiseNormalAngularVelocity", noise_normal_velocity, 0.0);
  getSdfParam<double>(_sdf, "noiseNormalTorque", noise_normal_torque, 0.0);
  getSdfParam<double>(_sdf, "noiseUniformAngle", noise_uniform_angle, 0.0);
  getSdfParam<double>(_sdf, "noiseUniformAngularVelocity", noise_uniform_velocity, 0.0);
  getSdfParam<double>(_sdf, "noiseUniformTorque", noise_uniform_torque, 0.0);
  getSdfParam<int>(_sdf, "measurementDelay", measurement_delay_, measurement_delay_);
  getSdfParam<int>(_sdf, "measurementDivisor", measurement_divisor_, measurement_divisor_);
  getSdfParam<double>(_sdf, "unknownDelay", unknown_delay_, unknown_delay_);

  position_n_ = NormalDistribution(0, noise_normal_angle);
  velocity_n_ = NormalDistribution(0, noise_normal_velocity);
  effort_n_ = NormalDistribution(0, noise_normal_torque);

  position_u_ = UniformDistribution(-noise_uniform_angle, noise_uniform_angle);
  velocity_u_ = UniformDistribution(-noise_uniform_velocity, noise_uniform_velocity);
  effort_u_ = UniformDistribution(-noise_uniform_torque, noise_uniform_torque);

  // Listen to the update event. This event is broadcast every simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboHebiSEA::OnUpdate, this, _1));

  command_sub_ = node_handle_->subscribe(command_sub_topic_, 10, &GazeboHebiSEA::CommandCallback,
                                         this);

  joint_state_pub_ = node_handle_->advertise<sensor_msgs::JointState>(joint_state_pub_topic_, 10);
}

// This gets called by the world update start event.
void GazeboHebiSEA::OnUpdate(const common::UpdateInfo& _info)
{
  static double prev_sim_time = _info.simTime.Double();
  sampling_time_ = _info.simTime.Double() - prev_sim_time;
  prev_sim_time = _info.simTime.Double();
  sampling_time_ = limit(sampling_time_, 1.0, 0.001);

  if (received_first_command_) {
    RunController();
  }

  if (gazebo_sequence_ % measurement_divisor_ == 0) {
    // Get the current simulation time.
    common::Time now = model_->GetWorld()->SimTime();
    sensor_msgs::JointState joint_state;

    joint_state.header.frame_id = joint_->GetParent()->GetScopedName();
    joint_state.header.seq = motor_sequence_++;
    joint_state.header.stamp.sec = now.sec + ros::Duration(unknown_delay_).sec;
    joint_state.header.stamp.nsec = now.nsec + ros::Duration(unknown_delay_).nsec;
    joint_state.name.push_back(motor_name_);
    joint_state.position.push_back(joint_->Position(0));
    joint_state.velocity.push_back(joint_->GetVelocity(0));
    joint_state.effort.push_back(joint_->GetForce(0));

    joint_state_queue_.push_back(
        std::make_pair(gazebo_sequence_ + measurement_delay_, joint_state));
  }

  // Is it time to publish the front element?
  if (gazebo_sequence_ == joint_state_queue_.front().first) {
    Publish();
  }

  ++gazebo_sequence_;
}

void GazeboHebiSEA::CommandCallback(const trajectory_msgs::JointTrajectoryPtr& msg)
{
  for (int i = 0; i < msg->joint_names.size(); i++) {
    if (msg->joint_names.at(i) == motor_name_) {
      if (msg->points.size() < 1) {
        gzerr << "[gazebo_hebi_sea: " << motor_name_
              << "] Number of points in command joint trajectory is wrong.\n";
        return;
      } else if (msg->points.size() > 1) {
        gzwarn << "[gazebo_hebi_sea: " << motor_name_
               << "] Number of points referenecs is > 1, will ignore the rest of queue\n.";
      }

      if (msg->points.at(0).positions.size() > i) {
        position_reference_ = msg->points.at(0).positions.at(i);
        received_first_command_ = true;
      } else {
        position_reference_ = joint_->Position(0);
      }
      if (msg->points.at(0).velocities.size() > i) {
        velocity_reference_ = msg->points.at(0).velocities.at(i);
        received_first_command_ = true;
      } else {
        velocity_reference_ = joint_->GetVelocity(0);
      }
      if (msg->points.at(0).effort.size() > i) {
        effort_reference_ = msg->points.at(0).effort.at(i);
        received_first_command_ = true;
      } else {
        effort_reference_ = joint_->GetForce(0);
      }
      return;
    }
  }
}

void GazeboHebiSEA::RunController()
{
  double output_pwm_ = 0.0;
  double current_position = joint_->Position(0);
  double current_velocity = joint_->GetVelocity(0);
  double current_effort = joint_->GetForce(0);

  switch (actuator_parameters_.control_strategy_) {
    case (HebiControlStrategies::Off): {
      output_pwm_ = 0.0;
      break;
    }
    case (HebiControlStrategies::Strategy1): {
      gzwarn << "[gazebo_hebi_sea: " << joint_name_
             << "] PWM control strategy is not supported at the moment\n.";
      break;
    }
    case (HebiControlStrategies::Strategy2): {
      double effort_position_loop = position_pid_.run(sampling_time_, position_reference_.Radian(),
                                                      current_position);
      double effort_velocity_loop = velocity_pid_.run(sampling_time_, velocity_reference_,
                                                      current_velocity);
      double total_effort = effort_position_loop + effort_velocity_loop + effort_reference_;
      output_pwm_ = effort_pid_.run(sampling_time_, total_effort, current_effort);
      break;
    }
    case (HebiControlStrategies::Strategy3): {
      double pwm_position_loop = position_pid_.run(sampling_time_, position_reference_.Radian(),
                                                   current_position);
      double pwm_velocity_loop = velocity_pid_.run(sampling_time_, velocity_reference_,
                                                   current_velocity);
      double pwm_effort_loop = effort_pid_.run(sampling_time_, effort_reference_, current_effort);

      // std::cout << "pwm_position_loop: " << pwm_position_loop << std::endl;
      // std::cout << "pwm_velocity_loop: " << pwm_velocity_loop << std::endl;
      // std::cout << "pwm_effort_loop: " <<  pwm_effort_loop << std::endl;

      output_pwm_ = pwm_position_loop + pwm_velocity_loop + pwm_effort_loop;
      break;
    }
    case (HebiControlStrategies::Strategy4): {
      double effort_position_loop = position_pid_.run(sampling_time_, position_reference_.Radian(),
                                                      current_position);
      double pwm_effort_loop = effort_pid_.run(sampling_time_, effort_position_loop + effort_reference_,
                                           current_effort);

      double pwm_velocity_loop = velocity_pid_.run(sampling_time_, velocity_reference_,
                                                   current_velocity);
      output_pwm_ = pwm_effort_loop + pwm_velocity_loop;
      break;
    }
  }
  output_pwm_ = limit(output_pwm_, 1.0, -1.0);

  //simple motor model
  double voltage = kDefaultVoltage * output_pwm_;
  double back_emf = (1.0 / kDefaultMotorConstant) * current_velocity;
  double current = (voltage - back_emf) / kDefaultStatorResistance;
  double torque = kDefaultMotorConstant * current;
  torque = limit(torque, 10.0, -10.0);
  joint_->SetForce(0, torque);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboHebiSEA);
}

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
}

void GazeboServoMotor::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;

  namespace_.clear();

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

  getSdfParam<double>(_sdf, "maxAngleErrorIntegral", max_angle_error_integral_,
                      max_angle_error_integral_);
  getSdfParam<std::string>(_sdf, "commandSubTopic", command_position_sub_topic_,
                           command_position_sub_topic_);
  getSdfParam<double>(_sdf, "Kp", kp_, kp_);
  getSdfParam<double>(_sdf, "Kd", kd_, kd_);
  getSdfParam<double>(_sdf, "Ki", ki_, ki_);

  getSdfParam<double>(_sdf, "maxAngle", max_angle_, max_angle_);
  getSdfParam<double>(_sdf, "minAngle", min_angle_, min_angle_);

  joint_->SetLowerLimit(0, min_angle_);
  joint_->SetUpperLimit(0, max_angle_);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboServoMotor::OnUpdate, this, _1));

  position_command_sub_ = node_handle_->subscribe(command_position_sub_topic_, 100,
                                                  &GazeboServoMotor::PositionCommandCallback, this);
}

// This gets called by the world update start event.
void GazeboServoMotor::OnUpdate(const common::UpdateInfo& _info)
{
  static double prev_sim_time = _info.simTime.Double();
  sampling_time_ = _info.simTime.Double() - prev_sim_time;
  prev_sim_time = _info.simTime.Double();
  sampling_time_ = limit(sampling_time_, 1.0, 0.001);

  if (received_first_command_ == false)
    return;

  UpdatePosition();
}

void GazeboServoMotor::PositionCommandCallback(
    const manipulator_msgs::CommandPositionServoMotorConstPtr& msg)
{
  double ref = msg->motor_angle;
  angle_reference_.SetFromRadian(ref);
  received_first_command_ = true;
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

GZ_REGISTER_MODEL_PLUGIN(GazeboServoMotor);
}

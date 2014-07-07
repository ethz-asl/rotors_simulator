/*
 * Copyright (C) 2014 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Pascal Gohl, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Sammy Omari, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * This software is released to the Contestants of the european 
 * robotics challenges (EuRoC) for the use in stage 1. (Re)-distribution, whether 
 * in parts or entirely, is NOT PERMITTED. 
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 */


#include <mav_gazebo_plugins/gazebo_motor_model.h>
#include <mav_gazebo_plugins/common.h>

namespace gazebo {
GazeboMotorModel::GazeboMotorModel()
    : ModelPlugin(),
      MotorModel(),
      node_handle_(0) {
}

GazeboMotorModel::~GazeboMotorModel() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}
;

void GazeboMotorModel::InitializeParams() {
}
;
void GazeboMotorModel::Publish() {
  turning_velocity_msg_.data = this->joint_->GetVelocity(0);
  motor_vel_pub_.publish(turning_velocity_msg_);
}
;

void GazeboMotorModel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  this->model_ = _model;

  // default params
  namespace_.clear();
  command_topic_ = "command/motors";
  motor_velocity_topic_ = "turning_vel";
  rotor_drag_coefficient_ = 1e-4;
  rolling_moment_coefficient_ = 0;

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("commandTopic"))
    command_topic_ = _sdf->GetElement("commandTopic")->Get<std::string>();

  if (_sdf->HasElement("motorVelocityTopic"))
    motor_velocity_topic_ = _sdf->GetElement("motorVelocityTopic")->Get<std::string>();

  if (_sdf->HasElement("jointName"))
    joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a jointName.\n";
  // Get the pointer to the joint
  this->joint_ = this->model_->GetJoint(joint_name_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a linkName.\n";
  // Get the pointer to the link
  this->link_ = this->model_->GetLink(link_name_);

  if (_sdf->HasElement("motorNumber"))
    motor_number_ = _sdf->GetElement("motorNumber")->Get<int>();
  else
    gzerr << "[gazebo_motor_model] Please specify a motorNumber.\n";

  if (_sdf->HasElement("rotorDragCoefficient"))
    rotor_drag_coefficient_ = _sdf->GetElement("rotorDragCoefficient")->Get<double>();
  else
    gzwarn << "[gazebo_motor_model] No rotorDragCoefficient value specified for motor " << motor_number_
           << " using default value " << rotor_drag_coefficient_ << ".\n";

  if (_sdf->HasElement("rollingMomentCoefficient"))
    rolling_moment_coefficient_ = _sdf->GetElement("rollingMomentCoefficient")->Get<double>();
  else
    gzwarn << "[gazebo_motor_model] No rollingMomentCoefficient value specified for motor " << motor_number_
           << " using default value " << rolling_moment_coefficient_ << ".\n";

  if (_sdf->HasElement("turningDirection")) {
    std::string turning_direction = _sdf->GetElement("turningDirection")->Get<std::string>();
    if (turning_direction == "cw")
      turning_direction_ = turning_direction::CW;
    else if (turning_direction == "ccw")
      turning_direction_ = turning_direction::CCW;
    else
      gzerr << "[gazebo_motor_model] Please only use 'cw' or 'ccw' as turningDirection.\n";
  }
  else
    gzerr << "[gazebo_motor_model] Please specify a turning direction ('cw' or 'ccw').\n";

  if (_sdf->HasElement("maxRotVelocity"))
    max_rot_velocity_ = _sdf->GetElement("maxRotVelocity")->Get<double>();
  else
    gzerr << "[gazebo_motor_model] Please specify a maxRotVelocity for the joint.\n";

  if (_sdf->HasElement("timeConstant"))
    time_constant_ = _sdf->GetElement("timeConstant")->Get<double>();
  else
    gzerr << "[gazebo_motor_model] Please specify a timeConstant for the joint.\n";

  inertia_ = link_->GetInertial()->GetIZZ();
  viscous_friction_coefficient_ = inertia_ / time_constant_;
  max_force_ = max_rot_velocity_ * viscous_friction_coefficient_;

  // Set the maximumForce on the joint
  this->joint_->SetMaxForce(0, max_force_);

  if (_sdf->HasElement("motorConstant"))
    motor_constant_ = _sdf->GetElement("motorConstant")->Get<double>();
  else
    gzerr << "[gazebo_motor_model] Please specify a motorConstant for the motor.\n";

  if (_sdf->HasElement("momentConstant"))
    moment_constant_ = _sdf->GetElement("momentConstant")->Get<double>();
  else
    gzerr << "[gazebo_motor_model] Please specify a momentConstant for the motor.\n";

  getSdfParam<double>(_sdf, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_, 10);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboMotorModel::OnUpdate, this, _1));

  cmd_sub_ = node_handle_->subscribe(command_topic_, 1000, &GazeboMotorModel::VelocityCallback, this);
  motor_vel_pub_ = node_handle_->advertise<std_msgs::Float32>(motor_velocity_topic_, 10);
}

// Called by the world update start event
void GazeboMotorModel::OnUpdate(const common::UpdateInfo& /*_info*/) {
  UpdateForcesAndMoments();
  Publish();
}

void GazeboMotorModel::VelocityCallback(const mav_msgs::MotorSpeedPtr& rot_velocities) {
  ref_motor_rot_vel_ = std::min(rot_velocities->motor_speed[motor_number_], static_cast<float>(max_rot_velocity_));
}

void GazeboMotorModel::UpdateForcesAndMoments() {

  motor_rot_vel_ = this->joint_->GetVelocity(0);
  // TODO: We had to add a factor of 10 here and one in the SetVelocity,
  // because currently gazebo doesn't allow to set the velocity to a higher value than 100.
  double real_motor_velocity = motor_rot_vel_ * rotor_velocity_slowdown_sim_;
  double force = real_motor_velocity * real_motor_velocity * motor_constant_;
  // Apply a force to the link
  this->link_->AddRelativeForce(math::Vector3(0, 0, force));

  // Forces from Philppe Martin's and Erwan Salaün's
  // 2010 IEEE Conference on Robotics and Automation paper
  // The True Role of Accelerometer Feedback in Quadrotor Control
  // - \omega * \lambda_1 * V_A^{\perp}
  math::Vector3 joint_axis = joint_->GetGlobalAxis(0);
  math::Vector3 body_velocity = link_->GetWorldLinearVel();
  math::Vector3 body_velocity_perpendicular = body_velocity - (body_velocity * joint_axis) * joint_axis;
  math::Vector3 air_drag = -std::abs(real_motor_velocity) * rotor_drag_coefficient_ * body_velocity_perpendicular;
  // Apply air_drag to link
  this->link_->AddForce(air_drag);
  // Moments
  this->link_->AddRelativeTorque(math::Vector3(0, 0, -turning_direction_ * force * moment_constant_));

  math::Vector3 rolling_moment;
  // - \omega * \mu_1 * V_A^{\perp}
  rolling_moment = -std::abs(real_motor_velocity) * rolling_moment_coefficient_ * body_velocity_perpendicular;
  this->link_->AddRelativeTorque(rolling_moment);
  this->joint_->SetVelocity(0, turning_direction_ * ref_motor_rot_vel_ / rotor_velocity_slowdown_sim_);
}
;

GZ_REGISTER_MODEL_PLUGIN(GazeboMotorModel);
}

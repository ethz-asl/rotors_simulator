//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>
// All rights reserved.
//
// TODO(ff): Enter some license
//==============================================================================
#include <mav_gazebo_plugins/gazebo_motor_model.h>

namespace gazebo
{
  GazeboMotorModel::GazeboMotorModel() : 
    ModelPlugin(), MotorModel(), node_handle_(0) {}

  GazeboMotorModel::~GazeboMotorModel() {
    event::Events::DisconnectWorldUpdateBegin(updateConnection_);
    if (node_handle_) {
      node_handle_->shutdown();
      delete node_handle_;
    }
  };

  void GazeboMotorModel::initializeParams() {};
  void GazeboMotorModel::publish() {};

  void GazeboMotorModel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Store the pointer to the model
    this->model_ = _model;

    // default params
    namespace_.clear();
    command_topic_ = "command/motor";
    motor_velocity_topic_ = "turning_vel";

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

    if (_sdf->HasElement("turningDirection")) {
      std::string turning_direction = _sdf->GetElement("turningDirection")->Get<std::string>();
      if(turning_direction == "cw")
        turning_direction_ = -1;
      else if(turning_direction == "ccw")
        turning_direction_ = 1;
      else
        gzerr << "[gazebo_motor_model] Please only use 'cw' or 'ccw' as turningDirection.\n";
    }
    else
      gzerr << "[gazebo_motor_model] Please specify a turning direction ('cw' or 'acw').\n";
    printf("turning direction set to %d\n", turning_direction_);

    if (_sdf->HasElement("maxForce")) 
      max_force_ = _sdf->GetElement("maxForce")->Get<double>();
    else
      gzerr << "[gazebo_motor_model] Please specify a maxForce for the joint.\n";
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

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GazeboMotorModel::OnUpdate, this, _1));

    cmd_sub_ = node_handle_->subscribe(command_topic_, 1000, &GazeboMotorModel::velocityCallback, this);
    motor_vel_pub_ = node_handle_->advertise<std_msgs::Float32>(motor_velocity_topic_, 10);
  }
  
  // Called by the world update start event
  void GazeboMotorModel::OnUpdate(const common::UpdateInfo & /*_info*/)
  {
    calculateMotorVelocity();
  }

  void GazeboMotorModel::velocityCallback(
    const std_msgs::Float32MultiArrayPtr& velocities)
  {
    ref_motor_rot_vel_ = velocities->data[motor_number_];
  }

  void GazeboMotorModel::calculateMotorVelocity() {
    motor_rot_vel_ = this->joint_->GetVelocity(0);
    turning_velocity_msg_.data = motor_rot_vel_;
    motor_vel_pub_.publish(turning_velocity_msg_);

    // Apply a force to the link
    this->link_->AddRelativeForce(
      math::Vector3(0, 0, motor_rot_vel_*motor_rot_vel_*motor_constant_));
    
    // Moments
    this->link_->AddRelativeTorque(math::Vector3(
      0, 0, turning_direction_*motor_rot_vel_*motor_rot_vel_*motor_constant_*moment_constant_));
    this->joint_->SetVelocity(0, turning_direction_*ref_motor_rot_vel_);
  };

  GZ_REGISTER_MODEL_PLUGIN(GazeboMotorModel);
}

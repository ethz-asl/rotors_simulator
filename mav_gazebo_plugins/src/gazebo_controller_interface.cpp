//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>
// All rights reserved.
//
// TODO(ff): Enter some license
//==============================================================================
#include <mav_gazebo_plugins/gazebo_controller_interface.h>

namespace gazebo
{
  GazeboControllerInterface::GazeboControllerInterface() : 
    ModelPlugin(), node_handle_(0), controller_created_(false) {}

  GazeboControllerInterface::~GazeboControllerInterface() {
    event::Events::DisconnectWorldUpdateBegin(updateConnection_);
    if (node_handle_) {
      node_handle_->shutdown();
      delete node_handle_;
    }
  };

  // void GazeboControllerInterface::InitializeParams() {};
  // void GazeboControllerInterface::Publish() {};

  void GazeboControllerInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Store the pointer to the model
    this->model_ = _model;

    // default params
    namespace_.clear();
    std::string command_topic_attitude = "command/attitude";
    std::string command_topic_rate = "command/rate";
    std::string command_topic_motor = "command/motor";

    if (_sdf->HasElement("robotNamespace"))
      namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
      gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";
    node_handle_ = new ros::NodeHandle(namespace_);

    if (_sdf->HasElement("commandTopicAttitude"))
      command_topic_attitude = _sdf->GetElement("commandTopicAttitude")->Get<std::string>();

    if (_sdf->HasElement("commandTopicMotor"))
      command_topic_motor = _sdf->GetElement("commandTopicMotor")->Get<std::string>();

    if (_sdf->HasElement("imuTopic"))
      imu_topic_ = _sdf->GetElement("imuTopic")->Get<std::string>();

    if (_sdf->HasElement("motorVelocityReferenceTopic")) {
      motor_velocity_topic_ = _sdf->GetElement(
        "motorVelocityReferenceTopic")->Get<std::string>();
    }


    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GazeboControllerInterface::OnUpdate, this, _1));

    cmd_attitude_sub_ = node_handle_->subscribe(command_topic_attitude,
      10, &GazeboControllerInterface::CommandAttitudeCallback, this);
    cmd_motor_sub_ = node_handle_->subscribe(command_topic_motor,
      10, &GazeboControllerInterface::CommandMotorCallback, this);
    imu_sub_ = node_handle_->subscribe(imu_topic_,
      10, &GazeboControllerInterface::ImuCallback, this);
    motor_cmd_pub_ = node_handle_->advertise<mav_msgs::MotorSpeed>(
      motor_velocity_topic_, 10);
  }

  // Called by the world update start event
  void GazeboControllerInterface::OnUpdate(const common::UpdateInfo& /*_info*/)
  {
    if(!controller_created_)
      return;
    Eigen::VectorXd ref_rotor_velocities;
    controller_->CalculateRotorVelocities(&ref_rotor_velocities);

    turning_velocities_msg_.motor_speed.clear();
    for (int i = 0; i < ref_rotor_velocities.size(); i++)
      turning_velocities_msg_.motor_speed.push_back(ref_rotor_velocities[i]);

    motor_cmd_pub_.publish(turning_velocities_msg_);
  }

  void GazeboControllerInterface::CommandAttitudeCallback(
    const mav_msgs::ControlAttitudeThrustPtr& input_reference_msg)
  {
    if(!controller_created_) {
      // Get the controller and initialize its parameters.
      controller_ = mav_controller_factory::ControllerFactory::Instance()
        .CreateController("AttitudeController");
      controller_->InitializeParams();
      controller_created_ = true;
      gzmsg << "started AttitudeController" << std::endl;
    }
    Eigen::Vector4d input_reference (
      input_reference_msg->roll,
      input_reference_msg->pitch,
      input_reference_msg->yaw_rate,
      input_reference_msg->thrust);
    controller_->SetAttitudeThrustReference(input_reference);
  }

  void GazeboControllerInterface::CommandMotorCallback(
    const mav_msgs::ControlMotorSpeedPtr& input_reference_msg)
  {
    if(!controller_created_) {
      // Get the controller and initialize its parameters.
      controller_ = mav_controller_factory::ControllerFactory::Instance()
        .CreateController("MotorController");
      controller_->InitializeParams();
      controller_created_ = true;
      gzmsg << "started MotorController" << std::endl;
    }

    Eigen::VectorXd input_reference;
    input_reference.resize(input_reference_msg->motor_speed.size());
    for(int i=0; i<input_reference_msg->motor_speed.size(); ++i) {
      input_reference[i] = input_reference_msg->motor_speed[i];
    }
    controller_->SetMotorReference(input_reference);
  }

  void GazeboControllerInterface::ImuCallback(
    const sensor_msgs::ImuPtr& imu_msg)
  {
    if(!controller_created_)
      return;
    Eigen::Vector3d angular_rate (
      imu_msg->angular_velocity.x,
      imu_msg->angular_velocity.y,
      imu_msg->angular_velocity.z);
    controller_->SetAngularRate(angular_rate);

    Eigen::Quaternion<double> orientation (
      imu_msg->orientation.w,
      imu_msg->orientation.x,
      imu_msg->orientation.y,
      imu_msg->orientation.z);
    controller_->SetAttitude(orientation);
    // imu->linear_acceleration;
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboControllerInterface);
}

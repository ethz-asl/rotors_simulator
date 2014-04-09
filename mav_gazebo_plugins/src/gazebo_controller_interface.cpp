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
    ModelPlugin(), node_handle_(0) {}

  GazeboControllerInterface::~GazeboControllerInterface() {
    event::Events::DisconnectWorldUpdateBegin(updateConnection_);
    if (node_handle_) {
      node_handle_->shutdown();
      delete node_handle_;
    }
  };

  // void GazeboControllerInterface::UpdateStates() {
  //   position_ = control_input_.position;
  //   velocity_ = control_input_.velocity;
  //   omega_ = control_input_.omega;
  //   attitude_ = control_input_.attitude;
  // }

  // void GazeboControllerInterface::InitializeParams() {};
  // void GazeboControllerInterface::Publish() {};

  void GazeboControllerInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Store the pointer to the model
    this->model_ = _model;

    // default params
    namespace_.clear();
    command_topic_ = "command/motor";

    // get Controller
    controller_ = factory_.GetNewInstance("attitude_controller");

    // if (_sdf->HasElement("robotNamespace"))
    //   namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    // else
    //   gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";
    // node_handle_ = new ros::NodeHandle(namespace_);

    if (_sdf->HasElement("commandTopic"))
      command_topic_ = _sdf->GetElement("commandTopic")->Get<std::string>();

    if (_sdf->HasElement("imuTopic"))
      imu_topic_ = _sdf->GetElement("imuTopic")->Get<std::string>();

    if (_sdf->HasElement("poseTopic"))
      pose_topic_ = _sdf->GetElement("poseTopic")->Get<std::string>();

    if (_sdf->HasElement("motorVelocityReferenceTopic")) {
      motor_velocity_topic_ = _sdf->GetElement(
        "motorVelocityReferenceTopic")->Get<std::string>();
    }

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GazeboControllerInterface::OnUpdate, this, _1));

    cmd_sub_ = node_handle_->subscribe(command_topic_,
      1000, &GazeboControllerInterface::ControlCommandCallback, this);
    imu_sub_ = node_handle_->subscribe(imu_topic_,
      1000, &GazeboControllerInterface::ImuCallback, this);
    pose_sub_ = node_handle_->subscribe(pose_topic_,
      1000, &GazeboControllerInterface::PoseCallback, this);
    motor_cmd_pub_ = node_handle_->advertise<std_msgs::Float32MultiArray>(
      motor_velocity_topic_, 10);
  }

  // Called by the world update start event
  void GazeboControllerInterface::OnUpdate(const common::UpdateInfo& /*_info*/)
  {
    Eigen::VectorXd ref_motor_velocities = controller_->GetMotorVelocities();
    turning_velocities_msg_.data.clear();
    for (int i = 0; i < ref_motor_velocities.size(); i++)
      turning_velocities_msg_.data.push_back(ref_motor_velocities[i]);

    motor_cmd_pub_.publish(turning_velocities_msg_);
  }

  void GazeboControllerInterface::ControlCommandCallback(
    const mav_msgs::ControlAttitudeThrustPtr& input_reference_msg)
  {
    Eigen::Vector4d input_reference (
      input_reference_msg->roll,
      input_reference_msg->pitch,
      input_reference_msg->yaw_rate,
      input_reference_msg->thrust);
    controller_->SetAttitudeThrustReference(input_reference);
  }

  void GazeboControllerInterface::ImuCallback(
    const sensor_msgs::ImuPtr& imu_msg)
  {
    Eigen::Vector3d velocity (
      imu_msg->angular_velocity.x,
      imu_msg->angular_velocity.y,
      imu_msg->angular_velocity.z);
    controller_->SetVelocity(velocity);
    // imu->linear_acceleration;
  }

  void GazeboControllerInterface::PoseCallback(
    const geometry_msgs::PoseStampedPtr& pose_msg)
  {
    Eigen::Quaternion<double> orientation (
      pose_msg->pose.orientation.w,
      pose_msg->pose.orientation.x,
      pose_msg->pose.orientation.y,
      pose_msg->pose.orientation.z);
    controller_->SetAttitude(orientation);
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboControllerInterface);
}

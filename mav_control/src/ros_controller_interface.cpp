//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>, Michael Burri <burri210@gmail.com>
// All rights reserved.
//
// TODO(ff): Enter some license
//==============================================================================
#include <mav_control/ros_controller_interface.h>

namespace mav_control {

RosControllerInterface::RosControllerInterface()
    : node_handle_(0),
      controller_created_(false) {
  node_handle_ = new ros::NodeHandle("/");

  InitializeParams();

  cmd_attitude_sub_ = node_handle_->subscribe(command_topic_attitude_, 10,
                                              &RosControllerInterface::CommandAttitudeCallback, this);
  cmd_motor_sub_ = node_handle_->subscribe(command_topic_motor_, 10,
                                           &RosControllerInterface::CommandMotorCallback, this);
  cmd_trajectory_sub_ = node_handle_->subscribe(command_topic_trajectory_, 10,
                                           &RosControllerInterface::CommandTrajectoryCallback, this);
  imu_sub_ = node_handle_->subscribe(imu_topic_, 10, &RosControllerInterface::ImuCallback, this);
  pose_sub_ = node_handle_->subscribe(pose_topic_, 10, &RosControllerInterface::PoseCallback, this);
  ekf_sub_ = node_handle_->subscribe(ekf_topic_, 10, &RosControllerInterface::ExtEkfCallback, this);

ROS_INFO_STREAM("subscribing to: "<< command_topic_trajectory_);

  motor_cmd_pub_ = node_handle_->advertise<mav_msgs::MotorSpeed>(motor_velocity_topic_, 10);
}

RosControllerInterface::~RosControllerInterface() {
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void RosControllerInterface::InitializeParams() {
  //TODO(burrimi): Read parameters from yaml.
  node_handle_->param<std::string>("ekf_topic", ekf_topic_, "msf_core/state_out");
  node_handle_->param<std::string>("command_topic_attitude_", command_topic_attitude_, "command/attitude");
  node_handle_->param<std::string>("command_topic_rate_", command_topic_rate_, "command/rate");
  node_handle_->param<std::string>("command_topic_motor_", command_topic_motor_, "command/motor");
  node_handle_->param<std::string>("command_topic_trajectory_", command_topic_trajectory_, "/fcu/control_new");
  node_handle_->param<std::string>("imu_topic", imu_topic_, "imu");
  node_handle_->param<std::string>("pose_topic", pose_topic_, "sensor_pose");
  node_handle_->param<std::string>("motor_velocity_topic", motor_velocity_topic_, "motor_velocity");

}
void RosControllerInterface::Publish() {
}

void RosControllerInterface::CommandTrajectoryCallback(
    const mav_msgs::ControlTrajectoryConstPtr& trajectory_reference_msg) {
  if (!controller_created_) {
    // Get the controller and initialize its parameters.
    controller_ = mav_controller_factory::ControllerFactory::Instance()
        .CreateController("LeePositionController");
    controller_->InitializeParams();
    controller_created_ = true;
    ROS_INFO_STREAM("started LeePositionController" << std::endl);
  }

  Eigen::Vector3d position_reference(trajectory_reference_msg->position[0],
                                     trajectory_reference_msg->position[1],
                                     trajectory_reference_msg->position[2]);

  controller_->SetPositionReference(position_reference);

  Eigen::Vector3d velocity_reference(trajectory_reference_msg->velocity[0],
                                     trajectory_reference_msg->velocity[1],
                                     trajectory_reference_msg->velocity[2]);
  controller_->SetVelocityReference(velocity_reference);

  Eigen::Vector3d acceleration_reference(trajectory_reference_msg->acceleration[0],
                                         trajectory_reference_msg->acceleration[1],
                                         trajectory_reference_msg->acceleration[2]);
  controller_->SetAccelerationReference(acceleration_reference);

  Eigen::Vector3d jerk_reference(trajectory_reference_msg->jerk[0],
                                 trajectory_reference_msg->jerk[1],
                                 trajectory_reference_msg->jerk[2]);
  controller_->SetJerkReference(jerk_reference);

  controller_->SetYawReference(trajectory_reference_msg->yaw);
  controller_->SetYawRateReference(trajectory_reference_msg->yaw_rate);
}

void RosControllerInterface::CommandAttitudeCallback(
    const mav_msgs::ControlAttitudeThrustConstPtr& input_reference_msg) {
  if (!controller_created_) {
    // Get the controller and initialize its parameters.
    controller_ = mav_controller_factory::ControllerFactory::Instance()
        .CreateController("AttitudeController");
    controller_->InitializeParams();
    controller_created_ = true;
    ROS_INFO_STREAM("started AttitudeController" << std::endl);
  }
  Eigen::Vector4d input_reference(input_reference_msg->roll,
                                  input_reference_msg->pitch,
                                  input_reference_msg->yaw_rate,
                                  input_reference_msg->thrust);
  controller_->SetAttitudeThrustReference(input_reference);
}

void RosControllerInterface::ExtEkfCallback(
    const sensor_fusion_comm::DoubleArrayStampedConstPtr ekf_state) {
  if (!controller_created_)
    return;

  Eigen::Vector3d position(ekf_state->data[0], ekf_state->data[1],
                           ekf_state->data[2]);
  controller_->SetPosition(position);

  Eigen::Vector3d velocity(ekf_state->data[3], ekf_state->data[4],
                           ekf_state->data[5]);
  controller_->SetVelocity(velocity);

  Eigen::Quaternion<double> orientation(ekf_state->data[6], ekf_state->data[7],
                                        ekf_state->data[8], ekf_state->data[9]);
  controller_->SetAttitude(orientation);

  // TODO(burrimi): do the calculation at a better place.
  Eigen::VectorXd ref_rotor_velocities;
  controller_->CalculateRotorVelocities(&ref_rotor_velocities);

  mav_msgs::MotorSpeed turning_velocities_msg;

  turning_velocities_msg.motor_speed.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    turning_velocities_msg.motor_speed.push_back(ref_rotor_velocities[i]);

  motor_cmd_pub_.publish(turning_velocities_msg);

}

void RosControllerInterface::CommandMotorCallback(
    const mav_msgs::ControlMotorSpeedConstPtr& input_reference_msg) {
  if (!controller_created_) {
    // Get the controller and initialize its parameters.
    controller_ = mav_controller_factory::ControllerFactory::Instance()
        .CreateController("MotorController");
    controller_->InitializeParams();
    controller_created_ = true;
    ROS_INFO_STREAM("started AttitudeController" << std::endl);
  }

  Eigen::VectorXd input_reference;
  input_reference.resize(input_reference_msg->motor_speed.size());
  for (int i = 0; i < input_reference_msg->motor_speed.size(); ++i) {
    input_reference[i] = input_reference_msg->motor_speed[i];
  }
  controller_->SetMotorReference(input_reference);
}

void RosControllerInterface::ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {
  if (!controller_created_)
    return;
  Eigen::Vector3d angular_rate(imu_msg->angular_velocity.x,
                               imu_msg->angular_velocity.y,
                               imu_msg->angular_velocity.z);
  controller_->SetAngularRate(angular_rate);
  // imu->linear_acceleration;
}

void RosControllerInterface::PoseCallback(
    const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  if (!controller_created_)
    return;
  Eigen::Quaternion<double> orientation(pose_msg->pose.orientation.w,
                                        pose_msg->pose.orientation.x,
                                        pose_msg->pose.orientation.y,
                                        pose_msg->pose.orientation.z);
  controller_->SetAttitude(orientation);
}
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros_controller_node");

  mav_control::RosControllerInterface controller_interface;

  ros::spin();

  return 0;
}

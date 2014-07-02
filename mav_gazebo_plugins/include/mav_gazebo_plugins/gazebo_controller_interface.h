//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>
// All rights reserved.
//
// TODO(ff): Enter some license
//==============================================================================
#ifndef MAV_GAZEBO_PLUGINS_CONTROLLER_INTERFACE_H
#define MAV_GAZEBO_PLUGINS_CONTROLLER_INTERFACE_H

#include <mav_control/controller_factory.h>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <stdio.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/CommandAttitudeThrust.h>
#include <mav_msgs/CommandRateThrust.h>
#include <mav_msgs/CommandMotorSpeed.h>
#include <mav_msgs/MotorSpeed.h>


namespace gazebo
{
  class GazeboControllerInterface : public ModelPlugin
  {
    public:
      GazeboControllerInterface();
      ~GazeboControllerInterface();

      void InitializeParams();
      void Publish();

    protected:
      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      void OnUpdate(const common::UpdateInfo& /*_info*/);

    private:
      std::shared_ptr<ControllerBase> controller_;
      bool controller_created_;

      std::string namespace_;
      std::string command_topic_;
      std::string imu_topic_;
      std::string motor_velocity_topic_;

      ros::NodeHandle* node_handle_;
      ros::Publisher motor_cmd_pub_;
      ros::Subscriber cmd_attitude_sub_;
      ros::Subscriber cmd_motor_sub_;
      ros::Subscriber imu_sub_;

      // Pointer to the model
      physics::ModelPtr model_;
      // Pointer to the update event connection
      event::ConnectionPtr updateConnection_;

      sensor_msgs::Imu imu_;

      mav_msgs::MotorSpeed turning_velocities_msg_;

      boost::thread callback_queue_thread_;
      void QueueThread();
      void CommandAttitudeCallback(const mav_msgs::CommandAttitudeThrustPtr& input_reference_msg);
      void CommandRateCallback(const mav_msgs::CommandRateThrustPtr& input_reference_msg);
      void CommandMotorCallback(const mav_msgs::CommandMotorSpeedPtr& input_reference_msg);
      void ImuCallback(const sensor_msgs::ImuPtr& imu);
      void PoseCallback(const geometry_msgs::PoseStampedPtr& pose);
  };
}

#endif // MAV_GAZEBO_PLUGINS_CONTROLLER_INTERFACE_H

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
#include <mav_msgs/ControlAttitudeThrust.h>
#include <std_msgs/Float32MultiArray.h>


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
      std::string namespace_;
      std::string command_topic_;
      std::string imu_topic_;
      std::string pose_topic_;
      std::string motor_velocity_topic_;

      ros::NodeHandle* node_handle_;
      ros::Publisher motor_cmd_pub_;
      ros::Subscriber cmd_sub_;
      ros::Subscriber imu_sub_;
      ros::Subscriber pose_sub_;

      // Pointer to the model
      physics::ModelPtr model_;
      // Pointer to the update event connection
      event::ConnectionPtr updateConnection_;

      sensor_msgs::Imu imu_;

      std_msgs::Float32MultiArray turning_velocities_msg_;

      boost::thread callback_queue_thread_;
      void QueueThread();
      void ControlCommandCallback(const mav_msgs::ControlAttitudeThrustPtr& input_reference_msg);
      void ImuCallback(const sensor_msgs::ImuPtr& imu);
      void PoseCallback(const geometry_msgs::PoseStampedPtr& pose);
  };
}

#endif // MAV_GAZEBO_PLUGINS_CONTROLLER_INTERFACE_H

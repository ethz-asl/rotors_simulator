//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>
// All rights reserved.
//
// TODO(ff): Enter some license
//==============================================================================
#ifndef MAV_GAZEBO_PLUGINS_MOTOR_MODELS_H
#define MAV_GAZEBO_PLUGINS_MOTOR_MODELS_H

#include <mav_model/motor_model.hpp>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <stdio.h>
#include <std_msgs/Float32.h>
#include <mav_msgs/MotorSpeed.h>

namespace turning_direction {
  const static int CCW = 1;
  const static int CW = -1;
};

namespace gazebo
{
  class GazeboMotorModel : public MotorModel, public ModelPlugin
  {
    public:
      GazeboMotorModel();
      virtual ~GazeboMotorModel();

      virtual void InitializeParams();
      virtual void Publish();

    protected:
      virtual void UpdateForcesAndMoments();
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

    private:
      std::string namespace_;
      std::string joint_name_;
      std::string link_name_;
      std::string command_topic_;
      std::string motor_velocity_topic_;
      int motor_number_;

      int turning_direction_;
      double max_force_;
      double motor_constant_;
      double moment_constant_;
      double time_constant_;
      double max_rot_velocity_;
      double viscous_friction_coefficient_;
      double inertia_;
      double lambda1_;

      ros::NodeHandle* node_handle_;
      ros::Publisher motor_vel_pub_;
      ros::Subscriber cmd_sub_;

      // Pointer to the model
      physics::ModelPtr model_;
      // Pointer to the joint
      physics::JointPtr joint_;
      // Pointer to the link
      physics::LinkPtr link_;
      // Pointer to the update event connection
      event::ConnectionPtr updateConnection_;

      boost::thread callback_queue_thread_;
      void QueueThread();
      std_msgs::Float32 turning_velocity_msg_;
      void VelocityCallback(const mav_msgs::MotorSpeedPtr& rot_velocities);
  };
}

#endif // MAV_GAZEBO_PLUGINS_MOTOR_MODELS_H

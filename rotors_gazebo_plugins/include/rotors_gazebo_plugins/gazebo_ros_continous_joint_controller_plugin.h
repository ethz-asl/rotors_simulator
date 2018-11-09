//
// Created by nico on 08.11.18.
// Based on the gazebo tutorial
// "http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i1"
//
#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_ROS_CONTINOUS_JOINT_CONTROLLER_PLUGIN_H_
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_ROS_CONTINOUS_JOINT_CONTROLLER_PLUGIN_H_

#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <thread>
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float64.h"
#include "rotors_comm/SetFrequency.h"

namespace gazebo {

/// \brief A plugin to control a continous joint at a constant rotational
/// frequency.
class ContinousJointControllerPlugin : public ModelPlugin {

  /// \brief Constructor
 public:
  ContinousJointControllerPlugin() {}

  /// \brief The load function is called by Gazebo when the plugin is
  /// inserted into simulation
  /// \param[in] _model A pointer to the model that this plugin is
  /// attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

 private:
  /// \brief Handle an incoming message from ROS
  /// \param[in] _msg A float value that is used to set the velocity
  /// of the joint.
  bool OnServiceCall(rotors_comm::SetFrequencyRequest &request,
                     rotors_comm::SetFrequencyResponse &response);

  /// \brief Set the velocity of the joint
  /// \param[in] spin_frequency_hz The target frequency at which the joint
  /// should rotate
  void SetVelocity(const double spin_frequency_hz);

  /// \brief A node use for ROS transport
  std::unique_ptr<ros::NodeHandle> ros_node_;

  /// \brief The ROS service provider
  ros::ServiceServer ros_service_;

  /// \brief A node used for transport
  transport::NodePtr node_;

  /// \brief Pointer to the model.
  physics::ModelPtr model_;

  /// \brief Pointer to the joint.
  physics::JointPtr joint_;

  /// \brief A PID controller for the joint.
  common::PID pid_;

  /// \brief The frequency [HZ] at which the joint should rotate
  double spin_frequency_;
};
}
#endif  // ROTORS_GAZEBO_PLUGINS_GAZEBO_ROS_CONTINOUS_JOINT_CONTROLLER_PLUGIN_H_

#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_ROS_CONTINOUS_JOINT_CONTROLLER_PLUGIN_H_
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_ROS_CONTINOUS_JOINT_CONTROLLER_PLUGIN_H_
//
// Created by nico on 08.11.18.
//

#include "motor_state.h"

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <ros/advertise_options.h>
#include <ros/callback_queue.h>

// Boost
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>

namespace gazebo {

/// \brief A plugin to control a continous joint at a constant rotational
/// frequency.
class ContinousJointControllerPlugin : public ModelPlugin {

  /// \brief Constructor
 public:
  ContinousJointControllerPlugin();
  ~ContinousJointControllerPlugin();

  void InitServices();
  /// inserted into simulation
  /// \param[in] _model A pointer to the model that this plugin is
  /// attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  MotorState ReadMotor() const;

  void UpdateMotor(const MotorState& read_motor_state);

  void OnWorldUpdate();

 protected:
  void Shutdown();

 private:
  physics::WorldPtr world_;
  physics::ModelPtr parent_;
  ros::ServiceServer set_frequency_service_;
  ros::NodeHandle* nh_;

  std::string robot_namespace_;
  std::string service_name_;

  bool alive_;
  physics::JointPtr joint_;

  double spin_frequency_;

  event::ConnectionPtr update_connection_;
  MotorState current_motor_state_;

  physics::JointPtr GetReferencedJoint(physics::ModelPtr parent,
                                       sdf::ElementPtr sdf,
                                       const std::string& jointName);

  void SetVelocity(double spin_frequency);
};
}
#endif  // ROTORS_GAZEBO_PLUGINS_GAZEBO_ROS_CONTINOUS_JOINT_CONTROLLER_PLUGIN_H_

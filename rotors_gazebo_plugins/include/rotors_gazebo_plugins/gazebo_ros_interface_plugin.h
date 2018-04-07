/*
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROTORS_GAZEBO_PLUGINS_MSG_INTERFACE_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_MSG_INTERFACE_PLUGIN_H

// SYSTEM INCLUDES
#include <random>

#include <Eigen/Core>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/msgs/msgs.hh"

//=================== ROS =====================//
#include <mav_msgs/default_topics.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

//============= GAZEBO MSG TYPES ==============//
#include "ConnectGazeboToRosTopic.pb.h"
#include "ConnectRosToGazeboTopic.pb.h"

#include "Actuators.pb.h"
#include "CommandMotorSpeed.pb.h"
#include "Float32.pb.h"
#include "FluidPressure.pb.h"
#include "Imu.pb.h"
#include "JointState.pb.h"
#include "MagneticField.pb.h"
#include "NavSatFix.pb.h"
#include "Odometry.pb.h"
#include "PoseWithCovarianceStamped.pb.h"
#include "RollPitchYawrateThrust.pb.h"
#include "TransformStamped.pb.h"
#include "TransformStampedWithFrameIds.pb.h"
#include "TwistStamped.pb.h"
#include "Vector3dStamped.pb.h"
#include "WindSpeed.pb.h"
#include "WrenchStamped.pb.h"

//=============== ROS MSG TYPES ===============//
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <nav_msgs/Odometry.h>
#include <rotors_comm/WindSpeed.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32.h>

#include "rotors_gazebo_plugins/common.h"

namespace gazebo {

// typedef's to make life easier
typedef const boost::shared_ptr<const gz_std_msgs::ConnectGazeboToRosTopic>
    GzConnectGazeboToRosTopicMsgPtr;
typedef const boost::shared_ptr<const gz_std_msgs::ConnectRosToGazeboTopic>
    GzConnectRosToGazeboTopicMsgPtr;
typedef const boost::shared_ptr<const gz_std_msgs::Float32> GzFloat32MsgPtr;
typedef const boost::shared_ptr<const gz_geometry_msgs::Odometry>
    GzOdometryMsgPtr;
typedef const boost::shared_ptr<const gazebo::msgs::Pose> GzPoseMsgPtr;
typedef const boost::shared_ptr<
    const gz_geometry_msgs::PoseWithCovarianceStamped>
    GzPoseWithCovarianceStampedMsgPtr;
typedef const boost::shared_ptr<const gz_geometry_msgs::TransformStamped>
    GzTransformStampedMsgPtr;
typedef const boost::shared_ptr<
    const gz_geometry_msgs::TransformStampedWithFrameIds>
    GzTransformStampedWithFrameIdsMsgPtr;
typedef const boost::shared_ptr<const gz_geometry_msgs::TwistStamped>
    GzTwistStampedMsgPtr;
typedef const boost::shared_ptr<const gz_geometry_msgs::Vector3dStamped>
    GzVector3dStampedMsgPtr;
typedef const boost::shared_ptr<const gz_geometry_msgs::WrenchStamped>
    GzWrenchStampedMsgPtr;
typedef const boost::shared_ptr<const gz_mav_msgs::RollPitchYawrateThrust>
    GzRollPitchYawrateThrustPtr;
typedef const boost::shared_ptr<const gz_mav_msgs::WindSpeed> GzWindSpeedMsgPtr;
typedef const boost::shared_ptr<const gz_sensor_msgs::Actuators>
    GzActuatorsMsgPtr;
typedef const boost::shared_ptr<const gz_sensor_msgs::FluidPressure>
    GzFluidPressureMsgPtr;
typedef const boost::shared_ptr<const gz_sensor_msgs::Imu> GzImuPtr;
typedef const boost::shared_ptr<const gz_sensor_msgs::JointState>
    GzJointStateMsgPtr;
typedef const boost::shared_ptr<const gz_sensor_msgs::MagneticField>
    GzMagneticFieldMsgPtr;
typedef const boost::shared_ptr<const gz_sensor_msgs::NavSatFix> GzNavSatFixPtr;

/// \brief    ROS interface plugin for Gazebo.
/// \details  This routes messages to/from Gazebo and ROS. This is used
///           so that individual plugins are not ROS dependent.
///           This is a WorldPlugin, only one of these is designed to be enabled
///           per Gazebo world.
class GazeboRosInterfacePlugin : public WorldPlugin {
 public:
  GazeboRosInterfacePlugin();
  ~GazeboRosInterfacePlugin();

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

  /// \brief  	This gets called by the world update start event.
  /// \details	Calculates IMU parameters and then publishes one IMU message.
  void OnUpdate(const common::UpdateInfo&);

 private:
  /// \brief  Provides a way for GzConnectGazeboToRosTopicMsgCallback() to
  ///         connect a Gazebo subscriber to a ROS publisher.
  /// \details
  ///   GazeboMsgT  The type of the message that will be subscribed to the
  ///   Gazebo framework.
  ///   RosMsgT     The type of the message published to the ROS framework.
  template <typename GazeboMsgT, typename RosMsgT>
  void ConnectHelper(void (GazeboRosInterfacePlugin::*fp)(
                         const boost::shared_ptr<GazeboMsgT const>&,
                         ros::Publisher),
                     GazeboRosInterfacePlugin* ptr, std::string gazeboNamespace,
                     std::string gazeboTopicName, std::string rosTopicName,
                     transport::NodePtr gz_node_handle);

  std::vector<gazebo::transport::NodePtr> nodePtrs_;
  std::vector<gazebo::transport::SubscriberPtr> subscriberPtrs_;

  // std::string namespace_;

  /// \brief  Handle for the Gazebo node.
  transport::NodePtr gz_node_handle_;

  /// \brief  Handle for the ROS node.
  ros::NodeHandle* ros_node_handle_;

  /// \brief  Pointer to the world.
  physics::WorldPtr world_;

  /// \brief  Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  // ============================================ //
  // ====== CONNECT GAZEBO TO ROS MESSAGES ====== //
  // ============================================ //

  transport::SubscriberPtr gz_connect_gazebo_to_ros_topic_sub_;
  void GzConnectGazeboToRosTopicMsgCallback(
      GzConnectGazeboToRosTopicMsgPtr& gz_connect_gazebo_to_ros_topic_msg);

  // ============================================ //
  // ====== CONNECT ROS TO GAZEBO MESSAGES ====== //
  // ============================================ //

  transport::SubscriberPtr gz_connect_ros_to_gazebo_topic_sub_;

  /// @brief    Subscribes to the provided ROS topic and publishes on the
  /// provided Gazebo topic (all info contained within the message).
  /// @details  Will create a Gazebo publisher if one doesn't already exist.
  void GzConnectRosToGazeboTopicMsgCallback(
      GzConnectRosToGazeboTopicMsgPtr& gz_connect_ros_to_gazebo_topic_msg);

  /// \brief      Looks if a publisher on the provided topic already exists, and
  ///             returns it.
  ///             If no publisher exists, this method creates one and returns
  ///             that instead.
  /// \warning    Finding an already created publisher is not supported yet!
  template <typename T>
  transport::PublisherPtr FindOrMakeGazeboPublisher(std::string topic);

  // ============================================ //
  // ===== HELPER METHODS FOR MSG CONVERSION ==== //
  // ============================================ //

  void ConvertHeaderGzToRos(
      const gz_std_msgs::Header& gz_header,
      std_msgs::Header_<std::allocator<void> >* ros_header);

  void ConvertHeaderRosToGz(
      const std_msgs::Header_<std::allocator<void> >& ros_header,
      gz_std_msgs::Header* gz_header);

  // ============================================ //
  // ===== GAZEBO->ROS CALLBACKS/CONVERTERS ===== //
  // ============================================ //

  // ACTUATORS
  void GzActuatorsMsgCallback(GzActuatorsMsgPtr& gz_actuators_msg,
                              ros::Publisher ros_publisher);
  mav_msgs::Actuators ros_actuators_msg_;

  // FLOAT32
  void GzFloat32MsgCallback(GzFloat32MsgPtr& gz_float_32_msg,
                            ros::Publisher ros_publisher);
  std_msgs::Float32 ros_float_32_msg_;

  // FLUID PRESSURE
  void GzFluidPressureMsgCallback(GzFluidPressureMsgPtr& gz_fluid_pressure_msg,
                                  ros::Publisher ros_publisher);
  sensor_msgs::FluidPressure ros_fluid_pressure_msg_;

  // IMU
  void GzImuMsgCallback(GzImuPtr& gz_imu_msg, ros::Publisher ros_publisher);
  sensor_msgs::Imu ros_imu_msg_;

  // JOINT STATE
  void GzJointStateMsgCallback(GzJointStateMsgPtr& gz_joint_state_msg,
                               ros::Publisher ros_publisher);
  sensor_msgs::JointState ros_joint_state_msg_;

  // MAGNETIC FIELD
  void GzMagneticFieldMsgCallback(GzMagneticFieldMsgPtr& gz_magnetic_field_msg,
                                  ros::Publisher ros_publisher);
  sensor_msgs::MagneticField ros_magnetic_field_msg_;

  // NAT SAT FIX (GPS)
  void GzNavSatFixCallback(GzNavSatFixPtr& gz_nav_sat_fix_msg,
                           ros::Publisher ros_publisher);
  sensor_msgs::NavSatFix ros_nav_sat_fix_msg_;

  // ODOMETRY
  void GzOdometryMsgCallback(GzOdometryMsgPtr& gz_odometry_msg,
                             ros::Publisher ros_publisher);
  nav_msgs::Odometry ros_odometry_msg_;

  // POSE
  void GzPoseMsgCallback(GzPoseMsgPtr& gz_pose_msg,
                         ros::Publisher ros_publisher);
  geometry_msgs::Pose ros_pose_msg_;

  // POSE WITH COVARIANCE STAMPED
  void GzPoseWithCovarianceStampedMsgCallback(
      GzPoseWithCovarianceStampedMsgPtr& gz_pose_with_covariance_stamped_msg,
      ros::Publisher ros_publisher);
  geometry_msgs::PoseWithCovarianceStamped
      ros_pose_with_covariance_stamped_msg_;

  // POSITION STAMPED
  void GzVector3dStampedMsgCallback(
      GzVector3dStampedMsgPtr& gz_vector_3d_stamped_msg,
      ros::Publisher ros_publisher);
  geometry_msgs::PointStamped ros_position_stamped_msg_;

  // TRANSFORM STAMPED
  void GzTransformStampedMsgCallback(
      GzTransformStampedMsgPtr& gz_transform_stamped_msg,
      ros::Publisher ros_publisher);
  geometry_msgs::TransformStamped ros_transform_stamped_msg_;

  // TWIST STAMPED
  void GzTwistStampedMsgCallback(GzTwistStampedMsgPtr& gz_twist_stamped_msg,
                                 ros::Publisher ros_publisher);
  geometry_msgs::TwistStamped ros_twist_stamped_msg_;

  // WIND SPEED
  void GzWindSpeedMsgCallback(GzWindSpeedMsgPtr& gz_wind_speed_msg,
                              ros::Publisher ros_publisher);
  rotors_comm::WindSpeed ros_wind_speed_msg_;

  // WRENCH STAMPED
  void GzWrenchStampedMsgCallback(GzWrenchStampedMsgPtr& gz_wrench_stamped_msg,
                                  ros::Publisher ros_publisher);
  geometry_msgs::WrenchStamped ros_wrench_stamped_msg_;

  // ============================================ //
  // ===== ROS->GAZEBO CALLBACKS/CONVERTERS ===== //
  // ============================================ //

  // ACTUATORS (change name??? motor control? motor speed?)
  void RosActuatorsMsgCallback(
      const mav_msgs::ActuatorsConstPtr& ros_actuators_msg_ptr,
      gazebo::transport::PublisherPtr gz_publisher_ptr);

  // COMMAND MOTOR SPEED (this is the same as ACTUATORS!, merge???)
  void RosCommandMotorSpeedMsgCallback(
      const mav_msgs::ActuatorsConstPtr& ros_command_motor_speed_msg_ptr,
      gazebo::transport::PublisherPtr gz_publisher_ptr);

  // ROLL PITCH YAWRATE THRUST
  void RosRollPitchYawrateThrustMsgCallback(
      const mav_msgs::RollPitchYawrateThrustConstPtr&
          ros_roll_pitch_yawrate_thrust_msg_ptr,
      gazebo::transport::PublisherPtr gz_publisher_ptr);

  // WIND SPEED
  void RosWindSpeedMsgCallback(
      const rotors_comm::WindSpeedConstPtr& ros_wind_speed_msg_ptr,
      gazebo::transport::PublisherPtr gz_publisher_ptr);

  // ============================================ //
  // ====== TRANSFORM BROADCASTER RELATED ======= //
  // ============================================ //

  transport::SubscriberPtr gz_broadcast_transform_sub_;

  /// \brief    This is a special-case callback which listens for Gazebo
  ///           "Transform" messages. Upon receiving one it
  ///           broadcasts the transform on the ROS system (using
  ///           transform_broadcast()).
  void GzBroadcastTransformMsgCallback(
      GzTransformStampedWithFrameIdsMsgPtr& broadcast_transform_msg);

  tf::Transform tf_;
  tf::TransformBroadcaster transform_broadcaster_;
};

}  // namespace gazebo

#endif  // #ifndef ROTORS_GAZEBO_PLUGINS_MSG_INTERFACE_PLUGIN_H

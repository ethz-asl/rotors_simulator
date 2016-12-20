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
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <mav_msgs/default_topics.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>


#include "gazebo/msgs/msgs.hh"

// GAZEBO MSG TYPES
#include "ConnectGazeboToRosTopic.pb.h"
#include "ConnectRosToGazeboTopic.pb.h"

#include "Actuators.pb.h"
#include "JointState.pb.h"
#include "MagneticField.pb.h"
#include "NavSatFix.pb.h"
#include "Odometry.pb.h"
#include "Pose.pb.h"
#include "PoseWithCovarianceStamped.pb.h"
#include "PositionStamped.pb.h"
#include "SensorImu.pb.h"
#include "TransformStamped.pb.h"
#include "TwistStamped.pb.h"

// ROS MSG TYPES
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "rotors_gazebo_plugins/common.h"

namespace gazebo {

// typedef's to make life easier
typedef const boost::shared_ptr<const gz_std_msgs::ConnectGazeboToRosTopic> GzConnectGazeboToRosTopicMsgPtr;

typedef const boost::shared_ptr<const sensor_msgs::msgs::Actuators> GzActuatorsMsgPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Imu> GzImuPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::JointState> GzJointStateMsgPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::MagneticField> GzMagneticFieldMsgPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::NavSatFix> GzNavSatFixPtr;
typedef const boost::shared_ptr<const gz_geometry_msgs::Odometry> GzOdometryMsgPtr;
typedef const boost::shared_ptr<const gz_geometry_msgs::Pose> GzPoseMsgPtr;
typedef const boost::shared_ptr<const gz_geometry_msgs::PoseWithCovarianceStamped> GzPoseWithCovarianceStampedMsgPtr;
typedef const boost::shared_ptr<const gz_geometry_msgs::PositionStamped> GzPositionStampedMsgPtr;
typedef const boost::shared_ptr<const gz_geometry_msgs::TransformStamped> GzTransformStampedMsgPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::TwistStamped> GzTwistStampedMsgPtr;

//! @brief    Message interface plugin for Gazebo.
//! @details  Interfaces to both ROS and MAVlink.
class GazeboRosInterfacePlugin : public ModelPlugin {
 public:

  //! @brief    The message types that GazeboRosInterfacePlugin supports.
  //! @details  For each one of these, GazeboRosInterfacePlugin knows how to convert the message
  //!           from a Gazebo message to a ROS message.
  //! @warning  If you add another enum here, make sure to add a corresponding case block
  //!           to the switch statement in ConnectToRos().
//  enum class SupportedMsgTypes {
//    ACTUATORS,
//    IMU,
//    JOINT_STATE,
//    MAGNETIC_FIELD,
//    NAV_SAT_FIX,
//    ODOMETRY,
//    POSE,
//    POSE_WITH_COVARIANCE_STAMPED,
//    POSITION_STAMPED,
//    TRANSFORM_STAMPED,
//    TWIST_STAMPED,
//  };

  GazeboRosInterfacePlugin();
  ~GazeboRosInterfacePlugin();

  //! @brief    Call this to connect a Gazebo topic to a ROS topic.
  //! @details  Any messages published on the specified Gazebo topic will be converted into a ROS message
  //!           and then published on the ROS framework.
//  void ConnectToRos(std::string gazeboTopicName, std::string rosTopicName, SupportedMsgTypes msgType);

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// @brief  	This gets called by the world update start event.
  /// @details	Calculates IMU parameters and then publishes one IMU message.
  void OnUpdate(const common::UpdateInfo&);

 private:

  template <typename M, typename N>
  void ConnectHelper(
      void(GazeboRosInterfacePlugin::*fp)(const boost::shared_ptr<M const> &, ros::Publisher),
      GazeboRosInterfacePlugin * ptr,
      std::string gazeboTopicName,
      std::string rosTopicName,
      transport::NodePtr gz_node_handle);

  std::vector<gazebo::transport::SubscriberPtr> subscriberPtrs_;

  std::string namespace_;

  /// @brief  Handle for the Gazebo node.
  transport::NodePtr gz_node_handle_;

  /// @brief  Handle for the ROS node.
  ros::NodeHandle* ros_node_handle_;


  std::string link_name_;


  // Pointer to the world
  physics::WorldPtr world_;
  // Pointer to the model
  physics::ModelPtr model_;
  // Pointer to the link
//  physics::LinkPtr link_;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection_;

  common::Time last_time_;

  // ============================================ //
  // ========== CONNECT TO ROS MESSAGES ========= //
  // ============================================ //

  transport::SubscriberPtr gz_connect_gazebo_to_ros_topic_sub_;
  void GzConnectGazeboToRosTopicMsgCallback(GzConnectGazeboToRosTopicMsgPtr& gz_connect_gazebo_to_ros_topic_msg);

  // ============================================ //
  // ============= ACTUATORS MESSAGES =========== //
  // ============================================ //

  //transport::SubscriberPtr gz_actuator_sub_;                            ///< Listens to Gazebo messages.
  void GzActuatorsMsgCallback(GzActuatorsMsgPtr& gz_actuators_msg, ros::Publisher ros_publisher);     ///< Callback for when Gazebo message is received.
  //ros::Publisher ros_actuators_pub_;                                    ///< Publishes ROS messages.
  mav_msgs::ActuatorsPtr ros_actuators_msg_;                            ///< Persistant msg object to prevent mem alloc everytime Gazebo message is converted to ROS message.

  // ============================================ //
  // ==================== IMU =================== //
  // ============================================ //

  //transport::SubscriberPtr gz_imu_sub_;         ///< Listens to Gazebo messages.
  void GzImuMsgCallback(GzImuPtr& gz_imu_msg, ros::Publisher ros_publisher);     ///< Callback for when Gazebo message is received.
  //ros::Publisher ros_imu_pub_;                  ///< Publishes ROS messages.
  sensor_msgs::Imu ros_imu_msg_;                ///< Persistant msg object to prevent mem alloc everytime Gazebo message is converted to ROS message.

  // ============================================ //
  // =========== JOINT STATE MESSAGES =========== //
  // ============================================ //

//  transport::SubscriberPtr gz_joing_state_sub_;                            ///< Listens to Gazebo messages.
  void GzJointStateMsgCallback(GzJointStateMsgPtr& gz_joint_state_msg, ros::Publisher ros_publisher);     ///< Callback for when Gazebo message is received.
//  ros::Publisher ros_joint_state_pub_;                                    ///< Publishes ROS messages.
  sensor_msgs::JointStatePtr ros_joint_state_msg_;                            ///< Persistant msg object to prevent mem alloc everytime Gazebo message is converted to ROS message.

  // ============================================ //
  // ========== MAGNETIC FIELD MESSAGES ========= //
  // ============================================ //

//  transport::SubscriberPtr gz_magnetic_field_sub_;                                  ///< Listens to Gazebo messages.
  void GzMagneticFieldMsgCallback(GzMagneticFieldMsgPtr& gz_magnetic_field_msg, ros::Publisher ros_publisher);    ///< Callback for when Gazebo message is received.
//  ros::Publisher ros_magnetic_field_pub_;                                           ///< Publishes ROS messages.
  sensor_msgs::MagneticField ros_magnetic_field_msg_;                               ///< Persistant msg object to prevent mem alloc everytime Gazebo message is converted to ROS message.

  // ============================================ //
  // ============= NAV SAT FIX (GPS) ============ //
  // ============================================ //

//  transport::SubscriberPtr gz_nav_sat_fix_sub_;                   ///< Listens to Gazebo messages.
  void GzNavSatFixCallback(GzNavSatFixPtr& gz_nav_sat_fix_msg, ros::Publisher ros_publisher);   ///< Callback for when Gazebo message is received.
//  ros::Publisher ros_nav_sat_fix_pub_;                            ///< Publishes ROS messages.
  sensor_msgs::NavSatFix ros_nav_sat_fix_msg_;                    ///< Persistant msg object to prevent mem alloc everytime Gazebo message is converted to ROS message.

  // ============================================ //
  // ============== ODOMETRY MESSAGES =========== //
  // ============================================ //

//  transport::SubscriberPtr gz_odometry_sub_;                            ///< Listens to Gazebo messages.
  void GzOdometryMsgCallback(GzOdometryMsgPtr& gz_odometry_msg, ros::Publisher ros_publisher);     ///< Callback for when Gazebo message is received.
//  ros::Publisher ros_odometry_pub_;                                    ///< Publishes ROS messages.
  nav_msgs::Odometry ros_odometry_msg_;                            ///< Persistant msg object to prevent mem alloc everytime Gazebo message is converted to ROS message.

  // ============================================ //
  // ================ POSE MESSAGES ============= //
  // ============================================ //

  void GzPoseMsgCallback(GzPoseMsgPtr& gz_pose_msg, ros::Publisher ros_publisher);    //!< Callback for when Gazebo message is received.
  geometry_msgs::Pose ros_pose_msg_;                                              //!< Persistant msg object to prevent mem alloc everytime Gazebo message is converted to ROS message.

  // ============================================ //
  // === POSE WITH COVARIANCE STAMPED MESSAGES == //
  // ============================================ //

  void GzPoseWithCovarianceStampedMsgCallback(
      GzPoseWithCovarianceStampedMsgPtr& gz_pose_with_covariance_stamped_msg, ros::Publisher ros_publisher);    //!< Callback for when Gazebo message is received.
  geometry_msgs::Pose ros_pose_with_covariance_stamped_msg_;                                              //!< Persistant msg object to prevent mem alloc everytime Gazebo message is converted to ROS message.

  // ============================================ //
  // ========= POSITION STAMPED MESSAGES ======== //
  // ============================================ //

  void GzPositionStampedMsgCallback(
      GzPositionStampedMsgPtr& gz_position_stamped_msg,     //!< Callback for when Gazebo message is received.
      ros::Publisher ros_publisher);                        //!< Persistant msg object to prevent mem alloc everytime Gazebo message is converted to ROS message.
  geometry_msgs::Point ros_position_stamped_msg_;

  // ============================================ //
  // ======== TRANSFORM STAMPED MESSAGES ======== //
  // ============================================ //

  void GzTransformStampedMsgCallback(
      GzTransformStampedMsgPtr& gz_transform_stamped_msg,     //!< Callback for when Gazebo message is received.
      ros::Publisher ros_publisher);                          //!< Persistant msg object to prevent mem alloc everytime Gazebo message is converted to ROS message.
  geometry_msgs::TransformStamped ros_transform_stamped_msg_;

  // ============================================ //
  // =========== TWIST STAMPED MESSAGES ========= //
  // ============================================ //

//  transport::SubscriberPtr gz_odometry_sub_;                            ///< Listens to Gazebo messages.
  void GzTwistStampedMsgCallback(GzTwistStampedMsgPtr& gz_twist_stamped_msg, ros::Publisher ros_publisher);     ///< Callback for when Gazebo message is received.
//  ros::Publisher ros_odometry_pub_;                                    ///< Publishes ROS messages.
  geometry_msgs::TwistStamped ros_twist_stamped_msg_;                            ///< Persistant msg object to prevent mem alloc everytime Gazebo message is converted to ROS message.


};

}  // namespace gazebo

#endif // #ifndef ROTORS_GAZEBO_PLUGINS_MSG_INTERFACE_PLUGIN_H

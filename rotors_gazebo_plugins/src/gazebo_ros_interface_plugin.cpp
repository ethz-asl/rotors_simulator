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

// MODULE
#include <rotors_gazebo_plugins/gazebo_ros_interface_plugin.h>

// SYSTEM
#include <stdio.h>
#include <chrono>
#include <cmath>
#include <iostream>

// 3RD PARTY
#include <std_msgs/Header.h>
#include <boost/bind.hpp>

namespace gazebo {

GazeboRosInterfacePlugin::GazeboRosInterfacePlugin()
    : WorldPlugin(), gz_node_handle_(0), ros_node_handle_(0) {}

GazeboRosInterfacePlugin::~GazeboRosInterfacePlugin() {

  // Shutdown and delete ROS node handle
  if (ros_node_handle_) {
    ros_node_handle_->shutdown();
    delete ros_node_handle_;
  }
}

void GazeboRosInterfacePlugin::Load(physics::WorldPtr _world,
                                    sdf::ElementPtr _sdf) {
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  /// \brief    Store the pointer to the model.
  world_ = _world;

  // namespace_.clear();

  //==============================================//
  //========== READ IN PARAMS FROM SDF ===========//
  //==============================================//

  /*if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "Please specify a robotNamespace.\n";
  gzdbg << "namespace_ = \"" << namespace_ << "\"." << std::endl;*/

  // Get Gazebo node handle
  gz_node_handle_ = transport::NodePtr(new transport::Node());
  // gz_node_handle_->Init(namespace_);
  gz_node_handle_->Init();

  // Get ROS node handle
  // ros_node_handle_ = new ros::NodeHandle(namespace_);
  ros_node_handle_ = new ros::NodeHandle();

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosInterfacePlugin::OnUpdate, this, _1));

  // ============================================ //
  // === CONNECT GAZEBO TO ROS MESSAGES SETUP === //
  // ============================================ //

  gz_connect_gazebo_to_ros_topic_sub_ = gz_node_handle_->Subscribe(
      "~/" + kConnectGazeboToRosSubtopic,
      &GazeboRosInterfacePlugin::GzConnectGazeboToRosTopicMsgCallback, this);

  // ============================================ //
  // === CONNECT ROS TO GAZEBO MESSAGES SETUP === //
  // ============================================ //

  gz_connect_ros_to_gazebo_topic_sub_ = gz_node_handle_->Subscribe(
      "~/" + kConnectRosToGazeboSubtopic,
      &GazeboRosInterfacePlugin::GzConnectRosToGazeboTopicMsgCallback, this);

  // ============================================ //
  // ===== BROADCAST TRANSFORM MESSAGE SETUP ==== //
  // ============================================ //

  gz_broadcast_transform_sub_ = gz_node_handle_->Subscribe(
      "~/" + kBroadcastTransformSubtopic,
      &GazeboRosInterfacePlugin::GzBroadcastTransformMsgCallback, this);
}

void GazeboRosInterfacePlugin::OnUpdate(const common::UpdateInfo& _info) {
  // Do nothing
  // This plugins actions are all executed through message callbacks.
}

/// \brief      A helper class that provides storage for additional parameters
///             that are inserted into the callback.
/// \details
///   GazeboMsgT  The type of the message that will be subscribed to the Gazebo
///   framework.
template <typename GazeboMsgT>
struct ConnectHelperStorage {
  /// \brief    Pointer to the ROS interface plugin class.
  GazeboRosInterfacePlugin* ptr;

  /// \brief    Function pointer to the subscriber callback with additional
  /// parameters.
  void (GazeboRosInterfacePlugin::*fp)(
      const boost::shared_ptr<GazeboMsgT const>&, ros::Publisher ros_publisher);

  /// \brief    The ROS publisher that is passed into the modified callback.
  ros::Publisher ros_publisher;

  /// \brief    This is what gets passed into the Gazebo Subscribe method as a
  ///           callback, and hence can only
  ///           have one parameter (note boost::bind() does not work with the
  ///           current Gazebo Subscribe() definitions).
  void callback(const boost::shared_ptr<GazeboMsgT const>& msg_ptr) {
    (ptr->*fp)(msg_ptr, ros_publisher);
  }
};

template <typename GazeboMsgT, typename RosMsgT>
void GazeboRosInterfacePlugin::ConnectHelper(
    void (GazeboRosInterfacePlugin::*fp)(
        const boost::shared_ptr<GazeboMsgT const>&, ros::Publisher),
    GazeboRosInterfacePlugin* ptr, std::string gazeboNamespace,
    std::string gazeboTopicName, std::string rosTopicName,
    transport::NodePtr gz_node_handle) {
  // One map will be created for each Gazebo message type
  static std::map<std::string, ConnectHelperStorage<GazeboMsgT> > callback_map;

  // Create ROS publisher
  ros::Publisher ros_publisher =
      ros_node_handle_->advertise<RosMsgT>(rosTopicName, 1);

  auto callback_entry = callback_map.emplace(
      gazeboTopicName,
      ConnectHelperStorage<GazeboMsgT>{ptr, fp, ros_publisher});

  // Check if element was already present
  if (!callback_entry.second)
    gzerr << "Tried to add element to map but the gazebo topic name was "
             "already present in map."
          << std::endl;

  // Create subscriber
  gazebo::transport::SubscriberPtr subscriberPtr;
  subscriberPtr = gz_node_handle->Subscribe(
      gazeboTopicName, &ConnectHelperStorage<GazeboMsgT>::callback,
      &callback_entry.first->second);

  // Save a reference to the subscriber pointer so subscriber
  // won't be deleted.
  subscriberPtrs_.push_back(subscriberPtr);
}

void GazeboRosInterfacePlugin::GzConnectGazeboToRosTopicMsgCallback(
    GzConnectGazeboToRosTopicMsgPtr& gz_connect_gazebo_to_ros_topic_msg) {
  if (kPrintOnMsgCallback) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  const std::string gazeboNamespace =
      "";  // gz_connect_gazebo_to_ros_topic_msg->gazebo_namespace();
  const std::string gazeboTopicName =
      gz_connect_gazebo_to_ros_topic_msg->gazebo_topic();
  const std::string rosTopicName =
      gz_connect_gazebo_to_ros_topic_msg->ros_topic();

  gzdbg << "Connecting Gazebo topic \"" << gazeboTopicName
        << "\" to ROS topic \"" << rosTopicName << "\"." << std::endl;

  switch (gz_connect_gazebo_to_ros_topic_msg->msgtype()) {
    case gz_std_msgs::ConnectGazeboToRosTopic::ACTUATORS:
      ConnectHelper<gz_sensor_msgs::Actuators, mav_msgs::Actuators>(
          &GazeboRosInterfacePlugin::GzActuatorsMsgCallback, this,
          gazeboNamespace, gazeboTopicName, rosTopicName, gz_node_handle_);
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::FLOAT_32:
      ConnectHelper<gz_std_msgs::Float32, std_msgs::Float32>(
          &GazeboRosInterfacePlugin::GzFloat32MsgCallback, this,
          gazeboNamespace, gazeboTopicName, rosTopicName, gz_node_handle_);
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::FLUID_PRESSURE:
      ConnectHelper<gz_sensor_msgs::FluidPressure, sensor_msgs::FluidPressure>(
          &GazeboRosInterfacePlugin::GzFluidPressureMsgCallback, this,
          gazeboNamespace, gazeboTopicName, rosTopicName, gz_node_handle_);
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::IMU:
      ConnectHelper<gz_sensor_msgs::Imu, sensor_msgs::Imu>(
          &GazeboRosInterfacePlugin::GzImuMsgCallback, this, gazeboNamespace,
          gazeboTopicName, rosTopicName, gz_node_handle_);
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::JOINT_STATE:
      ConnectHelper<gz_sensor_msgs::JointState, sensor_msgs::JointState>(
          &GazeboRosInterfacePlugin::GzJointStateMsgCallback, this,
          gazeboNamespace, gazeboTopicName, rosTopicName, gz_node_handle_);
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::MAGNETIC_FIELD:
      ConnectHelper<gz_sensor_msgs::MagneticField, sensor_msgs::MagneticField>(
          &GazeboRosInterfacePlugin::GzMagneticFieldMsgCallback, this,
          gazeboNamespace, gazeboTopicName, rosTopicName, gz_node_handle_);
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::NAV_SAT_FIX:
      ConnectHelper<gz_sensor_msgs::NavSatFix, sensor_msgs::NavSatFix>(
          &GazeboRosInterfacePlugin::GzNavSatFixCallback, this, gazeboNamespace,
          gazeboTopicName, rosTopicName, gz_node_handle_);
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::POSE:
      ConnectHelper<gazebo::msgs::Pose, geometry_msgs::Pose>(
          &GazeboRosInterfacePlugin::GzPoseMsgCallback, this, gazeboNamespace,
          gazeboTopicName, rosTopicName, gz_node_handle_);
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::POSE_WITH_COVARIANCE_STAMPED:
      ConnectHelper<gz_geometry_msgs::PoseWithCovarianceStamped,
                    geometry_msgs::PoseWithCovarianceStamped>(
          &GazeboRosInterfacePlugin::GzPoseWithCovarianceStampedMsgCallback,
          this, gazeboNamespace, gazeboTopicName, rosTopicName,
          gz_node_handle_);
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::ODOMETRY:
      ConnectHelper<gz_geometry_msgs::Odometry, nav_msgs::Odometry>(
          &GazeboRosInterfacePlugin::GzOdometryMsgCallback, this,
          gazeboNamespace, gazeboTopicName, rosTopicName, gz_node_handle_);
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::TRANSFORM_STAMPED:
      ConnectHelper<gz_geometry_msgs::TransformStamped,
                    geometry_msgs::TransformStamped>(
          &GazeboRosInterfacePlugin::GzTransformStampedMsgCallback, this,
          gazeboNamespace, gazeboTopicName, rosTopicName, gz_node_handle_);
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::TWIST_STAMPED:
      ConnectHelper<gz_geometry_msgs::TwistStamped,
                    geometry_msgs::TwistStamped>(
          &GazeboRosInterfacePlugin::GzTwistStampedMsgCallback, this,
          gazeboNamespace, gazeboTopicName, rosTopicName, gz_node_handle_);
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::VECTOR_3D_STAMPED:
      ConnectHelper<gz_geometry_msgs::Vector3dStamped,
                    geometry_msgs::PointStamped>(
          &GazeboRosInterfacePlugin::GzVector3dStampedMsgCallback, this,
          gazeboNamespace, gazeboTopicName, rosTopicName, gz_node_handle_);
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::WIND_SPEED:
      ConnectHelper<gz_mav_msgs::WindSpeed,
                    rotors_comm::WindSpeed>(
          &GazeboRosInterfacePlugin::GzWindSpeedMsgCallback, this,
          gazeboNamespace, gazeboTopicName, rosTopicName, gz_node_handle_);
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::WRENCH_STAMPED:
      ConnectHelper<gz_geometry_msgs::WrenchStamped,
                    geometry_msgs::WrenchStamped>(
          &GazeboRosInterfacePlugin::GzWrenchStampedMsgCallback, this,
          gazeboNamespace, gazeboTopicName, rosTopicName, gz_node_handle_);
      break;
    default:
      gzthrow("ConnectGazeboToRosTopic message type with enum val = "
              << gz_connect_gazebo_to_ros_topic_msg->msgtype()
              << " is not supported by GazeboRosInterfacePlugin.");
  }

  gzdbg << __FUNCTION__ << "() finished." << std::endl;
}

void GazeboRosInterfacePlugin::GzConnectRosToGazeboTopicMsgCallback(
    GzConnectRosToGazeboTopicMsgPtr& gz_connect_ros_to_gazebo_topic_msg) {
  if (kPrintOnMsgCallback) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  static std::vector<ros::Subscriber> ros_subscribers;

  switch (gz_connect_ros_to_gazebo_topic_msg->msgtype()) {
    case gz_std_msgs::ConnectRosToGazeboTopic::ACTUATORS: {
      gazebo::transport::PublisherPtr gz_publisher_ptr =
          gz_node_handle_->Advertise<gz_sensor_msgs::Actuators>(
              gz_connect_ros_to_gazebo_topic_msg->gazebo_topic(), 1);

      // Create ROS subscriber.
      ros::Subscriber ros_subscriber =
          ros_node_handle_->subscribe<mav_msgs::Actuators>(
              gz_connect_ros_to_gazebo_topic_msg->ros_topic(), 1,
              boost::bind(&GazeboRosInterfacePlugin::RosActuatorsMsgCallback,
                          this, _1, gz_publisher_ptr));

      // Save reference to the ROS subscriber so callback will continue to be
      // called.
      ros_subscribers.push_back(ros_subscriber);

      break;
    }
    case gz_std_msgs::ConnectRosToGazeboTopic::COMMAND_MOTOR_SPEED: {
      gazebo::transport::PublisherPtr gz_publisher_ptr =
          gz_node_handle_->Advertise<gz_mav_msgs::CommandMotorSpeed>(
              gz_connect_ros_to_gazebo_topic_msg->gazebo_topic(), 1);

      // Create ROS subscriber.
      ros::Subscriber ros_subscriber =
          ros_node_handle_->subscribe<mav_msgs::Actuators>(
              gz_connect_ros_to_gazebo_topic_msg->ros_topic(), 1,
              boost::bind(
                  &GazeboRosInterfacePlugin::RosCommandMotorSpeedMsgCallback,
                  this, _1, gz_publisher_ptr));

      // Save reference to the ROS subscriber so callback will continue to be
      // called.
      ros_subscribers.push_back(ros_subscriber);

      break;
    }
    case gz_std_msgs::ConnectRosToGazeboTopic::ROLL_PITCH_YAWRATE_THRUST: {
      gazebo::transport::PublisherPtr gz_publisher_ptr =
          gz_node_handle_->Advertise<gz_mav_msgs::RollPitchYawrateThrust>(
              gz_connect_ros_to_gazebo_topic_msg->gazebo_topic(), 1);

      // Create ROS subscriber.
      ros::Subscriber ros_subscriber =
          ros_node_handle_->subscribe<mav_msgs::RollPitchYawrateThrust>(
              gz_connect_ros_to_gazebo_topic_msg->ros_topic(), 1,
              boost::bind(
                  &GazeboRosInterfacePlugin::
                      RosRollPitchYawrateThrustMsgCallback,
                  this, _1, gz_publisher_ptr));

      // Save reference to the ROS subscriber so callback will continue to be
      // called.
      ros_subscribers.push_back(ros_subscriber);

      break;
    }
    case gz_std_msgs::ConnectRosToGazeboTopic::WIND_SPEED: {
      gazebo::transport::PublisherPtr gz_publisher_ptr =
          gz_node_handle_->Advertise<gz_mav_msgs::WindSpeed>(
              gz_connect_ros_to_gazebo_topic_msg->gazebo_topic(), 1);

      // Create ROS subscriber.
      ros::Subscriber ros_subscriber =
          ros_node_handle_->subscribe<rotors_comm::WindSpeed>(
              gz_connect_ros_to_gazebo_topic_msg->ros_topic(), 1,
              boost::bind(&GazeboRosInterfacePlugin::RosWindSpeedMsgCallback,
                          this, _1, gz_publisher_ptr));

      // Save reference to the ROS subscriber so callback will continue to be
      // called.
      ros_subscribers.push_back(ros_subscriber);

      break;
    }
    default: {
      gzthrow("ConnectRosToGazeboTopic message type with enum val = "
              << gz_connect_ros_to_gazebo_topic_msg->msgtype()
              << " is not supported by GazeboRosInterfacePlugin.");
    }
  }
}

//===========================================================================//
//==================== HELPER METHODS FOR MSG CONVERSION ====================//
//===========================================================================//

void GazeboRosInterfacePlugin::ConvertHeaderGzToRos(
    const gz_std_msgs::Header& gz_header,
    std_msgs::Header_<std::allocator<void> >* ros_header) {
  ros_header->stamp.sec = gz_header.stamp().sec();
  ros_header->stamp.nsec = gz_header.stamp().nsec();
  ros_header->frame_id = gz_header.frame_id();
}

void GazeboRosInterfacePlugin::ConvertHeaderRosToGz(
    const std_msgs::Header_<std::allocator<void> >& ros_header,
    gz_std_msgs::Header* gz_header) {
  gz_header->mutable_stamp()->set_sec(ros_header.stamp.sec);
  gz_header->mutable_stamp()->set_nsec(ros_header.stamp.nsec);
  gz_header->set_frame_id(ros_header.frame_id);
}

//===========================================================================//
//================ GAZEBO -> ROS MSG CALLBACKS/CONVERTERS ===================//
//===========================================================================//

void GazeboRosInterfacePlugin::GzActuatorsMsgCallback(
    GzActuatorsMsgPtr& gz_actuators_msg, ros::Publisher ros_publisher) {
  // We need to convert the Acutuators message from a Gazebo message to a
  // ROS message and then publish it to the ROS framework

  ConvertHeaderGzToRos(gz_actuators_msg->header(), &ros_actuators_msg_.header);

  ros_actuators_msg_.angular_velocities.resize(
      gz_actuators_msg->angular_velocities_size());
  for (int i = 0; i < gz_actuators_msg->angular_velocities_size(); i++) {
    ros_actuators_msg_.angular_velocities[i] =
        gz_actuators_msg->angular_velocities(i);
  }

  // Publish to ROS.
  ros_publisher.publish(ros_actuators_msg_);
}

void GazeboRosInterfacePlugin::GzFloat32MsgCallback(
    GzFloat32MsgPtr& gz_float_32_msg, ros::Publisher ros_publisher) {
  // Convert Gazebo message to ROS message
  ros_float_32_msg_.data = gz_float_32_msg->data();

  // Publish to ROS
  ros_publisher.publish(ros_float_32_msg_);
}

void GazeboRosInterfacePlugin::GzFluidPressureMsgCallback(
    GzFluidPressureMsgPtr &gz_fluid_pressure_msg,
    ros::Publisher ros_publisher) {
  // We need to convert from a Gazebo message to a ROS message,
  // and then forward the FluidPressure message onto ROS.

  ConvertHeaderGzToRos(gz_fluid_pressure_msg->header(),
                       &ros_fluid_pressure_msg_.header);

  ros_fluid_pressure_msg_.fluid_pressure =
      gz_fluid_pressure_msg->fluid_pressure();

  ros_fluid_pressure_msg_.variance = gz_fluid_pressure_msg->variance();

  // Publish to ROS.
  ros_publisher.publish(ros_fluid_pressure_msg_);
}

void GazeboRosInterfacePlugin::GzImuMsgCallback(GzImuPtr& gz_imu_msg,
                                                ros::Publisher ros_publisher) {
  // We need to convert from a Gazebo message to a ROS message,
  // and then forward the IMU message onto ROS

  ConvertHeaderGzToRos(gz_imu_msg->header(), &ros_imu_msg_.header);

  ros_imu_msg_.orientation.x = gz_imu_msg->orientation().x();
  ros_imu_msg_.orientation.y = gz_imu_msg->orientation().y();
  ros_imu_msg_.orientation.z = gz_imu_msg->orientation().z();
  ros_imu_msg_.orientation.w = gz_imu_msg->orientation().w();

  // Orientation covariance should have 9 elements, and both the Gazebo and ROS
  // arrays should be the same size!
  GZ_ASSERT(gz_imu_msg->orientation_covariance_size() == 9,
            "The Gazebo IMU message does not have 9 orientation covariance "
            "elements.");
  GZ_ASSERT(
      ros_imu_msg_.orientation_covariance.size() == 9,
      "The ROS IMU message does not have 9 orientation covariance elements.");
  for (int i = 0; i < gz_imu_msg->orientation_covariance_size(); i++) {
    ros_imu_msg_.orientation_covariance[i] =
        gz_imu_msg->orientation_covariance(i);
  }

  ros_imu_msg_.angular_velocity.x = gz_imu_msg->angular_velocity().x();
  ros_imu_msg_.angular_velocity.y = gz_imu_msg->angular_velocity().y();
  ros_imu_msg_.angular_velocity.z = gz_imu_msg->angular_velocity().z();

  GZ_ASSERT(gz_imu_msg->angular_velocity_covariance_size() == 9,
            "The Gazebo IMU message does not have 9 angular velocity "
            "covariance elements.");
  GZ_ASSERT(ros_imu_msg_.angular_velocity_covariance.size() == 9,
            "The ROS IMU message does not have 9 angular velocity covariance "
            "elements.");
  for (int i = 0; i < gz_imu_msg->angular_velocity_covariance_size(); i++) {
    ros_imu_msg_.angular_velocity_covariance[i] =
        gz_imu_msg->angular_velocity_covariance(i);
  }

  ros_imu_msg_.linear_acceleration.x = gz_imu_msg->linear_acceleration().x();
  ros_imu_msg_.linear_acceleration.y = gz_imu_msg->linear_acceleration().y();
  ros_imu_msg_.linear_acceleration.z = gz_imu_msg->linear_acceleration().z();

  GZ_ASSERT(gz_imu_msg->linear_acceleration_covariance_size() == 9,
            "The Gazebo IMU message does not have 9 linear acceleration "
            "covariance elements.");
  GZ_ASSERT(ros_imu_msg_.linear_acceleration_covariance.size() == 9,
            "The ROS IMU message does not have 9 linear acceleration "
            "covariance elements.");
  for (int i = 0; i < gz_imu_msg->linear_acceleration_covariance_size(); i++) {
    ros_imu_msg_.linear_acceleration_covariance[i] =
        gz_imu_msg->linear_acceleration_covariance(i);
  }

  // Publish to ROS.
  ros_publisher.publish(ros_imu_msg_);
}

void GazeboRosInterfacePlugin::GzJointStateMsgCallback(
    GzJointStateMsgPtr& gz_joint_state_msg, ros::Publisher ros_publisher) {
  ConvertHeaderGzToRos(gz_joint_state_msg->header(),
                       &ros_joint_state_msg_.header);

  ros_joint_state_msg_.name.resize(gz_joint_state_msg->name_size());
  for (int i = 0; i < gz_joint_state_msg->name_size(); i++) {
    ros_joint_state_msg_.name[i] = gz_joint_state_msg->name(i);
  }

  ros_joint_state_msg_.position.resize(gz_joint_state_msg->position_size());
  for (int i = 0; i < gz_joint_state_msg->position_size(); i++) {
    ros_joint_state_msg_.position[i] = gz_joint_state_msg->position(i);
  }

  // Publish to ROS.
  ros_publisher.publish(ros_joint_state_msg_);
}

void GazeboRosInterfacePlugin::GzMagneticFieldMsgCallback(
    GzMagneticFieldMsgPtr& gz_magnetic_field_msg,
    ros::Publisher ros_publisher) {
  // We need to convert from a Gazebo message to a ROS message,
  // and then forward the MagneticField message onto ROS

  ConvertHeaderGzToRos(gz_magnetic_field_msg->header(),
                       &ros_magnetic_field_msg_.header);

  ros_magnetic_field_msg_.magnetic_field.x =
      gz_magnetic_field_msg->magnetic_field().x();
  ros_magnetic_field_msg_.magnetic_field.y =
      gz_magnetic_field_msg->magnetic_field().y();
  ros_magnetic_field_msg_.magnetic_field.z =
      gz_magnetic_field_msg->magnetic_field().z();

  // Position covariance should have 9 elements, and both the Gazebo and ROS
  // arrays should be the same size!
  GZ_ASSERT(gz_magnetic_field_msg->magnetic_field_covariance_size() == 9,
            "The Gazebo MagneticField message does not have 9 magnetic field "
            "covariance elements.");
  GZ_ASSERT(ros_magnetic_field_msg_.magnetic_field_covariance.size() == 9,
            "The ROS MagneticField message does not have 9 magnetic field "
            "covariance elements.");
  for (int i = 0; i < gz_magnetic_field_msg->magnetic_field_covariance_size();
       i++) {
    ros_magnetic_field_msg_.magnetic_field_covariance[i] =
        gz_magnetic_field_msg->magnetic_field_covariance(i);
  }

  // Publish to ROS.
  ros_publisher.publish(ros_magnetic_field_msg_);
}

void GazeboRosInterfacePlugin::GzNavSatFixCallback(
    GzNavSatFixPtr& gz_nav_sat_fix_msg, ros::Publisher ros_publisher) {
  // We need to convert from a Gazebo message to a ROS message, and then forward
  // the NavSatFix message to ROS.

  ConvertHeaderGzToRos(gz_nav_sat_fix_msg->header(),
                       &ros_nav_sat_fix_msg_.header);

  switch (gz_nav_sat_fix_msg->service()) {
    case gz_sensor_msgs::NavSatFix::SERVICE_GPS:
      ros_nav_sat_fix_msg_.status.service =
          sensor_msgs::NavSatStatus::SERVICE_GPS;
      break;
    case gz_sensor_msgs::NavSatFix::SERVICE_GLONASS:
      ros_nav_sat_fix_msg_.status.service =
          sensor_msgs::NavSatStatus::SERVICE_GLONASS;
      break;
    case gz_sensor_msgs::NavSatFix::SERVICE_COMPASS:
      ros_nav_sat_fix_msg_.status.service =
          sensor_msgs::NavSatStatus::SERVICE_COMPASS;
      break;
    case gz_sensor_msgs::NavSatFix::SERVICE_GALILEO:
      ros_nav_sat_fix_msg_.status.service =
          sensor_msgs::NavSatStatus::SERVICE_GALILEO;
      break;
    default:
      gzthrow(
          "Specific value of enum type gz_sensor_msgs::NavSatFix::Service is "
          "not yet supported.");
  }

  switch (gz_nav_sat_fix_msg->status()) {
    case gz_sensor_msgs::NavSatFix::STATUS_NO_FIX:
      ros_nav_sat_fix_msg_.status.status =
          sensor_msgs::NavSatStatus::STATUS_NO_FIX;
      break;
    case gz_sensor_msgs::NavSatFix::STATUS_FIX:
      ros_nav_sat_fix_msg_.status.status =
          sensor_msgs::NavSatStatus::STATUS_FIX;
      break;
    case gz_sensor_msgs::NavSatFix::STATUS_SBAS_FIX:
      ros_nav_sat_fix_msg_.status.status =
          sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
      break;
    case gz_sensor_msgs::NavSatFix::STATUS_GBAS_FIX:
      ros_nav_sat_fix_msg_.status.status =
          sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
      break;
    default:
      gzthrow(
          "Specific value of enum type gz_sensor_msgs::NavSatFix::Status is "
          "not yet supported.");
  }

  ros_nav_sat_fix_msg_.latitude = gz_nav_sat_fix_msg->latitude();
  ros_nav_sat_fix_msg_.longitude = gz_nav_sat_fix_msg->longitude();
  ros_nav_sat_fix_msg_.altitude = gz_nav_sat_fix_msg->altitude();

  switch (gz_nav_sat_fix_msg->position_covariance_type()) {
    case gz_sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN:
      ros_nav_sat_fix_msg_.position_covariance_type =
          sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
      break;
    case gz_sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED:
      ros_nav_sat_fix_msg_.position_covariance_type =
          sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
      break;
    case gz_sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN:
      ros_nav_sat_fix_msg_.position_covariance_type =
          sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      break;
    case gz_sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN:
      ros_nav_sat_fix_msg_.position_covariance_type =
          sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;
      break;
    default:
      gzthrow(
          "Specific value of enum type "
          "gz_sensor_msgs::NavSatFix::PositionCovarianceType is not yet "
          "supported.");
  }

  // Position covariance should have 9 elements, and both the Gazebo and ROS
  // arrays should be the same size!
  GZ_ASSERT(gz_nav_sat_fix_msg->position_covariance_size() == 9,
            "The Gazebo NavSatFix message does not have 9 position covariance "
            "elements.");
  GZ_ASSERT(ros_nav_sat_fix_msg_.position_covariance.size() == 9,
            "The ROS NavSatFix message does not have 9 position covariance "
            "elements.");
  for (int i = 0; i < gz_nav_sat_fix_msg->position_covariance_size(); i++) {
    ros_nav_sat_fix_msg_.position_covariance[i] =
        gz_nav_sat_fix_msg->position_covariance(i);
  }

  // Publish to ROS.
  ros_publisher.publish(ros_nav_sat_fix_msg_);
}

void GazeboRosInterfacePlugin::GzOdometryMsgCallback(
    GzOdometryMsgPtr& gz_odometry_msg, ros::Publisher ros_publisher) {
  // We need to convert from a Gazebo message to a ROS message, and then forward
  // the Odometry message to ROS.

  // ============================================ //
  // =================== HEADER ================= //
  // ============================================ //
  ConvertHeaderGzToRos(gz_odometry_msg->header(), &ros_odometry_msg_.header);

  ros_odometry_msg_.child_frame_id = gz_odometry_msg->child_frame_id();

  // ============================================ //
  // ===================== POSE ================= //
  // ============================================ //
  ros_odometry_msg_.pose.pose.position.x =
      gz_odometry_msg->pose().pose().position().x();
  ros_odometry_msg_.pose.pose.position.y =
      gz_odometry_msg->pose().pose().position().y();
  ros_odometry_msg_.pose.pose.position.z =
      gz_odometry_msg->pose().pose().position().z();

  ros_odometry_msg_.pose.pose.orientation.w =
      gz_odometry_msg->pose().pose().orientation().w();
  ros_odometry_msg_.pose.pose.orientation.x =
      gz_odometry_msg->pose().pose().orientation().x();
  ros_odometry_msg_.pose.pose.orientation.y =
      gz_odometry_msg->pose().pose().orientation().y();
  ros_odometry_msg_.pose.pose.orientation.z =
      gz_odometry_msg->pose().pose().orientation().z();

  for (int i = 0; i < gz_odometry_msg->pose().covariance_size(); i++) {
    ros_odometry_msg_.pose.covariance[i] =
        gz_odometry_msg->pose().covariance(i);
  }

  // ============================================ //
  // ===================== TWIST ================ //
  // ============================================ //
  ros_odometry_msg_.twist.twist.linear.x =
      gz_odometry_msg->twist().twist().linear().x();
  ros_odometry_msg_.twist.twist.linear.y =
      gz_odometry_msg->twist().twist().linear().y();
  ros_odometry_msg_.twist.twist.linear.z =
      gz_odometry_msg->twist().twist().linear().z();

  ros_odometry_msg_.twist.twist.angular.x =
      gz_odometry_msg->twist().twist().angular().x();
  ros_odometry_msg_.twist.twist.angular.y =
      gz_odometry_msg->twist().twist().angular().y();
  ros_odometry_msg_.twist.twist.angular.z =
      gz_odometry_msg->twist().twist().angular().z();

  for (int i = 0; i < gz_odometry_msg->twist().covariance_size(); i++) {
    ros_odometry_msg_.twist.covariance[i] =
        gz_odometry_msg->twist().covariance(i);
  }

  // Publish to ROS framework.
  ros_publisher.publish(ros_odometry_msg_);
}

void GazeboRosInterfacePlugin::GzPoseMsgCallback(GzPoseMsgPtr& gz_pose_msg,
                                                 ros::Publisher ros_publisher) {
  ros_pose_msg_.position.x = gz_pose_msg->position().x();
  ros_pose_msg_.position.y = gz_pose_msg->position().y();
  ros_pose_msg_.position.z = gz_pose_msg->position().z();

  ros_pose_msg_.orientation.w = gz_pose_msg->orientation().w();
  ros_pose_msg_.orientation.x = gz_pose_msg->orientation().x();
  ros_pose_msg_.orientation.y = gz_pose_msg->orientation().y();
  ros_pose_msg_.orientation.z = gz_pose_msg->orientation().z();

  ros_publisher.publish(ros_pose_msg_);
}

void GazeboRosInterfacePlugin::GzPoseWithCovarianceStampedMsgCallback(
    GzPoseWithCovarianceStampedMsgPtr& gz_pose_with_covariance_stamped_msg,
    ros::Publisher ros_publisher) {
  // ============================================ //
  // =================== HEADER ================= //
  // ============================================ //
  ConvertHeaderGzToRos(gz_pose_with_covariance_stamped_msg->header(),
                       &ros_pose_with_covariance_stamped_msg_.header);

  // ============================================ //
  // === POSE (both position and orientation) === //
  // ============================================ //
  ros_pose_with_covariance_stamped_msg_.pose.pose.position.x =
      gz_pose_with_covariance_stamped_msg->pose_with_covariance()
          .pose()
          .position()
          .x();
  ros_pose_with_covariance_stamped_msg_.pose.pose.position.y =
      gz_pose_with_covariance_stamped_msg->pose_with_covariance()
          .pose()
          .position()
          .y();
  ros_pose_with_covariance_stamped_msg_.pose.pose.position.z =
      gz_pose_with_covariance_stamped_msg->pose_with_covariance()
          .pose()
          .position()
          .z();

  ros_pose_with_covariance_stamped_msg_.pose.pose.orientation.w =
      gz_pose_with_covariance_stamped_msg->pose_with_covariance()
          .pose()
          .orientation()
          .w();
  ros_pose_with_covariance_stamped_msg_.pose.pose.orientation.x =
      gz_pose_with_covariance_stamped_msg->pose_with_covariance()
          .pose()
          .orientation()
          .x();
  ros_pose_with_covariance_stamped_msg_.pose.pose.orientation.y =
      gz_pose_with_covariance_stamped_msg->pose_with_covariance()
          .pose()
          .orientation()
          .y();
  ros_pose_with_covariance_stamped_msg_.pose.pose.orientation.z =
      gz_pose_with_covariance_stamped_msg->pose_with_covariance()
          .pose()
          .orientation()
          .z();

  // Covariance should have 36 elements, and both the Gazebo and ROS
  // arrays should be the same size!
  GZ_ASSERT(gz_pose_with_covariance_stamped_msg->pose_with_covariance()
                    .covariance_size() == 36,
            "The Gazebo PoseWithCovarianceStamped message does not have 9 "
            "position covariance elements.");
  GZ_ASSERT(ros_pose_with_covariance_stamped_msg_.pose.covariance.size() == 36,
            "The ROS PoseWithCovarianceStamped message does not have 9 "
            "position covariance elements.");
  for (int i = 0;
       i < gz_pose_with_covariance_stamped_msg->pose_with_covariance()
               .covariance_size();
       i++) {
    ros_pose_with_covariance_stamped_msg_.pose.covariance[i] =
        gz_pose_with_covariance_stamped_msg->pose_with_covariance().covariance(
            i);
  }

  ros_publisher.publish(ros_pose_with_covariance_stamped_msg_);
}

void GazeboRosInterfacePlugin::GzTransformStampedMsgCallback(
    GzTransformStampedMsgPtr& gz_transform_stamped_msg,
    ros::Publisher ros_publisher) {
  // ============================================ //
  // =================== HEADER ================= //
  // ============================================ //
  ConvertHeaderGzToRos(gz_transform_stamped_msg->header(),
                       &ros_transform_stamped_msg_.header);

  // ============================================ //
  // =========== TRANSFORM, TRANSLATION ========= //
  // ============================================ //
  ros_transform_stamped_msg_.transform.translation.x =
      gz_transform_stamped_msg->transform().translation().x();
  ros_transform_stamped_msg_.transform.translation.y =
      gz_transform_stamped_msg->transform().translation().y();
  ros_transform_stamped_msg_.transform.translation.z =
      gz_transform_stamped_msg->transform().translation().z();

  // ============================================ //
  // ============ TRANSFORM, ROTATION =========== //
  // ============================================ //
  ros_transform_stamped_msg_.transform.rotation.w =
      gz_transform_stamped_msg->transform().rotation().w();
  ros_transform_stamped_msg_.transform.rotation.x =
      gz_transform_stamped_msg->transform().rotation().x();
  ros_transform_stamped_msg_.transform.rotation.y =
      gz_transform_stamped_msg->transform().rotation().y();
  ros_transform_stamped_msg_.transform.rotation.z =
      gz_transform_stamped_msg->transform().rotation().z();

  ros_publisher.publish(ros_transform_stamped_msg_);
}

void GazeboRosInterfacePlugin::GzTwistStampedMsgCallback(
    GzTwistStampedMsgPtr& gz_twist_stamped_msg, ros::Publisher ros_publisher) {
  // ============================================ //
  // =================== HEADER ================= //
  // ============================================ //
  ConvertHeaderGzToRos(gz_twist_stamped_msg->header(),
                       &ros_twist_stamped_msg_.header);

  // ============================================ //
  // =================== TWIST ================== //
  // ============================================ //

  ros_twist_stamped_msg_.twist.linear.x =
      gz_twist_stamped_msg->twist().linear().x();
  ros_twist_stamped_msg_.twist.linear.y =
      gz_twist_stamped_msg->twist().linear().y();
  ros_twist_stamped_msg_.twist.linear.z =
      gz_twist_stamped_msg->twist().linear().z();

  ros_twist_stamped_msg_.twist.angular.x =
      gz_twist_stamped_msg->twist().angular().x();
  ros_twist_stamped_msg_.twist.angular.y =
      gz_twist_stamped_msg->twist().angular().y();
  ros_twist_stamped_msg_.twist.angular.z =
      gz_twist_stamped_msg->twist().angular().z();

  ros_publisher.publish(ros_twist_stamped_msg_);
}

void GazeboRosInterfacePlugin::GzVector3dStampedMsgCallback(
    GzVector3dStampedMsgPtr& gz_vector_3d_stamped_msg,
    ros::Publisher ros_publisher) {
  // ============================================ //
  // =================== HEADER ================= //
  // ============================================ //
  ConvertHeaderGzToRos(gz_vector_3d_stamped_msg->header(),
                       &ros_position_stamped_msg_.header);

  // ============================================ //
  // ================== POSITION ================ //
  // ============================================ //

  ros_position_stamped_msg_.point.x = gz_vector_3d_stamped_msg->position().x();
  ros_position_stamped_msg_.point.y = gz_vector_3d_stamped_msg->position().y();
  ros_position_stamped_msg_.point.z = gz_vector_3d_stamped_msg->position().z();

  ros_publisher.publish(ros_position_stamped_msg_);
}

void GazeboRosInterfacePlugin::GzWindSpeedMsgCallback(
    GzWindSpeedMsgPtr& gz_wind_speed_msg,
    ros::Publisher ros_publisher) {
  // ============================================ //
  // =================== HEADER ================= //
  // ============================================ //
  ConvertHeaderGzToRos(gz_wind_speed_msg->header(),
                       &ros_wind_speed_msg_.header);

  // ============================================ //
  // ================== VELOCITY ================ //
  // ============================================ //
  ros_wind_speed_msg_.velocity.x =
      gz_wind_speed_msg->velocity().x();
  ros_wind_speed_msg_.velocity.y =
      gz_wind_speed_msg->velocity().y();
  ros_wind_speed_msg_.velocity.z =
      gz_wind_speed_msg->velocity().z();
  ros_publisher.publish(ros_wind_speed_msg_);
}

void GazeboRosInterfacePlugin::GzWrenchStampedMsgCallback(
    GzWrenchStampedMsgPtr& gz_wrench_stamped_msg,
    ros::Publisher ros_publisher) {
  // ============================================ //
  // =================== HEADER ================= //
  // ============================================ //
  ConvertHeaderGzToRos(gz_wrench_stamped_msg->header(),
                       &ros_wrench_stamped_msg_.header);

  // ============================================ //
  // =================== FORCE ================== //
  // ============================================ //
  ros_wrench_stamped_msg_.wrench.force.x =
      gz_wrench_stamped_msg->wrench().force().x();
  ros_wrench_stamped_msg_.wrench.force.y =
      gz_wrench_stamped_msg->wrench().force().y();
  ros_wrench_stamped_msg_.wrench.force.z =
      gz_wrench_stamped_msg->wrench().force().z();

  // ============================================ //
  // ==================== TORQUE ================ //
  // ============================================ //
  ros_wrench_stamped_msg_.wrench.torque.x =
      gz_wrench_stamped_msg->wrench().torque().x();
  ros_wrench_stamped_msg_.wrench.torque.y =
      gz_wrench_stamped_msg->wrench().torque().y();
  ros_wrench_stamped_msg_.wrench.torque.z =
      gz_wrench_stamped_msg->wrench().torque().z();

  ros_publisher.publish(ros_wrench_stamped_msg_);
}

//===========================================================================//
//================ ROS -> GAZEBO MSG CALLBACKS/CONVERTERS ===================//
//===========================================================================//

void GazeboRosInterfacePlugin::RosActuatorsMsgCallback(
    const mav_msgs::ActuatorsConstPtr& ros_actuators_msg_ptr,
    gazebo::transport::PublisherPtr gz_publisher_ptr) {
  // Convert ROS message to Gazebo message

  gz_sensor_msgs::Actuators gz_actuators_msg;

  ConvertHeaderRosToGz(ros_actuators_msg_ptr->header,
                       gz_actuators_msg.mutable_header());

  for (int i = 0; i < ros_actuators_msg_ptr->angles.size(); i++) {
    gz_actuators_msg.add_angles(
        ros_actuators_msg_ptr->angles[i]);
  }

  for (int i = 0; i < ros_actuators_msg_ptr->angular_velocities.size(); i++) {
    gz_actuators_msg.add_angular_velocities(
        ros_actuators_msg_ptr->angular_velocities[i]);
  }

  for (int i = 0; i < ros_actuators_msg_ptr->normalized.size(); i++) {
    gz_actuators_msg.add_normalized(
        ros_actuators_msg_ptr->normalized[i]);
  }

  // Publish to Gazebo
  gz_publisher_ptr->Publish(gz_actuators_msg);
}

void GazeboRosInterfacePlugin::RosCommandMotorSpeedMsgCallback(
    const mav_msgs::ActuatorsConstPtr& ros_actuators_msg_ptr,
    gazebo::transport::PublisherPtr gz_publisher_ptr) {
  // Convert ROS message to Gazebo message

  gz_mav_msgs::CommandMotorSpeed gz_command_motor_speed_msg;

  for (int i = 0; i < ros_actuators_msg_ptr->angular_velocities.size(); i++) {
    gz_command_motor_speed_msg.add_motor_speed(
        ros_actuators_msg_ptr->angular_velocities[i]);
  }

  // Publish to Gazebo
  gz_publisher_ptr->Publish(gz_command_motor_speed_msg);
}

void GazeboRosInterfacePlugin::RosRollPitchYawrateThrustMsgCallback(
    const mav_msgs::RollPitchYawrateThrustConstPtr&
        ros_roll_pitch_yawrate_thrust_msg_ptr,
    gazebo::transport::PublisherPtr gz_publisher_ptr) {
  // Convert ROS message to Gazebo message

  gz_mav_msgs::RollPitchYawrateThrust gz_roll_pitch_yawrate_thrust_msg;

  ConvertHeaderRosToGz(ros_roll_pitch_yawrate_thrust_msg_ptr->header,
                       gz_roll_pitch_yawrate_thrust_msg.mutable_header());

  gz_roll_pitch_yawrate_thrust_msg.set_roll(
      ros_roll_pitch_yawrate_thrust_msg_ptr->roll);
  gz_roll_pitch_yawrate_thrust_msg.set_pitch(
      ros_roll_pitch_yawrate_thrust_msg_ptr->pitch);
  gz_roll_pitch_yawrate_thrust_msg.set_yaw_rate(
      ros_roll_pitch_yawrate_thrust_msg_ptr->yaw_rate);

  gz_roll_pitch_yawrate_thrust_msg.mutable_thrust()->set_x(
      ros_roll_pitch_yawrate_thrust_msg_ptr->thrust.x);
  gz_roll_pitch_yawrate_thrust_msg.mutable_thrust()->set_y(
      ros_roll_pitch_yawrate_thrust_msg_ptr->thrust.y);
  gz_roll_pitch_yawrate_thrust_msg.mutable_thrust()->set_z(
      ros_roll_pitch_yawrate_thrust_msg_ptr->thrust.z);

  // Publish to Gazebo
  gz_publisher_ptr->Publish(gz_roll_pitch_yawrate_thrust_msg);
}

void GazeboRosInterfacePlugin::RosWindSpeedMsgCallback(
    const rotors_comm::WindSpeedConstPtr& ros_wind_speed_msg_ptr,
    gazebo::transport::PublisherPtr gz_publisher_ptr) {
  // Convert ROS message to Gazebo message

  gz_mav_msgs::WindSpeed gz_wind_speed_msg;

  ConvertHeaderRosToGz(ros_wind_speed_msg_ptr->header,
                       gz_wind_speed_msg.mutable_header());

  gz_wind_speed_msg.mutable_velocity()->set_x(
      ros_wind_speed_msg_ptr->velocity.x);
  gz_wind_speed_msg.mutable_velocity()->set_y(
      ros_wind_speed_msg_ptr->velocity.y);
  gz_wind_speed_msg.mutable_velocity()->set_z(
      ros_wind_speed_msg_ptr->velocity.z);

  // Publish to Gazebo
  gz_publisher_ptr->Publish(gz_wind_speed_msg);
}

void GazeboRosInterfacePlugin::GzBroadcastTransformMsgCallback(
    GzTransformStampedWithFrameIdsMsgPtr& broadcast_transform_msg) {
  ros::Time stamp;
  stamp.sec = broadcast_transform_msg->header().stamp().sec();
  stamp.nsec = broadcast_transform_msg->header().stamp().nsec();

  tf::Quaternion tf_q_W_L(broadcast_transform_msg->transform().rotation().x(),
                          broadcast_transform_msg->transform().rotation().y(),
                          broadcast_transform_msg->transform().rotation().z(),
                          broadcast_transform_msg->transform().rotation().w());

  tf::Vector3 tf_p(broadcast_transform_msg->transform().translation().x(),
                   broadcast_transform_msg->transform().translation().y(),
                   broadcast_transform_msg->transform().translation().z());

  tf_ = tf::Transform(tf_q_W_L, tf_p);
  transform_broadcaster_.sendTransform(tf::StampedTransform(
      tf_, stamp, broadcast_transform_msg->parent_frame_id(),
      broadcast_transform_msg->child_frame_id()));
}

GZ_REGISTER_WORLD_PLUGIN(GazeboRosInterfacePlugin);

}  // namespace gazebo

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

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>
#include <rotors_gazebo_plugins/gazebo_ros_interface_plugin.h>

namespace gazebo {


GazeboRosInterfacePlugin* GazeboRosInterfacePlugin::instance;

GazeboRosInterfacePlugin::GazeboRosInterfacePlugin()
    : ModelPlugin(),
      gz_node_handle_(0),
      ros_node_handle_(0),
      ros_actuators_msg_(new mav_msgs::Actuators)
{
  GZ_ASSERT(!instance, "");
  instance = this;

  // Nothing
}

GazeboRosInterfacePlugin::~GazeboRosInterfacePlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  instance = nullptr;
}


void GazeboRosInterfacePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

  gzmsg << __PRETTY_FUNCTION__ << " called." << std::endl;

  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  // default params
  namespace_.clear();

  //==============================================//
  //========== READ IN PARAMS FROM SDF ===========//
  //==============================================//

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_imu_plugin] Please specify a robotNamespace.\n";
  gzmsg << "Namespace is \"" << namespace_ << "\"." << std::endl;

  // Get Gazebo node handle
  gz_node_handle_ = transport::NodePtr(new transport::Node());
  gz_node_handle_->Init(namespace_);

  // Get ROS node handle
  ros_node_handle_ = new ros::NodeHandle(namespace_);

//  if (_sdf->HasElement("linkName"))
//    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
//  else
//    gzerr << "[gazebo_imu_plugin] Please specify a linkName.\n";
//  // Get the pointer to the link
//  link_ = model_->GetLink(link_name_);
//  if (link_ == NULL)
//    gzthrow("[gazebo_imu_plugin] Couldn't find specified link \"" << link_name_ << "\".");


  last_time_ = world_->GetSimTime();

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRosInterfacePlugin::OnUpdate, this, _1));

  // ============================================ //
  // ============ ACTUATORS MSG SETUP =========== //
  // ============================================ //

//  std::string actuators_sub_topic;
//  if(_sdf->HasElement("actuatorsSubTopic"))
//    getSdfParam<std::string>(_sdf, "actuatorsSubTopic", actuators_sub_topic, "");
//  else
//    gzerr << "Please specify actuatorsSubTopic." << std::endl;
//
//  std::string gz_actuators_subtopic_name = "~/" + actuators_sub_topic;
//  gz_imu_sub_ = gz_node_handle_->Subscribe(gz_actuators_subtopic_name, &GazeboRosInterfacePlugin::GzActuatorsMsgCallback, this);
//  gzmsg << "GazeboMsgInterfacePlugin subscribing to Gazebo topic \"" << gz_actuators_subtopic_name << "\"." << std::endl;
//
//  ros_imu_pub_ = ros_node_handle_->advertise<mav_msgs::Actuators>(actuators_sub_topic, 1);
//  gzmsg << "GazeboMsgInterfacePlugin publishing to ROS topic \"" << actuators_sub_topic << "\"." << std::endl;


  // ============================================ //
  // =============== IMU MSG SETUP ============== //
  // ============================================ //

//  std::string imu_sub_topic;
//  if(_sdf->HasElement("imuSubTopic"))
//    getSdfParam<std::string>(_sdf, "imuSubTopic", imu_sub_topic,
//                                 mav_msgs::default_topics::IMU);
//  else
//    gzerr << "Please specify imuSubTopic." << std::endl;
//
//  std::string gz_imu_subtopic_name = "~/" + imu_sub_topic;
//  gz_imu_sub_ = gz_node_handle_->Subscribe(gz_imu_subtopic_name, &GazeboRosInterfacePlugin::GzImuCallback, this);
//  gzmsg << "GazeboMsgInterfacePlugin subscribing to Gazebo topic \"" << gz_imu_subtopic_name << "\"." << std::endl;
//
//  ros_imu_pub_ = ros_node_handle_->advertise<sensor_msgs::Imu>(imu_sub_topic, 1);
//  gzmsg << "GazeboMsgInterfacePlugin publishing to ROS topic \"" << imu_sub_topic << "\"." << std::endl;

  // ============================================ //
  // ========== JOINT STATE MSG SETUP =========== //
  // ============================================ //

//  std::string joint_state_sub_topic;
//  if(_sdf->HasElement("jointStateSubTopic"))
//    getSdfParam<std::string>(_sdf, "jointStateSubTopic", joint_state_sub_topic, "");
//  else
//    gzerr << "Please specify jointStateSubTopic." << std::endl;
//
//  std::string gz_joint_state_subtopic_name = "~/" + joint_state_sub_topic;
//  gz_imu_sub_ = gz_node_handle_->Subscribe(gz_joint_state_subtopic_name, &GazeboRosInterfacePlugin::GzJointStateMsgCallback, this);
//  gzmsg << "GazeboMsgInterfacePlugin subscribing to Gazebo topic \"" << gz_joint_state_subtopic_name << "\"." << std::endl;
//
//  ros_imu_pub_ = ros_node_handle_->advertise<sensor_msgs::JointState>(joint_state_sub_topic, 1);
//  gzmsg << "GazeboMsgInterfacePlugin publishing to ROS topic \"" << joint_state_sub_topic << "\"." << std::endl;

  // ============================================ //
  // ========= MAGNETIC FIELD MSG SETUP ========= //
  // ============================================ //

//  std::string magnetic_field_sub_topic;
//  if(_sdf->HasElement("magneticFieldSubTopic"))
//    getSdfParam<std::string>(_sdf, "magneticFieldSubTopic", magnetic_field_sub_topic, "");
//  else
//    gzerr << "Please specify an magneticFieldSubTopic." << std::endl;
//
//  std::string gz_magnetic_field_sub_topic_name = "~/" + magnetic_field_sub_topic;
//  gz_magnetic_field_sub_ = gz_node_handle_->Subscribe(gz_magnetic_field_sub_topic_name, &GazeboRosInterfacePlugin::GzMagneticFieldMsgCallback, this);
//  gzmsg << "GazeboMsgInterfacePlugin subscribing to Gazebo topic \"" << gz_magnetic_field_sub_topic_name << "\"." << std::endl;
//
//  ros_magnetic_field_pub_ = ros_node_handle_->advertise<sensor_msgs::MagneticField>(magnetic_field_sub_topic, 1);
//  gzmsg << "GazeboMsgInterfacePlugin publishing to ROS topic \"" << magnetic_field_sub_topic << "\"." << std::endl;

  // ============================================ //
  // =========== NAV SAT FIX MSG SETUP ========== //
  // ============================================ //

//  std::string nav_sat_fix_subtopic_name;
//  if(_sdf->HasElement("navSatFixSubTopic"))
//    getSdfParam<std::string>(_sdf, "navSatFixSubTopic", nav_sat_fix_subtopic_name, "");
//  else
//    gzerr << "Please specify an navSatFixSubTopic." << std::endl;
//
//  std::string gz_nav_sat_fix_subtopic_name = "~/" + nav_sat_fix_subtopic_name;
//  gz_nav_sat_fix_sub_ = gz_node_handle_->Subscribe(gz_nav_sat_fix_subtopic_name, &GazeboRosInterfacePlugin::GzNavSatFixCallback, this);
//  gzmsg << "GazeboMsgInterfacePlugin subscribing to Gazebo topic \"" << gz_nav_sat_fix_subtopic_name << "\"." << std::endl;
//
//  ros_nav_sat_fix_pub_ = ros_node_handle_->advertise<sensor_msgs::NavSatFix>(nav_sat_fix_subtopic_name, 1);
//  gzmsg << "GazeboMsgInterfacePlugin publishing to ROS topic \"" << nav_sat_fix_subtopic_name << "\"." << std::endl;

  // ============================================ //
  // ============= ODOMETRY MSG SETUP =========== //
  // ============================================ //

//  std::string odometry_subtopic_name;
//  if(_sdf->HasElement("odometrySubTopic"))
//    getSdfParam<std::string>(_sdf, "odometrySubTopic", odometry_subtopic_name, "");
//  else
//    gzerr << "Please specify an odometrySubTopic." << std::endl;
//
//  std::string gz_odometry_subtopic_name = "~/" + odometry_subtopic_name;
//  gz_odometry_sub_ = gz_node_handle_->Subscribe(gz_odometry_subtopic_name, &GazeboRosInterfacePlugin::GzOdometryMsgCallback, this);
//  gzmsg << "GazeboMsgInterfacePlugin subscribing to Gazebo topic \"" << gz_odometry_subtopic_name << "\"." << std::endl;
//
//  ros_odometry_pub_ = ros_node_handle_->advertise<nav_msgs::Odometry>(odometry_subtopic_name, 1);
//  gzmsg << "GazeboMsgInterfacePlugin publishing to ROS topic \"" << odometry_subtopic_name << "\"." << std::endl;

}

template <typename M>
struct ConnectHelperStorage {
  GazeboRosInterfacePlugin * ptr;
  void(GazeboRosInterfacePlugin::*fp)(const boost::shared_ptr<M const> &, ros::Publisher ros_publisher);
//  void(GazeboRosInterfacePlugin::*fp)(const boost::shared_ptr<M const> &, Publisher);

  ros::Publisher ros_publisher;

  /// @brief    This is what gets passed into the Gazebo Subscribe method as a callback, and hence can only
  ///           have one parameter (note boost::bind() does not work with the current Gazebo Subscribe() definitions).
  void callback (const boost::shared_ptr<M const> & msg_ptr) {
    //gzmsg << "callback() called." << std::endl;
    (ptr->*fp)(msg_ptr, ros_publisher);
//    (ptr->*fp)(msg_ptr, publisher);
  }

};

// M is the type of the message that will be subscribed to the Gazebo framework.
// N is the type of the message published to the ROS framework
template <typename M, typename N>
void GazeboRosInterfacePlugin::ConnectHelper(
    void(GazeboRosInterfacePlugin::*fp)(const boost::shared_ptr<M const> &, ros::Publisher),
    GazeboRosInterfacePlugin * ptr,
    std::string gazeboTopicName,
    std::string rosTopicName,
    transport::NodePtr gz_node_handle) {

  // One map will be created for each type M
  static std::map< std::string, ConnectHelperStorage<M> > callback_map;

  // Create ROS publisher
  gzmsg << "GazeboMsgInterfacePlugin publishing to ROS topic \"" << rosTopicName << "\"." << std::endl;
  ros::Publisher ros_publisher = ros_node_handle_->advertise<N>(rosTopicName, 1);

  // @todo Handle collision error
  auto callback_entry = callback_map.emplace(gazeboTopicName, ConnectHelperStorage<M>{ptr, fp, ros_publisher});

  gzmsg << "GazeboRosMsgInterfacePlugin subscribing to Gazebo topic \"" << gazeboTopicName << "\"."<< std::endl;
  gazebo::transport::SubscriberPtr subscriberPtr;
  subscriberPtr = gz_node_handle->Subscribe(
      gazeboTopicName,
      &ConnectHelperStorage<M>::callback,
      &callback_entry.first->second);

  // Save a reference to the subscriber pointer so subsriber
  // won't be deleted.
  subscriberPtrs_.push_back(subscriberPtr);

}

void GazeboRosInterfacePlugin::ConnectToRos(std::string gazeboTopicName, std::string rosTopicName, SupportedMsgTypes msgType) {

//  gz_odometry_sub_ = gz_node_handle_->Subscribe<gz_geometry_msgs::Odometry>(gazeboTopicName, &GazeboRosInterfacePlugin::GzOdometryMsgCallback, this);

  gazebo::transport::SubscriberPtr subscriberPtr;
  switch(msgType) {
    case SupportedMsgTypes::ACTUATORS:
      ConnectHelper<sensor_msgs::msgs::Actuators, mav_msgs::Actuators>(&GazeboRosInterfacePlugin::GzActuatorsMsgCallback, this, gazeboTopicName, rosTopicName, gz_node_handle_);
      break;
    case SupportedMsgTypes::IMU:
      ConnectHelper<sensor_msgs::msgs::Imu, sensor_msgs::Imu>(&GazeboRosInterfacePlugin::GzImuMsgCallback, this, gazeboTopicName, rosTopicName, gz_node_handle_);
      break;
    case SupportedMsgTypes::JOINT_STATE:
      ConnectHelper<sensor_msgs::msgs::JointState, sensor_msgs::JointState>(&GazeboRosInterfacePlugin::GzJointStateMsgCallback, this, gazeboTopicName, rosTopicName, gz_node_handle_);
      break;
    case SupportedMsgTypes::MAGNETIC_FIELD:
      ConnectHelper<sensor_msgs::msgs::MagneticField, sensor_msgs::MagneticField>(&GazeboRosInterfacePlugin::GzMagneticFieldMsgCallback, this, gazeboTopicName, rosTopicName, gz_node_handle_);
      break;
    case SupportedMsgTypes::NAV_SAT_FIX:
      ConnectHelper<sensor_msgs::msgs::NavSatFix, sensor_msgs::NavSatFix>(&GazeboRosInterfacePlugin::GzNavSatFixCallback, this, gazeboTopicName, rosTopicName, gz_node_handle_);
      break;
    case SupportedMsgTypes::ODOMETRY:
      ConnectHelper<gz_geometry_msgs::Odometry, nav_msgs::Odometry>(&GazeboRosInterfacePlugin::GzOdometryMsgCallback, this, gazeboTopicName, rosTopicName, gz_node_handle_);
      break;
    case SupportedMsgTypes::TWIST_STAMPED:
      ConnectHelper<sensor_msgs::msgs::TwistStamped, geometry_msgs::TwistStamped>(&GazeboRosInterfacePlugin::GzTwistStampedMsgCallback, this, gazeboTopicName, rosTopicName, gz_node_handle_);
      break;
    default:
      gzthrow("Message type is not supported by GazeboRosInterfacePlugin.");
  }

}

void GazeboRosInterfacePlugin::GzActuatorsMsgCallback(GzActuatorsMsgPtr& gz_actuators_msg, ros::Publisher ros_publisher) {
  gzmsg << __PRETTY_FUNCTION__ << " called." << std::endl;

  // We need to convert the Acutuators message from a Gazebo message to a
  // ROS message and then publish it to the ROS framework

  ros_actuators_msg_->header.stamp.sec = gz_actuators_msg->header().stamp().sec();
  ros_actuators_msg_->header.stamp.nsec = gz_actuators_msg->header().stamp().nsec();
  ros_actuators_msg_->header.frame_id = gz_actuators_msg->header().frame_id();

  ros_actuators_msg_->angular_velocities.resize(gz_actuators_msg->angular_velocities_size());
  for(int i = 0; i < gz_actuators_msg->angular_velocities_size(); i++) {
    ros_actuators_msg_->angular_velocities[i] = gz_actuators_msg->angular_velocities(i);
  }

  // Publish onto ROS framework
  ros_publisher.publish(ros_actuators_msg_);

}

void GazeboRosInterfacePlugin::GzImuMsgCallback(GzImuPtr& gz_imu_msg, ros::Publisher ros_publisher) {
//  gzmsg << __PRETTY_FUNCTION__ << " called." << std::endl;

  // We need to convert from a Gazebo message to a ROS message,
  // and then forward the IMU message onto ROS

  ros_imu_msg_.header.stamp.sec = gz_imu_msg->header().stamp().sec();
  ros_imu_msg_.header.stamp.nsec = gz_imu_msg->header().stamp().nsec();
  ros_imu_msg_.header.frame_id = gz_imu_msg->header().frame_id();

  ros_imu_msg_.orientation.x = gz_imu_msg->orientation().x();
  ros_imu_msg_.orientation.y = gz_imu_msg->orientation().y();
  ros_imu_msg_.orientation.z = gz_imu_msg->orientation().z();
  ros_imu_msg_.orientation.w = gz_imu_msg->orientation().w();

  // Orientation covariance should have 9 elements, and both the Gazebo and ROS
  // arrays should be the same size!
  GZ_ASSERT(gz_imu_msg->orientation_covariance_size() == 9, "The Gazebo IMU message does not have 9 orientation covariance elements.");
  GZ_ASSERT(ros_imu_msg_.orientation_covariance.size() == 9, "The ROS IMU message does not have 9 orientation covariance elements.");
  for(int i = 0; i < gz_imu_msg->orientation_covariance_size(); i ++) {
    ros_imu_msg_.orientation_covariance[i] = gz_imu_msg->orientation_covariance(i);
  }

  ros_imu_msg_.angular_velocity.x = gz_imu_msg->angular_velocity().x();
  ros_imu_msg_.angular_velocity.y = gz_imu_msg->angular_velocity().y();
  ros_imu_msg_.angular_velocity.z = gz_imu_msg->angular_velocity().z();

  GZ_ASSERT(gz_imu_msg->angular_velocity_covariance_size() == 9, "The Gazebo IMU message does not have 9 angular velocity covariance elements.");
  GZ_ASSERT(ros_imu_msg_.angular_velocity_covariance.size() == 9, "The ROS IMU message does not have 9 angular velocity covariance elements.");
  for(int i = 0; i < gz_imu_msg->angular_velocity_covariance_size(); i ++) {
    ros_imu_msg_.angular_velocity_covariance[i] = gz_imu_msg->angular_velocity_covariance(i);
  }

  ros_imu_msg_.linear_acceleration.x = gz_imu_msg->linear_acceleration().x();
  ros_imu_msg_.linear_acceleration.y = gz_imu_msg->linear_acceleration().y();
  ros_imu_msg_.linear_acceleration.z = gz_imu_msg->linear_acceleration().z();

  GZ_ASSERT(gz_imu_msg->linear_acceleration_covariance_size() == 9, "The Gazebo IMU message does not have 9 linear acceleration covariance elements.");
  GZ_ASSERT(ros_imu_msg_.linear_acceleration_covariance.size() == 9, "The ROS IMU message does not have 9 linear acceleration covariance elements.");
  for(int i = 0; i < gz_imu_msg->linear_acceleration_covariance_size(); i ++) {
    ros_imu_msg_.linear_acceleration_covariance[i] = gz_imu_msg->linear_acceleration_covariance(i);
  }

  // Publish onto ROS framework
  ros_publisher.publish(ros_imu_msg_);

}

void GazeboRosInterfacePlugin::GzJointStateMsgCallback(GzJointStateMsgPtr& gz_joint_state_msg, ros::Publisher ros_publisher) {
  gzmsg << __PRETTY_FUNCTION__ << " called." << std::endl;
}

void GazeboRosInterfacePlugin::GzMagneticFieldMsgCallback(GzMagneticFieldMsgPtr& gz_magnetic_field_msg, ros::Publisher ros_publisher) {
  gzmsg << __PRETTY_FUNCTION__ << " called." << std::endl;

  // We need to convert from a Gazebo message to a ROS message,
  // and then forward the MagneticField message onto ROS
  ros_magnetic_field_msg_.header.stamp.sec = gz_magnetic_field_msg->header().stamp().sec();
  ros_magnetic_field_msg_.header.stamp.nsec = gz_magnetic_field_msg->header().stamp().nsec();
  ros_magnetic_field_msg_.header.frame_id = gz_magnetic_field_msg->header().frame_id();

  ros_magnetic_field_msg_.magnetic_field.x = gz_magnetic_field_msg->magnetic_field().x();
  ros_magnetic_field_msg_.magnetic_field.y = gz_magnetic_field_msg->magnetic_field().y();
  ros_magnetic_field_msg_.magnetic_field.z = gz_magnetic_field_msg->magnetic_field().z();

  // Position covariance should have 9 elements, and both the Gazebo and ROS
  // arrays should be the same size!
  GZ_ASSERT(gz_magnetic_field_msg->magnetic_field_covariance_size() == 9, "The Gazebo MagneticField message does not have 9 magnetic field covariance elements.");
  GZ_ASSERT(ros_magnetic_field_msg_.magnetic_field_covariance.size() == 9, "The ROS MagneticField message does not have 9 magnetic field covariance elements.");
  for(int i = 0; i < gz_magnetic_field_msg->magnetic_field_covariance_size(); i ++) {
    ros_magnetic_field_msg_.magnetic_field_covariance[i] = gz_magnetic_field_msg->magnetic_field_covariance(i);
  }

  // Publish onto ROS framework
  ros_publisher.publish(ros_magnetic_field_msg_);

}

void GazeboRosInterfacePlugin::GzNavSatFixCallback(GzNavSatFixPtr& gz_nav_sat_fix_msg, ros::Publisher ros_publisher) {
  gzmsg << __PRETTY_FUNCTION__ << " called." << std::endl;

  // We need to convert from a Gazebo message to a ROS message,
  // and then forward the NavSatFix message onto ROS

  ros_nav_sat_fix_msg_.header.stamp.sec = gz_nav_sat_fix_msg->header().stamp().sec();
  ros_nav_sat_fix_msg_.header.stamp.nsec = gz_nav_sat_fix_msg->header().stamp().nsec();
  ros_nav_sat_fix_msg_.header.frame_id = gz_nav_sat_fix_msg->header().frame_id();

  ros_nav_sat_fix_msg_.status.service = gz_nav_sat_fix_msg->status().service();
  ros_nav_sat_fix_msg_.status.status = gz_nav_sat_fix_msg->status().status();

  ros_nav_sat_fix_msg_.latitude = gz_nav_sat_fix_msg->latitude();
  ros_nav_sat_fix_msg_.longitude = gz_nav_sat_fix_msg->longitude();
  ros_nav_sat_fix_msg_.altitude = gz_nav_sat_fix_msg->altitude();

  ros_nav_sat_fix_msg_.position_covariance_type = gz_nav_sat_fix_msg->position_covariance_type();

  // Position covariance should have 9 elements, and both the Gazebo and ROS
  // arrays should be the same size!
  GZ_ASSERT(gz_nav_sat_fix_msg->position_covariance_size() == 9, "The Gazebo NavSatFix message does not have 9 position covariance elements.");
  GZ_ASSERT(ros_nav_sat_fix_msg_.position_covariance.size() == 9, "The ROS NavSatFix message does not have 9 position covariance elements.");
  for(int i = 0; i < gz_nav_sat_fix_msg->position_covariance_size(); i ++) {
    ros_nav_sat_fix_msg_.position_covariance[i] = gz_nav_sat_fix_msg->position_covariance(i);
  }

  // Publish onto ROS framework
  ros_publisher.publish(ros_nav_sat_fix_msg_);

}

void GazeboRosInterfacePlugin::GzOdometryMsgCallback(GzOdometryMsgPtr& gz_odometry_msg, ros::Publisher ros_publisher) {
  //gzmsg << __PRETTY_FUNCTION__ << " called." << std::endl;

  // We need to convert from a Gazebo message to a ROS message,
  // and then forward the Odometry message onto ROS

  ros_odometry_msg_.header.stamp.sec = gz_odometry_msg->header().stamp().sec();
  ros_odometry_msg_.header.stamp.nsec = gz_odometry_msg->header().stamp().nsec();
  ros_odometry_msg_.header.frame_id = gz_odometry_msg->header().frame_id();

  ros_odometry_msg_.child_frame_id = gz_odometry_msg->child_frame_id();

  // ============================================ //
  // ===================== POSE ================= //
  // ============================================ //
  ros_odometry_msg_.pose.pose.position.x = gz_odometry_msg->pose().pose().position().x();
  ros_odometry_msg_.pose.pose.position.y = gz_odometry_msg->pose().pose().position().y();
  ros_odometry_msg_.pose.pose.position.z = gz_odometry_msg->pose().pose().position().z();

  ros_odometry_msg_.pose.pose.orientation.w = gz_odometry_msg->pose().pose().orientation().w();
  ros_odometry_msg_.pose.pose.orientation.x = gz_odometry_msg->pose().pose().orientation().x();
  ros_odometry_msg_.pose.pose.orientation.y = gz_odometry_msg->pose().pose().orientation().y();
  ros_odometry_msg_.pose.pose.orientation.z = gz_odometry_msg->pose().pose().orientation().z();

  for(int i = 0; i < gz_odometry_msg->pose().covariance_size(); i ++) {
    ros_odometry_msg_.pose.covariance[i] = gz_odometry_msg->pose().covariance(i);
  }

  // ============================================ //
  // ===================== TWIST ================ //
  // ============================================ //
  ros_odometry_msg_.twist.twist.linear.x = gz_odometry_msg->twist().twist().linear().x();
  ros_odometry_msg_.twist.twist.linear.y = gz_odometry_msg->twist().twist().linear().y();
  ros_odometry_msg_.twist.twist.linear.z = gz_odometry_msg->twist().twist().linear().z();

  ros_odometry_msg_.twist.twist.angular.x = gz_odometry_msg->twist().twist().angular().x();
  ros_odometry_msg_.twist.twist.angular.y = gz_odometry_msg->twist().twist().angular().y();
  ros_odometry_msg_.twist.twist.angular.z = gz_odometry_msg->twist().twist().angular().z();

  for(int i = 0; i < gz_odometry_msg->twist().covariance_size(); i ++) {
    ros_odometry_msg_.twist.covariance[i] = gz_odometry_msg->twist().covariance(i);
  }

  // Publish onto ROS framework
//  gzmsg << "Publishing Odometry message to ROS framework..." << std::endl;
  ros_publisher.publish(ros_odometry_msg_);

}

void GazeboRosInterfacePlugin::GzTwistStampedMsgCallback(GzTwistStampedMsgPtr& gz_twist_stamped_msg, ros::Publisher publisher) {
  gzmsg << __PRETTY_FUNCTION__ << " called." << std::endl;
}



void GazeboRosInterfacePlugin::OnUpdate(const common::UpdateInfo& _info) {
  common::Time current_time  = world_->GetSimTime();
  double dt = (current_time - last_time_).Double();
  last_time_ = current_time;
  double t = current_time.Double();



}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosInterfacePlugin);

} // namespace gazebo

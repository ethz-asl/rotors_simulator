/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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


#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_BAG_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_BAG_PLUGIN_H

#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <mav_msgs/CommandAttitudeThrust.h>
#include <mav_msgs/CommandMotorSpeed.h>
#include <mav_msgs/CommandRateThrust.h>
#include <mav_msgs/CommandTrajectory.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>

#include "rotors_gazebo_plugins/common.h"


namespace gazebo {
// Default values
static const std::string kDefaultNamespace = "";

static const std::string kDefaultGroundTruthPosePubTopic = "/ground_truth/pose";
static const std::string kDefaultGroundTruthTwistPubTopic = "/ground_truth/twist";
static const std::string kDefaultImuPubTopic = "/imu";
static const std::string kDefaultImuSubTopic = "/imu";
static const std::string kDefaultControlAttitudeThrustPubTopic = "/command/attitude";
static const std::string kDefaultControlAttitudeThrustSubTopic = "/command/attitude";
static const std::string kDefaultControlMotorSpeedPubTopic = "/command/motors";
static const std::string kDefaultControlMotorSpeedSubTopic = "/command/motors";
static const std::string kDefaultControlRateThrustPubTopic = "/command/rate";
static const std::string kDefaultControlRateThrustSubTopic = "/command/rate";
static const std::string kDefaultMotorPubTopic = "/motors";
static const std::string kDefaultCollisionsPubTopic = "/collisions";
static const std::string kDefaultWindPubTopic = "/wind";
static const std::string kDefaultWindSubTopic = "/wind";
static const std::string kDefaultWaypointPubTopic = "/waypoint";
static const std::string kDefaultWaypointSubTopic = "/waypoint";

static const std::string kDefaultFrameId = "ground_truth_pose";
static const std::string kDefaultLinkName = "base_link";
static const std::string kDefaultBagFilename_ = "simulator.bag";

static constexpr double kDefaultRotorVelocitySlowdownSim = 10.0;


/// \brief This plugin is used to create rosbag files from within gazebo.
class GazeboBagPlugin : public ModelPlugin {
  typedef std::map<const unsigned int, const physics::JointPtr> MotorNumberToJointMap;
  typedef std::pair<const unsigned int, const physics::JointPtr> MotorNumberToJointPair;
 public:
  GazeboBagPlugin()
      : ModelPlugin(),
        namespace_(kDefaultNamespace),
        ground_truth_pose_pub_topic_(kDefaultGroundTruthPosePubTopic),
        ground_truth_twist_pub_topic_(kDefaultGroundTruthTwistPubTopic),
        imu_pub_topic_(kDefaultImuPubTopic),
        imu_sub_topic_(kDefaultImuSubTopic),
        control_attitude_thrust_pub_topic_(kDefaultControlAttitudeThrustPubTopic),
        control_attitude_thrust_sub_topic_(kDefaultControlAttitudeThrustSubTopic),
        control_motor_speed_pub_topic_(kDefaultControlMotorSpeedPubTopic),
        control_motor_speed_sub_topic_(kDefaultControlMotorSpeedSubTopic),
        control_rate_thrust_pub_topic_(kDefaultControlRateThrustPubTopic),
        control_rate_thrust_sub_topic_(kDefaultControlRateThrustSubTopic),
        motor_pub_topic_(kDefaultMotorPubTopic),
        collisions_pub_topic_(kDefaultCollisionsPubTopic),
        wind_pub_topic_(kDefaultWindPubTopic),
        wind_sub_topic_(kDefaultWindSubTopic),
        waypoint_pub_topic_(kDefaultWaypointPubTopic),
        waypoint_sub_topic_(kDefaultWaypointSubTopic),
        frame_id_(kDefaultFrameId),
        link_name_(kDefaultLinkName),
        bag_filename_(kDefaultBagFilename_),
        rotor_velocity_slowdown_sim_(kDefaultRotorVelocitySlowdownSim),
        node_handle_(NULL) {}

  virtual ~GazeboBagPlugin();

 protected:
  /// \brief Load the plugin.
  /// \param[in] _model Pointer to the model that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Called when the world is updated.
  /// \param[in] _info Update timing information.
  void OnUpdate(const common::UpdateInfo& /*_info*/);

  /// \brief Called when an IMU message is received.
  /// \param[in] imu_msg A IMU message from sensor_msgs.
  void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg);

  /// \brief Called when an Wind message is received.
  /// \param[in] wind_msg A WrenchStamped message from geometry_msgs.
  void WindCallback(const geometry_msgs::WrenchStampedConstPtr& wind_msg);

  /// \brief Called when an Trajectory message is received.
  /// \param[in] trajectory_msg A CommandTrajectory message from mav_msgs.
  void WaypointCallback(const mav_msgs::CommandTrajectoryConstPtr& trajectory_msg);

  /// \brief Called when a CommandAttitudeThrust message is received.
  /// \param[in] control_msg A CommandAttitudeThrust message from mav_msgs.
  void CommandAttitudeThrustCallback(const mav_msgs::CommandAttitudeThrustConstPtr& control_msg);

  /// \brief Called when a CommandMotorSpeed message is received.
  /// \param[in] control_msg A CommandMotorSpeed message from mav_msgs.
  void CommandMotorSpeedCallback(const mav_msgs::CommandMotorSpeedConstPtr& control_msg);

  /// \brief Called when a CommandRateThrust message is received.
  /// \param[in] control_msg A CommandRateThrust message from mav_msgs.
  void CommandRateThrustCallback(const mav_msgs::CommandRateThrustConstPtr& control_msg);

  /// \brief Log the ground truth pose and twist.
  /// \param[in] now The current gazebo common::Time
  void LogGroundTruth(const common::Time now);

  /// \brief Log all the motor velocities.
  /// \param[in] now The current gazebo common::Time
  void LogMotorVelocities(const common::Time now);

  /// \brief Log all the collisions.
  /// \param[in] now The current gazebo common::Time
  void LogCollisions(const common::Time now);

 private:
  /// \brief Pointer to the update event connection.
  event::ConnectionPtr update_connection_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;

  physics::Link_V child_links_;

  MotorNumberToJointMap motor_joints_;

  // /// \brief Pointer to the ContactManager to get all collisions of this
  // /// link and its children
  physics::ContactManager *contact_mgr_;

  std::string namespace_;
  std::string ground_truth_pose_pub_topic_;
  std::string ground_truth_twist_pub_topic_;
  std::string imu_pub_topic_;
  std::string imu_sub_topic_;
  std::string wind_pub_topic_;
  std::string wind_sub_topic_;
  std::string waypoint_pub_topic_;
  std::string waypoint_sub_topic_;
  std::string control_attitude_thrust_pub_topic_;
  std::string control_attitude_thrust_sub_topic_;
  std::string control_motor_speed_pub_topic_;
  std::string control_motor_speed_sub_topic_;
  std::string control_rate_thrust_pub_topic_;
  std::string control_rate_thrust_sub_topic_;
  std::string collisions_pub_topic_;
  std::string motor_pub_topic_;
  std::string frame_id_;
  std::string link_name_;
  std::string bag_filename_;
  double rotor_velocity_slowdown_sim_;

  /// \brief Mutex lock for thread safty of writing bag files
  boost::mutex mtx_;

  rosbag::Bag bag_;
  ros::NodeHandle *node_handle_;

  // Ros subscribers
  ros::Subscriber imu_sub_;
  ros::Subscriber wind_sub_;
  ros::Subscriber waypoint_sub_;
  ros::Subscriber control_attitude_thrust_sub_;
  ros::Subscriber control_motor_speed_sub_;
  ros::Subscriber control_rate_thrust_sub_;

  std::ofstream csvOut;

  template<class T>
  void writeBag(const std::string& topic, const ros::Time& time, const T& msg) {
    boost::mutex::scoped_lock lock(mtx_);
    try {
      bag_.write(topic, time, msg);
    }
    catch (rosbag::BagIOException& e) {
      gzerr << "Error while writing to bag " << e.what() << std::endl;
    }
    catch (rosbag::BagException& e) {
      if (time < ros::TIME_MIN) {
        gzerr<<"Header stamp not set for msg published on topic: "<< topic << ". " << e.what() << std::endl;
      }
      else {
        gzerr << "Error while writing to bag " << e.what() << std::endl;
      }
    }
  }

  template<class T>
  void writeBag(const std::string& topic, const ros::Time& time, boost::shared_ptr<T const> const& msg) {
    boost::mutex::scoped_lock lock(mtx_);
    try {
      bag_.write(topic, time, msg);
    }
    catch (rosbag::BagIOException& e) {
      gzerr << "Error while writing to bag " << e.what() << std::endl;
    }
    catch (rosbag::BagException& e) {
      if (time < ros::TIME_MIN) {
        gzerr<<"Header stamp not set for msg published on topic: "<< topic << ". " << e.what() << std::endl;
      }
      else {
        gzerr << "Error while writing to bag " << e.what() << std::endl;
      }
    }
  }

};
}

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_BAG_PLUGIN_H

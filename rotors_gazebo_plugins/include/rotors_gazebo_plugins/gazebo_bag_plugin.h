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
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/RateThrust.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "rotors_comm/RecordRosbag.h"
#include "rotors_gazebo_plugins/common.h"


namespace gazebo {
// Default values, the rest from common.h
static const std::string kDefaultFrameId = "ground_truth_pose";
static const std::string kDefaultLinkName = "base_link";
static const std::string kDefaultBagFilename_ = "simulator.bag";
static const std::string kDefaultRecordingServiceName = "record_rosbag";
static constexpr bool kDefaultWaitToRecord = false;
static constexpr bool kDefaultIsRecording = false;

/// \brief This plugin is used to create rosbag files from within gazebo.
class GazeboBagPlugin : public ModelPlugin {
  typedef std::map<const unsigned int, const physics::JointPtr> MotorNumberToJointMap;
  typedef std::pair<const unsigned int, const physics::JointPtr> MotorNumberToJointPair;
 public:
  GazeboBagPlugin()
      : ModelPlugin(),
        namespace_(kDefaultNamespace),
        ground_truth_pose_topic_(mav_msgs::default_topics::GROUND_TRUTH_POSE),
        ground_truth_twist_topic_(mav_msgs::default_topics::GROUND_TRUTH_TWIST),
        imu_topic_(mav_msgs::default_topics::IMU),
        control_attitude_thrust_topic_(mav_msgs::default_topics::COMMAND_ATTITUDE_THRUST),
        control_motor_speed_topic_(mav_msgs::default_topics::COMMAND_ACTUATORS),
        control_rate_thrust_topic_(mav_msgs::default_topics::COMMAND_RATE_THRUST),
        motor_topic_(mav_msgs::default_topics::MOTOR_MEASUREMENT),
        wrench_topic_(mav_msgs::default_topics::WRENCH),
        wind_topic_(mav_msgs::default_topics::WIND),
        waypoint_topic_(mav_msgs::default_topics::COMMAND_TRAJECTORY),
        command_pose_topic_(mav_msgs::default_topics::COMMAND_POSE),
        frame_id_(kDefaultFrameId),
        link_name_(kDefaultLinkName),
        bag_filename_(kDefaultBagFilename_),
        recording_service_name_(kDefaultRecordingServiceName),
        rotor_velocity_slowdown_sim_(kDefaultRotorVelocitySlowdownSim),
        wait_to_record_(kDefaultWaitToRecord),
        is_recording_(kDefaultIsRecording),
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

  /// \brief Starting recording the rosbag
  void StartRecording();

  /// \brief Stop recording the rosbag
  void StopRecording();

  /// \brief Called when an IMU message is received.
  /// \param[in] imu_msg A IMU message from sensor_msgs.
  void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg);

  /// \brief Called when an Wind message is received.
  /// \param[in] wind_msg A WrenchStamped message from geometry_msgs.
  void WindCallback(const geometry_msgs::WrenchStampedConstPtr& wind_msg);

  /// \brief Called when a MultiDOFJointTrajectoryPoint message is received.
  /// \param[in] trajectory_msg A MultiDOFJointTrajectoryPoint message from trajectory_msgs.
  void WaypointCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_msg);

  /// \brief Called when a PoseStamped message is received.
  /// \param[in] pose_msg A PoseStamped message from geometry_msgs.
  void CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg);

  /// \brief Called when a AttitudeThrust message is received.
  /// \param[in] control_msg A AttitudeThrust message from mav_msgs.
  void AttitudeThrustCallback(const mav_msgs::AttitudeThrustConstPtr& control_msg);

  /// \brief Called when a Actuators message is received.
  /// \param[in] control_msg A Actuators message from mav_msgs.
  void ActuatorsCallback(const mav_msgs::ActuatorsConstPtr& control_msg);

  /// \brief Called when a RateThrust message is received.
  /// \param[in] control_msg A RateThrust message from mav_msgs.
  void RateThrustCallback(const mav_msgs::RateThrustConstPtr& control_msg);

  /// \brief Log the ground truth pose and twist.
  /// \param[in] now The current gazebo common::Time
  void LogGroundTruth(const common::Time now);

  /// \brief Log all the motor velocities.
  /// \param[in] now The current gazebo common::Time
  void LogMotorVelocities(const common::Time now);

  /// \brief Log all the wrenches.
  /// \param[in] now The current gazebo common::Time
  void LogWrenches(const common::Time now);

  /// \brief Called when a request to start or stop recording is received.
  /// \param[in] req The request to start or stop recording.
  /// \param[out] res The response to be sent back to the client.
  bool RecordingServiceCallback(rotors_comm::RecordRosbag::Request& req,
                                rotors_comm::RecordRosbag::Response& res);

 private:
  /// \brief Pointer to the update event connection.
  event::ConnectionPtr update_connection_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;

  physics::Link_V child_links_;

  MotorNumberToJointMap motor_joints_;

  /// \brief Pointer to the ContactManager to get all collisions of this
  /// link and its children
  physics::ContactManager *contact_mgr_;

  std::string namespace_;
  std::string ground_truth_pose_topic_;
  std::string ground_truth_twist_topic_;
  std::string imu_topic_;
  std::string wind_topic_;
  std::string waypoint_topic_;
  std::string command_pose_topic_;
  std::string control_attitude_thrust_topic_;
  std::string control_motor_speed_topic_;
  std::string control_rate_thrust_topic_;
  std::string wrench_topic_;
  std::string motor_topic_;
  std::string frame_id_;
  std::string link_name_;
  std::string bag_filename_;
  std::string recording_service_name_;
  double rotor_velocity_slowdown_sim_;

  /// \brief Mutex lock for thread safety of writing bag files
  boost::mutex mtx_;

  /// \brief Whether the plugin should wait for user command to start recording
  bool wait_to_record_;

  /// \brief Whether the plugin is currenly recording a rosbag
  bool is_recording_;

  rosbag::Bag bag_;
  ros::NodeHandle *node_handle_;

  // Ros subscribers
  ros::Subscriber imu_sub_;
  ros::Subscriber wind_sub_;
  ros::Subscriber waypoint_sub_;
  ros::Subscriber control_attitude_thrust_sub_;
  ros::Subscriber control_motor_speed_sub_;
  ros::Subscriber control_rate_thrust_sub_;
  ros::Subscriber command_pose_sub_;

  // Ros service server
  ros::ServiceServer recording_service_;

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

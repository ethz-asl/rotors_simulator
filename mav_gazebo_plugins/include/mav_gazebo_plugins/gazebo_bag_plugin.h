/*
 * Copyright (C) 2014 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Pascal Gohl, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Sammy Omari, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * This software is released to the Contestants of the european 
 * robotics challenges (EuRoC) for the use in stage 1. (Re)-distribution, whether 
 * in parts or entirely, is NOT PERMITTED. 
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 */

#include <string>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <mav_msgs/CommandAttitudeThrust.h>
#include <mav_msgs/CommandMotorSpeed.h>
#include <mav_msgs/CommandRateThrust.h>
#include <mav_msgs/CommandTrajectory.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>

namespace gazebo {
/// \brief This plugin is used to create rosbag files in within gazebo.
class GazeboBagPlugin : public ModelPlugin {
  typedef std::map<const unsigned int, const physics::JointPtr> MotorNumberToJointMap;
  typedef std::pair<const unsigned int, const physics::JointPtr> MotorNumberToJointPair;
 public:
  /// \brief Constructor
  GazeboBagPlugin();
  /// \brief Destructor
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
  void ImuCallback(const sensor_msgs::ImuPtr& imu_msg);

  /// \brief Called when an Wind message is received.
  /// \param[in] wind_msg A WrenchStamped message from geometry_msgs.
  void WindCallback(const geometry_msgs::WrenchStampedPtr& wind_msg);

  /// \brief Called when an Trajectory message is received.
  /// \param[in] trajectory_msg A CommandTrajectory message from mav_msgs.
  void WaypointCallback(const mav_msgs::CommandTrajectoryPtr& trajectory_msg);

  /// \brief Called when a CommandAttitudeThrust message is received.
  /// \param[in] control_msg A CommandAttitudeThrust message from mav_msgs.
  void CommandAttitudeThrustCallback(const mav_msgs::CommandAttitudeThrustPtr& control_msg);

  /// \brief Called when a CommandMotorSpeed message is received.
  /// \param[in] control_msg A CommandMotorSpeed message from mav_msgs.
  void CommandMotorSpeedCallback(const mav_msgs::CommandMotorSpeedPtr& control_msg);

  /// \brief Called when a CommandRateThrust message is received.
  /// \param[in] control_msg A CommandRateThrust message from mav_msgs.
  void CommandRateThrustCallback(const mav_msgs::CommandRateThrustPtr& control_msg);

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
  /// \brief The connections.
  event::ConnectionPtr update_connection_;

  /// \brief Pointer to the world.
  physics::WorldPtr world_;

  /// \brief Pointer to the model.
  physics::ModelPtr model_;

  /// \brief Pointer to the link.
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
  std::string exclude_floor_link_from_collision_check_;
  double rotor_velocity_slowdown_sim_;
  double mass_;
  double gravity_;
  double gravitational_force_exclusion_multiplier_;

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

//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>
// All rights reserved.
//
// TODO(ff): Enter some license
//==============================================================================

#include <string>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <mav_msgs/ControlAttitudeThrust.h>
#include <mav_msgs/ControlMotorSpeed.h>
#include <mav_msgs/ControlRateThrust.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>


namespace gazebo
{
  /// \brief This plugin is used to create rosbag files in within gazebo.
  class GazeboBagPlugin : public ModelPlugin
  {
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

      /// \brief Called when a ControlAttitudeThrust message is received.
      /// \param[in] control_msg A ControlAttitudeThrust message from mav_msgs.
      void ControlAttitudeThrustCallback(
        const mav_msgs::ControlAttitudeThrustPtr& control_msg);

      /// \brief Called when a ControlMotorSpeed message is received.
      /// \param[in] control_msg A ControlMotorSpeed message from mav_msgs.
      void ControlMotorSpeedCallback(
        const mav_msgs::ControlMotorSpeedPtr& control_msg);

      /// \brief Called when a ControlRateThrust message is received.
      /// \param[in] control_msg A ControlRateThrust message from mav_msgs.
      void ControlRateThrustCallback(
        const mav_msgs::ControlRateThrustPtr& control_msg);

      /// \brief Log the ground truth pose and twist.
      /// \param[in] now The current gazebo common::Time
      void LogGroundTruth(const common::Time now);

      /// \brief Log all the motor velocities.
      /// \param[in] now The current gazebo common::Time
      void LogMotorVelocities(const common::Time now);

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

      /// \brief The pose of the model.
      math::Pose pose_;

      // /// \brief Pointer to the ContactManager to get all collisions of this 
      // /// link and its children
      // physics::ContactManager *contact_mgr_;

      // /// \brief The collisions for the link and its children in the model.
      // std::map<std::string, physics::CollisionPtr> collisions;

      std::string namespace_;
      std::string ground_truth_pose_pub_topic_;
      std::string ground_truth_twist_pub_topic_;
      std::string imu_pub_topic_;
      std::string imu_sub_topic_;
      std::string control_attitude_thrust_pub_topic_;
      std::string control_attitude_thrust_sub_topic_;
      std::string control_motor_speed_pub_topic_;
      std::string control_motor_speed_sub_topic_;
      std::string control_rate_thrust_pub_topic_;
      std::string control_rate_thrust_sub_topic_;
      std::string motor_pub_topic_;
      std::string frame_id_;
      std::string link_name_;
      std::string bag_filename_;

      /// \brief Mutex lock for thread safty of writing bag files
      boost::mutex mtx_;

      rosbag::Bag bag_;
      ros::NodeHandle *node_handle_;

      // Ros subscribers
      ros::Subscriber imu_sub_;
      ros::Subscriber control_attitude_thrust_sub_;
      ros::Subscriber control_motor_speed_sub_;
      ros::Subscriber control_rate_thrust_sub_;

      std::ofstream csvOut;
  };
}

#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_LANDING_TARGET_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_LANDING_TARGET_PLUGIN_H

#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <mav_msgs/default_topics.h> // This comes from the mav_comm repo

#include "Odometry.pb.h"
#include "JointState.pb.h"
#include "TransformStampedWithFrameIds.pb.h"

#include "rotors_gazebo_plugins/common.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

namespace gazebo
{

// Default values
static const std::string kDefaultLinkName = "base_link";
static const std::string kDefaultFrameId = "base_link";
static const std::string kDefaultJointStatePubTopic = "joint_states";

/// \brief This plugin publishes the motor speeds of your multirotor model.
class GazeboLandingTargetPlugin : public ModelPlugin
{

public:
    GazeboLandingTargetPlugin()
        : ModelPlugin(),
          namespace_(kDefaultNamespace),
          joint_state_pub_topic_(kDefaultJointStatePubTopic),
          link_name_(kDefaultLinkName),
          frame_id_(kDefaultFrameId),
          node_handle_(NULL),
          pubs_and_subs_created_(false) {}

    virtual ~GazeboLandingTargetPlugin();

protected:
    /// \brief Load the plugin.
    /// \param[in] _model Pointer to the model that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Called when the world is updated.
    /// \param[in] _info Update timing information.
    void OnUpdate(const common::UpdateInfo & /*_info*/);

private:
    /// \brief    Flag that is set to true once CreatePubsAndSubs() is called, used
    ///           to prevent CreatePubsAndSubs() from be called on every OnUpdate().
    bool pubs_and_subs_created_;

    /// \brief    Creates all required publishers and subscribers, incl. routing of messages to/from ROS if required.
    /// \details  Call this once the first time OnUpdate() is called (can't
    ///           be called from Load() because there is no guarantee GazeboRosInterfacePlugin has
    ///           has loaded and listening to ConnectGazeboToRosTopic and ConnectRosToGazeboTopic messages).
    void CreatePubsAndSubs();

    /// \brief    Callback to listen for commands to change position of the model.
    /// \details  Runs as a callback whenever a ROS message is published to the topic.
    void PoseCallback(const geometry_msgs::Pose::ConstPtr &msg);

    /// \brief    Callback to listen for commands to change velocity of the model.
    /// \details  Runs as a callback whenever a ROS message is published to the topic.
    void TwistCallback(const geometry_msgs::Twist::ConstPtr &msg);

    /// \brief Pointer to the update event connection.
    event::ConnectionPtr update_connection_;

    physics::WorldPtr world_;
    physics::ModelPtr model_;
    physics::LinkPtr link_;

    std::string namespace_;
    std::string joint_state_pub_topic_;
    std::string twist_pub_topic_;
    std::string pose_pub_topic_;
    std::string link_name_;
    std::string frame_id_;

    ros::Subscriber twist_sub_;
    ros::Subscriber pose_sub_;

    gazebo::transport::PublisherPtr odom_pub_;

    /// \details    Re-used message object, defined here to reduce dynamic memory allocation.
    gz_geometry_msgs::Odometry odom_msg_;

    gazebo::transport::PublisherPtr joint_state_pub_;

    /// \details    Re-used message object, defined here to reduce dynamic memory allocation.
    gz_sensor_msgs::JointState joint_state_msg_;

    /// \brief    Special-case publisher to publish stamped transforms with
    ///           frame IDs. The ROS interface plugin (if present) will
    ///           listen to this publisher and broadcast the transform
    ///           using transform_broadcast().
    gazebo::transport::PublisherPtr broadcast_transform_pub_;

    gazebo::transport::NodePtr node_handle_;
    ros::NodeHandle *ros_node_handle_;
};

} // namespace gazebo

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_LANDING_TARGET_PLUGIN_H

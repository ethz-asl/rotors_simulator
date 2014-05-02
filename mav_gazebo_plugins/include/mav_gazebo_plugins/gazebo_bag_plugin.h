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

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>

namespace gazebo
{
  /// \brief This plugin is used to create rosbag files in within gazebo.
  class GazeboBagPlugin : public ModelPlugin
  {
    public:
      /// \brief Constructor
      GazeboBagPlugin();
      /// \brief Destructor
      virtual ~GazeboBagPlugin();

      
    protected:
      /// \brief Load the plugin.
      /// \param[in] _model Number of command line arguments.
      /// \param[in] _sdf Array of command line arguments.
      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

      /// \brief Called when the world is updated.
      /// \param[in] _info Update timing information.
      void OnUpdate(const common::UpdateInfo& /*_info*/);
    private:
      /// \brief The connections.
      event::ConnectionPtr update_connection_;

      /// \brief Pointer to the world.
      physics::WorldPtr world_;

      /// \brief Pointer to the model.
      physics::ModelPtr model_;

      /// \brief Pointer to the link.
      physics::LinkPtr link_;

      /// \brief The pose of the model.
      math::Pose pose_;

      std::string namespace_;
      std::string pose_topic_;
      std::string frame_id_;
      std::string link_name_;
      std::string bag_filename_;

      rosbag::Bag bag_;
      ros::NodeHandle *node_handle_;

      ros::Publisher ground_truth_pose_pub_;
      ros::Publisher ground_truth_twist_pub_;
      // ros::Publisher collision_positions_pub_;

      ros::Publisher time_pub_;

      geometry_msgs::PoseStamped pose_msg_;
      geometry_msgs::TwistStamped twist_msg_;

      std::ofstream csvOut;
  };
}

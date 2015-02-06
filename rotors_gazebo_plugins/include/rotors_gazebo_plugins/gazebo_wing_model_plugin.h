/*
 * Copyright (C) 2014 Roman Bapst, CVG, ETH Zurich, Switzerland
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
#include <Eigen/Eigen>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
 #include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace gazebo {
/// \brief This plugin is used to create rosbag files in within gazebo.
class GazeboWingModelPlugin : public ModelPlugin {
  //typedef std::map<const unsigned int, const physics::JointPtr> MotorNumberToJointMap;
  //typedef std::pair<const unsigned int, const physics::JointPtr> MotorNumberToJointPair;
 public:
  /// \brief Constructor
  GazeboWingModelPlugin();
  /// \brief Destructor
  virtual ~GazeboWingModelPlugin();

 protected:
  /// \brief Load the plugin.
  /// \param[in] _model Pointer to the model that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Called when the world is updated.
  /// \param[in] _info Update timing information.
  void OnUpdate(const common::UpdateInfo& /*_info*/);
  void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose);
  void UpdateForcesAndMoments();
  void Publish();
  math::Vector3 get_aerodynamic_forces(math::Vector3 &vel);
  math::Vector3 get_aerodynamic_moments(math::Vector3 &vel);

 private:
  /// \brief The connections.
  event::ConnectionPtr updateConnection_;

  /// \brief Pointer to the world.
  physics::WorldPtr world_;

  /// \brief Pointer to the model.
  physics::ModelPtr model_;

  /// \brief Pointer to the link.
  physics::LinkPtr link_;

  std::string namespace_;
  std::string wing_pub_topic_;
  std::string frame_id_;
  std::string link_name_;
  std::string _pose_topic;

  // orientation represented as quaternion
  math::Quaternion _q;
  // elevon positions looking from behind, range [-1,1] -> 0 is neutral position
  float _elevon_pos_left;
  float _elevon_pos_right;

  float _rho;
  float _F;
  
  ros::Publisher wing_pub_;
  ros::NodeHandle *node_handle_;
  ros::Subscriber _pose_sub;
};
}

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

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>

namespace gazebo {
/// \brief This gazebo plugin simulates wind acting on a model.
class GazeboWindPlugin : public ModelPlugin {
 public:
  /// \brief Constructor
  GazeboWindPlugin();
  /// \brief Destructor
  virtual ~GazeboWindPlugin();

 protected:
  /// \brief Load the plugin.
  /// \param[in] _model Pointer to the model that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
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

  std::string namespace_;

  std::string frame_id_;
  std::string link_name_;
  std::string wind_pub_topic_;

  math::Vector3 xyz_offset_;

  double wind_force_mean_;
  double wind_force_variance_;
  math::Vector3 wind_direction_;

  common::Time wind_gust_end_;
  common::Time wind_gust_start_;
  double wind_gust_force_mean_;
  double wind_gust_force_variance_;
  math::Vector3 wind_gust_direction_;

  ros::Publisher wind_pub_;

  ros::NodeHandle *node_handle_;
};
}

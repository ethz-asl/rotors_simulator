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
// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultWindPubTopic = "/wind";
static const std::string kDefaultFrameId = "world";
static const std::string kDefaultLinkName = "base_link";

static constexpr double kDefaultWindForceMean = 0.0;
static constexpr double kDefaultWindForceVariance = 0.0;
static constexpr double kDefaultWindGustForceMean = 0.0;
static constexpr double kDefaultWindGustForceVariance = 0.0;

static constexpr double kDefaultWindGustStart = 10.0;
static constexpr double kDefaultWindGustDuration = 0.0;

static const math::Vector3 kDefaultWindDirection = math::Vector3(1, 0, 0);
static const math::Vector3 kDefaultWindGustDirection = math::Vector3(0, 1, 0);



/// \brief This gazebo plugin simulates wind acting on a model.
class GazeboWindPlugin : public ModelPlugin {
 public:
  /// \brief Constructor
  GazeboWindPlugin()
      : ModelPlugin(),
        namespace_(kDefaultNamespace),
        wind_pub_topic_(kDefaultWindPubTopic),
        wind_force_mean_(kDefaultWindForceMean),
        wind_force_variance_(kDefaultWindForceVariance),
        wind_gust_force_mean_(kDefaultWindGustForceMean),
        wind_gust_force_variance_(kDefaultWindGustForceVariance),
        wind_direction_(kDefaultWindDirection),
        wind_gust_direction_(kDefaultWindGustDirection),
        frame_id_(kDefaultFrameId),
        link_name_(kDefaultLinkName),
        node_handle_(NULL) {}

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

  double wind_force_mean_;
  double wind_force_variance_;
  double wind_gust_force_mean_;
  double wind_gust_force_variance_;

  math::Vector3 xyz_offset_;
  math::Vector3 wind_direction_;
  math::Vector3 wind_gust_direction_;

  common::Time wind_gust_end_;
  common::Time wind_gust_start_;

  ros::Publisher wind_pub_;

  ros::NodeHandle *node_handle_;
};
}

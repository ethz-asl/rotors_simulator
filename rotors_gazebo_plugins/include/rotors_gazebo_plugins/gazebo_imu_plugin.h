/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#ifndef ROTORS_GAZEBO_PLUGINS_IMU_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_IMU_PLUGIN_H

#include <random>

#include <Eigen/Core>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "Imu.pb.h"

#include "rotors_gazebo_plugins/common.h"
#include "rotors_gazebo_plugins/uav_parameters.h"

namespace gazebo {

class GazeboImuPlugin : public ModelPlugin {
 public:

  GazeboImuPlugin();
  ~GazeboImuPlugin();

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief  This method adds noise to acceleration and angular rates for
  ///         accelerometer and gyroscope measurement simulation.
  void AddNoise(
      Eigen::Vector3d* linear_acceleration,
      Eigen::Vector3d* angular_velocity,
      const double dt);

  /// \brief  	This gets called by the world update start event.
  /// \details	Calculates IMU parameters and then publishes one IMU message.
  void OnUpdate(const common::UpdateInfo&);

 private:

  /// \brief    Flag that is set to true once CreatePubsAndSubs() is called, used
  ///           to prevent CreatePubsAndSubs() from be called on every OnUpdate().
  bool pubs_and_subs_created_;

  /// \brief    Creates all required publishers and subscribers, incl. routing of messages to/from ROS if required.
  /// \details  Call this once the first time OnUpdate() is called (can't
  ///           be called from Load() because there is no guarantee GazeboRosInterfacePlugin has
  ///           has loaded and listening to ConnectGazeboToRosTopic and ConnectRosToGazeboTopic messages).
  void CreatePubsAndSubs();


  std::string namespace_;
  std::string imu_topic_;

  /// \brief    Handle for the Gazebo node.
  transport::NodePtr node_handle_;

  transport::PublisherPtr imu_pub_;

  std::string frame_id_;
  std::string link_name_;

  std::default_random_engine random_generator_;
  std::normal_distribution<double> standard_normal_distribution_;

  /// \brief    Pointer to the world.
  physics::WorldPtr world_;

  /// \brief    Pointer to the model.
  physics::ModelPtr model_;

  /// \brief    Pointer to the link.
  physics::LinkPtr link_;

  /// \brief    Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  common::Time last_time_;

  /// \brief    IMU message.
  /// \details  This is modified everytime OnUpdate() is called,
  //            and then published onto a topic
  gz_sensor_msgs::Imu imu_message_;

  ignition::math::Vector3d gravity_W_;
  ignition::math::Vector3d velocity_prev_W_;

  Eigen::Vector3d gyroscope_bias_;
  Eigen::Vector3d accelerometer_bias_;

  Eigen::Vector3d gyroscope_turn_on_bias_;
  Eigen::Vector3d accelerometer_turn_on_bias_;

  ImuParameters imu_parameters_;

  uint64_t seq_ = 0;
};

}  // namespace gazebo

#endif // ROTORS_GAZEBO_PLUGINS_IMU_PLUGIN_H

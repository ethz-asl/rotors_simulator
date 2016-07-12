/*
 * Copyright 2016 Pavel Vechersky, ASL, ETH Zurich, Switzerland
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

#include "rotors_gazebo_plugins/gazebo_magnetometer_plugin.h"

namespace gazebo {

GazeboMagnetometerPlugin::GazeboMagnetometerPlugin()
    : ModelPlugin(),
      node_handle_(0),
      random_generator_(random_device_()) {
}

GazeboMagnetometerPlugin::~GazeboMagnetometerPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboMagnetometerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model and the world
  model_ = _model;
  world_ = model_->GetWorld();

  // Use the robot namespace to create the node handle
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_magnetometer_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  // Use the link name as the frame id
  std::string link_name;
  if (_sdf->HasElement("linkName"))
    link_name = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_magnetometer_plugin] Please specify a linkName.\n";
  // Get the pointer to the link
  link_ = model_->GetLink(link_name);
  if (link_ == NULL)
    gzthrow("[gazebo_magnetometer_plugin] Couldn't find specified link \"" << link_name << "\".");

  frame_id_ = link_name;

  double ref_mag_north;
  double ref_mag_east;
  double ref_mag_down;
  SdfVector3 noise_normal;
  SdfVector3 noise_uniform_initial_bias;
  const SdfVector3 zeros3(0.0, 0.0, 0.0);

  // Retrieve the rest of the SDF parameters
  getSdfParam<std::string>(_sdf, "magnetometerTopic", magnetometer_topic_,
                           mav_msgs::default_topics::MAGNETIC_FIELD);
  getSdfParam<double>(_sdf, "refMagNorth", ref_mag_north, kDefaultRefMagNorth);
  getSdfParam<double>(_sdf, "refMagEast", ref_mag_east, kDefaultRefMagEast);
  getSdfParam<double>(_sdf, "refMagDown", ref_mag_down, kDefaultRefMagDown);
  getSdfParam<SdfVector3>(_sdf, "noiseNormal", noise_normal, zeros3);
  getSdfParam<SdfVector3>(_sdf, "noiseUniformInitialBias", noise_uniform_initial_bias, zeros3);

  // Listen to the update event. This event is broadcast every simulation iteration.
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboMagnetometerPlugin::OnUpdate, this, _1));

  magnetometer_pub_ = node_handle_->advertise<sensor_msgs::MagneticField>(magnetometer_topic_, 1);

  // Create the normal noise distributions
  noise_n_[0] = NormalDistribution(0, noise_normal.X());
  noise_n_[1] = NormalDistribution(0, noise_normal.Y());
  noise_n_[2] = NormalDistribution(0, noise_normal.Z());

  // Create the uniform noise distribution for initial bias
  UniformDistribution initial_bias[3];
  initial_bias[0] = UniformDistribution(-noise_uniform_initial_bias.X(),
                                        noise_uniform_initial_bias.X());
  initial_bias[1] = UniformDistribution(-noise_uniform_initial_bias.Y(),
                                        noise_uniform_initial_bias.Y());
  initial_bias[2] = UniformDistribution(-noise_uniform_initial_bias.Z(),
                                        noise_uniform_initial_bias.Z());

  // Initialize the reference magnetic field vector in world frame, taking into
  // account the initial bias
  mag_W_ = math::Vector3(ref_mag_north + initial_bias[0](random_generator_),
                         ref_mag_east + initial_bias[1](random_generator_),
                         ref_mag_down + initial_bias[2](random_generator_));

  // Fill the magnetometer message
  mag_message_.header.frame_id = frame_id_;
  mag_message_.magnetic_field_covariance[0] = noise_normal.X() * noise_normal.X();
  mag_message_.magnetic_field_covariance[4] = noise_normal.Y() * noise_normal.Y();
  mag_message_.magnetic_field_covariance[8] = noise_normal.Z() * noise_normal.Z();
}

void GazeboMagnetometerPlugin::OnUpdate(const common::UpdateInfo& _info) {
  // Get the current pose and time from Gazebo
  math::Pose T_W_B = link_->GetWorldPose();
  common::Time current_time  = world_->GetSimTime();

  // Calculate the magnetic field noise.
  math::Vector3 mag_noise(noise_n_[0](random_generator_),
                          noise_n_[1](random_generator_),
                          noise_n_[2](random_generator_));

  // Rotate the earth magnetic field into the inertial frame
  math::Vector3 field_B = T_W_B.rot.RotateVectorReverse(mag_W_ + mag_noise);

  // Fill the magnetic field message
  mag_message_.header.stamp.sec = current_time.sec;
  mag_message_.header.stamp.nsec = current_time.nsec;
  mag_message_.magnetic_field.x = field_B.x;
  mag_message_.magnetic_field.y = field_B.y;
  mag_message_.magnetic_field.z = field_B.z;

  // Publish the message
  magnetometer_pub_.publish(mag_message_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMagnetometerPlugin);
}

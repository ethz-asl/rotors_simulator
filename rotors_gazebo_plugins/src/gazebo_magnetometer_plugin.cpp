/*
 * Copyright 2016 Pavel Vechersky, ASL, ETH Zurich, Switzerland
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

// MODULE HEADER INCLUDE
#include "rotors_gazebo_plugins/gazebo_magnetometer_plugin.h"

#include <mav_msgs/default_topics.h>  // This comes from the mav_comm repo

#include "ConnectGazeboToRosTopic.pb.h"

namespace gazebo {

GazeboMagnetometerPlugin::GazeboMagnetometerPlugin()
    : ModelPlugin(),
      random_generator_(random_device_()),
      pubs_and_subs_created_(false) {
  // Nothing
}

GazeboMagnetometerPlugin::~GazeboMagnetometerPlugin() {
}

void GazeboMagnetometerPlugin::Load(physics::ModelPtr _model,
                                    sdf::ElementPtr _sdf) {
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  // Store the pointer to the model and the world
  model_ = _model;
  world_ = model_->GetWorld();

  // Use the robot namespace to create the node handle
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_magnetometer_plugin] Please specify a robotNamespace.\n";

  node_handle_ = gazebo::transport::NodePtr(new transport::Node());

  // Initialise with default namespace (typically /gazebo/default/)
  node_handle_->Init();

  // Use the link name as the frame id
  std::string link_name;
  if (_sdf->HasElement("linkName"))
    link_name = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_magnetometer_plugin] Please specify a linkName.\n";
  // Get the pointer to the link
  link_ = model_->GetLink(link_name);
  if (link_ == NULL)
    gzthrow("[gazebo_magnetometer_plugin] Couldn't find specified link \""
            << link_name << "\".");

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
  getSdfParam<SdfVector3>(_sdf, "noiseUniformInitialBias",
                          noise_uniform_initial_bias, zeros3);

  // Listen to the update event. This event is broadcast every simulation
  // iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboMagnetometerPlugin::OnUpdate, this, _1));

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
  mag_W_ = ignition::math::Vector3d (ref_mag_north + initial_bias[0](random_generator_),
                         ref_mag_east + initial_bias[1](random_generator_),
                         ref_mag_down + initial_bias[2](random_generator_));

  // Fill the static parts of the magnetometer message.
  mag_message_.mutable_header()->set_frame_id(frame_id_);

  for (int i = 0; i < 9; i++) {
    switch (i) {
      case 0:
        mag_message_.add_magnetic_field_covariance(noise_normal.X() *
                                                   noise_normal.X());
        break;
      case 1:
      case 2:
      case 3:
        mag_message_.add_magnetic_field_covariance(0);
        break;
      case 4:
        mag_message_.add_magnetic_field_covariance(noise_normal.Y() *
                                                   noise_normal.Y());
        break;
      case 5:
      case 6:
      case 7:
        mag_message_.add_magnetic_field_covariance(0);
        break;
      case 8:
        mag_message_.add_magnetic_field_covariance(noise_normal.Z() *
                                                   noise_normal.Z());
        break;
    }
  }
}

void GazeboMagnetometerPlugin::OnUpdate(const common::UpdateInfo& _info) {
  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  // Get the current pose and time from Gazebo
  ignition::math::Pose3d T_W_B = link_->WorldPose();
  common::Time current_time = world_->SimTime();

  // Calculate the magnetic field noise.
  ignition::math::Vector3d mag_noise(noise_n_[0](random_generator_),
                          noise_n_[1](random_generator_),
                          noise_n_[2](random_generator_));

  // Rotate the earth magnetic field into the inertial frame
  ignition::math::Vector3d field_B = T_W_B.Rot().RotateVectorReverse(mag_W_ + mag_noise);

  // Fill the magnetic field message
  mag_message_.mutable_header()->mutable_stamp()->set_sec(current_time.sec);
  mag_message_.mutable_header()->mutable_stamp()->set_nsec(current_time.nsec);
  mag_message_.mutable_magnetic_field()->set_x(field_B.X());
  mag_message_.mutable_magnetic_field()->set_y(field_B.Y());
  mag_message_.mutable_magnetic_field()->set_z(field_B.Z());

  // Publish the message
  magnetometer_pub_->Publish(mag_message_);
}

void GazeboMagnetometerPlugin::CreatePubsAndSubs() {
  // Create temporary "ConnectGazeboToRosTopic" publisher and message
  gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
          "~/" + kConnectGazeboToRosSubtopic, 1);

  // ============================================ //
  // ========= MAGNETIC FIELD MSG SETUP ========= //
  // ============================================ //

  magnetometer_pub_ = node_handle_->Advertise<gz_sensor_msgs::MagneticField>(
      "~/" + namespace_ + "/" + magnetometer_topic_, 1);

  gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;
  // connect_gazebo_to_ros_topic_msg.set_gazebo_namespace(namespace_);
  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   magnetometer_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                magnetometer_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::MAGNETIC_FIELD);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMagnetometerPlugin);

}  // namespace gazebo

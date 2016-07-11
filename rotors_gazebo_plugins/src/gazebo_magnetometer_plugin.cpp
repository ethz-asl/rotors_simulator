/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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
    : SensorPlugin(),
      node_handle_(0) {}

GazeboMagnetometerPlugin::~GazeboMagnetometerPlugin() {
  this->parent_sensor_->DisconnectUpdated(this->updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboMagnetometerPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
  // Store the pointer to the parent sensor
  parent_sensor_ = std::dynamic_pointer_cast<sensors::MagnetometerSensor>(_sensor);

  // Retrieve the necessary parameters
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_magnetometer_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_magnetometer_plugin] Please specify a linkName.\n";

  frame_id_ = link_name_;

  getSdfParam<std::string>(_sdf, "magnetometerTopic", magnetometer_topic_,
                             mav_msgs::default_topics::MAGNETIC_FIELD);

  // Connect to the sensor update event.
  this->updateConnection_ =
      this->parent_sensor_->ConnectUpdated(
          boost::bind(&GazeboMagnetometerPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  parent_sensor_->SetActive(true);

  // Initialize the ROS publisher for sending magnetic field messages.
  magnetometer_pub_ = node_handle_->advertise<sensor_msgs::MagneticField>(magnetometer_topic_, 1);

  // Get the sensor noise in case it is known.
  double std_dev_mag_x = 0.0;
  double std_dev_mag_y = 0.0;
  double std_dev_mag_z = 0.0;

  sensors::NoisePtr mag_x_noise = this->parent_sensor_->Noise(sensors::SensorNoiseType::MAGNETOMETER_X_NOISE_TESLA);
  sensors::NoisePtr mag_y_noise = this->parent_sensor_->Noise(sensors::SensorNoiseType::MAGNETOMETER_Y_NOISE_TESLA);
  sensors::NoisePtr mag_z_noise = this->parent_sensor_->Noise(sensors::SensorNoiseType::MAGNETOMETER_Z_NOISE_TESLA);

  if (mag_x_noise->GetNoiseType() == sensors::Noise::GAUSSIAN) {
    sensors::GaussianNoiseModelPtr mag_x_gaussian_noise =
        std::dynamic_pointer_cast<sensors::GaussianNoiseModel>(mag_x_noise);
    std_dev_mag_x = mag_x_gaussian_noise->GetStdDev();
  }

  if (mag_y_noise->GetNoiseType() == sensors::Noise::GAUSSIAN) {
    sensors::GaussianNoiseModelPtr mag_y_gaussian_noise =
        std::dynamic_pointer_cast<sensors::GaussianNoiseModel>(mag_y_noise);
    std_dev_mag_y = mag_y_gaussian_noise->GetStdDev();
  }

  if (mag_z_noise->GetNoiseType() == sensors::Noise::GAUSSIAN) {
    sensors::GaussianNoiseModelPtr mag_z_gaussian_noise =
        std::dynamic_pointer_cast<sensors::GaussianNoiseModel>(mag_z_noise);
    std_dev_mag_z = mag_z_gaussian_noise->GetStdDev();
  }

  // Fill the magnetic field message.
  magnetic_field_message_.header.frame_id = frame_id_;
  magnetic_field_message_.magnetic_field_covariance[0] = std_dev_mag_x * std_dev_mag_x;
  magnetic_field_message_.magnetic_field_covariance[4] = std_dev_mag_y * std_dev_mag_y;
  magnetic_field_message_.magnetic_field_covariance[8] = std_dev_mag_z * std_dev_mag_z;
}

void GazeboMagnetometerPlugin::OnUpdate() {
  common::Time current_time = parent_sensor_->LastMeasurementTime();

  // Retrieve the latest magnetic field measurement
  ignition::math::Vector3d magnetic_field = parent_sensor_->MagneticField();
  magnetic_field_message_.magnetic_field.x = magnetic_field.X();
  magnetic_field_message_.magnetic_field.y = magnetic_field.Y();
  magnetic_field_message_.magnetic_field.z = magnetic_field.Z();

  // Update the message header.
  magnetic_field_message_.header.stamp.sec = current_time.sec;
  magnetic_field_message_.header.stamp.nsec = current_time.nsec;

  // Publish the magnetic field message.
  magnetometer_pub_.publish(magnetic_field_message_);
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboMagnetometerPlugin);
}

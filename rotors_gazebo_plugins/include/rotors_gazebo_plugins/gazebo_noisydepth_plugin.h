/*
 * Copyright 2018 Michael Pantic, ASL, ETH Zurich, Switzerland
 *
 * Forked from Openni/Kinect Depth Plugin, retaining original copyright header:
 *
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 * Desc: A dynamic controller plugin that publishes ROS image_raw camera_info
 * topic for generic camera sensor.
 * Author: John Hsu
 * Date: 24 Sept 2008
 */

#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_NOISYDEPTH_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_NOISYDEPTH_PLUGIN_H

#include <dynamic_reconfigure/server.h>
#include <boost/thread/mutex.hpp>

#include <gazebo_plugins/GazeboRosOpenniKinectConfig.h>
#include <gazebo_plugins/gazebo_ros_camera_utils.h>
#include <gazebo/common/Time.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/plugins/DepthCameraPlugin.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/TransportTypes.hh>

#include <image_transport/image_transport.h>
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <sdf/Param.hh>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/fill_image.h>
#include <std_msgs/Float64.h>

#include <rotors_gazebo_plugins/depth_noise_model.hpp>

namespace gazebo {
class GazeboNoisyDepth : public DepthCameraPlugin, GazeboRosCameraUtils {
 public:
  GazeboNoisyDepth();

  ~GazeboNoisyDepth();

  /// \brief Load the plugin
  /// \param take in SDF root element
  virtual void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

  /// \brief Advertise depth image
  virtual void Advertise();

  virtual void OnNewDepthFrame(const float *_image, unsigned int _width,
                               unsigned int _height, unsigned int _depth,
                               const std::string &_format);

 protected:
  virtual void OnNewImageFrame(const unsigned char *_image, unsigned int _width,
                               unsigned int _height, unsigned int _depth,
                               const std::string &_format);

 private:
  void DepthImageConnect();
  void DepthImageDisconnect();
  void DepthInfoConnect();
  void DepthInfoDisconnect();
  void FillDepthImage(const float *_src);
  bool FillDepthImageHelper(const uint32_t rows_arg,
                            const uint32_t cols_arg,
                            const uint32_t step_arg,
                            const float *data_arg,
                            sensor_msgs::Image *image_msg);

  virtual void PublishCameraInfo();

  event::ConnectionPtr load_connection_;
  std::unique_ptr<DepthNoiseModel> noise_model;

  ros::Publisher depth_image_pub_;
  ros::Publisher depth_image_camera_info_pub_;

  int depth_image_connect_count_;
  int depth_info_connect_count_;

  std::string depth_image_topic_name_;
  std::string depth_image_camera_info_topic_name_;

  common::Time depth_sensor_update_time_;
  common::Time last_depth_image_camera_info_update_time_;
  sensor_msgs::Image depth_image_msg_;
};
}
#endif  // ROTORS_GAZEBO_PLUGINS_GAZEBO_NOISYDEPTH_PLUGIN_H

/*
 * Copyright 2018 Michael Pantic, ASL, ETH Zurich, Switzerland
 *
 * Forked from Openni/Kinect Depth Plugin, retaining original copyright header:
 * Copyright 2013 Open Source Robotics Foundation
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
 *
 * Desc: GazeboNoisyDepth plugin for simulating cameras in Gazebo
 * Author: Michael Pantic / John Hsu (original Depth Plugin)
 * Date: 03 Dez 18
 */
#include <rotors_gazebo_plugins/gazebo_noisydepth_plugin.h>

#include <algorithm>
#include <assert.h>

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <sdf/sdf.hh>
#include <tf/tf.h>

namespace gazebo {
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboNoisyDepth)

GazeboNoisyDepth::GazeboNoisyDepth() {
  this->depth_info_connect_count_ = 0;
  this->depth_image_connect_count_ = 0;
  this->last_depth_image_camera_info_update_time_ = common::Time(0);
}

GazeboNoisyDepth::~GazeboNoisyDepth() {}

void GazeboNoisyDepth::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
  DepthCameraPlugin::Load(_parent, _sdf);

  // copying from DepthCameraPlugin into GazeboRosCameraUtils
  this->parentSensor_ = this->parentSensor;
  this->width_ = this->width;
  this->height_ = this->height;
  this->depth_ = this->depth;
  this->format_ = this->format;
  this->camera_ = this->depthCamera;

  // using a different default
  if (!_sdf->HasElement("imageTopicName")) {
    this->image_topic_name_ = "ir/image_raw";
  }

  if (!_sdf->HasElement("cameraInfoTopicName")) {
    this->camera_info_topic_name_ = "ir/camera_info";
  }

  // depth image stuff
  if (!_sdf->HasElement("depthImageTopicName")) {
    this->depth_image_topic_name_ = "depth/image_raw";
  } else {
    this->depth_image_topic_name_ =
        _sdf->GetElement("depthImageTopicName")->Get<std::string>();
  }

  if (!_sdf->HasElement("depthImageCameraInfoTopicName")) {
    this->depth_image_camera_info_topic_name_ = "depth/camera_info";
  } else {
    this->depth_image_camera_info_topic_name_ =
        _sdf->GetElement("depthImageCameraInfoTopicName")->Get<std::string>();
  }

  // noise model stuff
  std::string noise_model;
  if (!_sdf->HasElement("depthNoiseModelName")) {
    noise_model = "Kinect";
    ROS_WARN_NAMED("NoisyDepth",
                   "depthNoiseModelName not defined, assuming 'Kinect'");
  } else {
    noise_model = _sdf->GetElement("depthNoiseModelName")->Get<std::string>();
  }

  if (boost::iequals(noise_model, "Kinect")) {
    this->noise_model.reset(new KinectDepthNoiseModel());

    /* no other properties for Kinect */
  } else if (boost::iequals(noise_model, "D435")) {
    D435DepthNoiseModel *model = new D435DepthNoiseModel();
    this->noise_model.reset(model);

    if (_sdf->HasElement("horizontal_fov")) {
      model->h_fov = _sdf->GetElement("horizontal_fov")->Get<float>();
    }

    if (_sdf->HasElement("baseline")) {
      model->baseline = _sdf->GetElement("baseline")->Get<float>();
    }

    /* Load D435 specific noise params if available*/
    if (_sdf->HasElement("D435NoiseSubpixelErr")) {
      model->subpixel_err =
          _sdf->GetElement("D435NoiseSubpixelErr")->Get<float>();
    }

    if (_sdf->HasElement("D435MaxStdev")) {
      model->max_stdev = _sdf->GetElement("D435MaxStdev")->Get<float>();
    }

    ROS_INFO_STREAM_NAMED(
        "NoisyDepth", "D435 Depth noise configuration: "
                          << "\tHorizontal FoV: " << model->h_fov << std::endl
                          << "\tBaseline: " << model->baseline << std::endl
                          << "\tSubpixel Err: " << model->subpixel_err
                          << std::endl
                          << "\tNoise StDev cutoff: " << model->max_stdev);
  } else {
    ROS_WARN_NAMED("NoisyDepth",
                   "Invalid depthNoiseModelName (%s), assuming 'Kinect'",
                   noise_model.c_str());
    this->noise_model.reset(new KinectDepthNoiseModel());
  }

  /* Load properties that apply for all noise model */
  /* Note that these min/max values may not be the same as near/far clip range
   * of the camera */
  if (_sdf->HasElement("depthNoiseMinDist")) {
    this->noise_model->min_depth =
        _sdf->GetElement("depthNoiseMinDist")->Get<float>();
  }

  if (_sdf->HasElement("depthNoiseMaxDist")) {
    this->noise_model->max_depth =
        _sdf->GetElement("depthNoiseMaxDist")->Get<float>();
  }

  load_connection_ = GazeboRosCameraUtils::OnLoad(boost::bind(&GazeboNoisyDepth::Advertise, this));

  GazeboRosCameraUtils::Load(_parent, _sdf);
}

void GazeboNoisyDepth::Advertise() {
  ros::AdvertiseOptions depth_image_ao =
      ros::AdvertiseOptions::create<sensor_msgs::Image>(
          this->depth_image_topic_name_, 1,
          boost::bind(&GazeboNoisyDepth::DepthImageConnect, this),
          boost::bind(&GazeboNoisyDepth::DepthImageDisconnect, this),
          ros::VoidPtr(), &this->camera_queue_);

  this->depth_image_pub_ = this->rosnode_->advertise(depth_image_ao);

  ros::AdvertiseOptions depth_image_camera_info_ao =
      ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(
          this->depth_image_camera_info_topic_name_, 1,
          boost::bind(&GazeboNoisyDepth::DepthInfoConnect, this),
          boost::bind(&GazeboNoisyDepth::DepthInfoDisconnect, this),
          ros::VoidPtr(), &this->camera_queue_);

  this->depth_image_camera_info_pub_ = this->rosnode_->advertise(depth_image_camera_info_ao);
}

void GazeboNoisyDepth::DepthImageConnect() {
  ++this->depth_image_connect_count_;
  this->parentSensor->SetActive(true);
}

void GazeboNoisyDepth::DepthImageDisconnect() {
  --this->depth_image_connect_count_;
}

void GazeboNoisyDepth::DepthInfoConnect() {
  ++this->depth_info_connect_count_;
}

void GazeboNoisyDepth::DepthInfoDisconnect() {
  --this->depth_info_connect_count_;
}

void GazeboNoisyDepth::OnNewDepthFrame(const float *_image, unsigned int _width,
                                       unsigned int _height,
                                       unsigned int _depth,
                                       const std::string &_format) {
  if (!this->initialized_ || this->height_ <= 0 || this->width_ <= 0) return;

  this->depth_sensor_update_time_ = this->parentSensor->LastMeasurementTime();

  // check if there are subscribers, if not disable parent, else process images..
  if (this->parentSensor->IsActive()) {
    if (this->depth_image_connect_count_ <= 0 && (*this->image_connect_count_) <= 0) {
      this->parentSensor->SetActive(false);
    } else {
      if (this->depth_image_connect_count_ > 0) this->FillDepthImage(_image);
    }
  }
  else {
    // if parent is disabled, but has subscribers, enable it.
    if ((*this->image_connect_count_) > 0){
      this->parentSensor->SetActive(true);
    }
  }
  PublishCameraInfo();
}

void GazeboNoisyDepth::OnNewImageFrame(const unsigned char *_image,
                                       unsigned int _width,
                                       unsigned int _height,
                                       unsigned int _depth,
                                       const std::string &_format) {
  if (!this->initialized_ || this->height_ <= 0 || this->width_ <= 0) return;

  this->sensor_update_time_ = this->parentSensor_->LastMeasurementTime();

  // check if there are subscribers, if not disable parent, else process images..
  if (this->parentSensor->IsActive()) {
    if (this->depth_image_connect_count_ <= 0 &&
        (*this->image_connect_count_) <= 0) {
      this->parentSensor->SetActive(false);
    } else {
      if ((*this->image_connect_count_) > 0) this->PutCameraData(_image);
    }
  } else {
    if ((*this->image_connect_count_) > 0) {
      // if parent is disabled, but has subscribers, enable it.
      this->parentSensor->SetActive(true);
    }
  }
}

void GazeboNoisyDepth::FillDepthImage(const float *_src) {
  this->lock_.lock();
  // copy data into image
  this->depth_image_msg_.header.frame_id = this->frame_name_;
  this->depth_image_msg_.header.stamp.sec = this->depth_sensor_update_time_.sec;
  this->depth_image_msg_.header.stamp.nsec = this->depth_sensor_update_time_.nsec;

  // copy from depth to depth image message
  if(FillDepthImageHelper(this->height, this->width,this->skip_, _src, &this->depth_image_msg_)){
    this->depth_image_pub_.publish(this->depth_image_msg_);
  }

  this->lock_.unlock();
}

bool GazeboNoisyDepth::FillDepthImageHelper(const uint32_t rows_arg,
                                            const uint32_t cols_arg,
                                            const uint32_t step_arg,
                                            const float *data_arg,
                                            sensor_msgs::Image *image_msg) {
  if(data_arg == nullptr){
    ROS_WARN_NAMED("NoisyDepth", "Invalid data array received - nullptr.");
    return false;
  }

  image_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  image_msg->height = rows_arg;
  image_msg->width = cols_arg;
  image_msg->step = sizeof(float) * cols_arg;
  image_msg->data.resize(rows_arg * cols_arg * sizeof(float));
  image_msg->is_bigendian = 0;

  float *dest = (float *)(&(image_msg->data[0]));
  memcpy(dest, data_arg, sizeof(float) * width * height);

  noise_model->ApplyNoise(width, height, dest);

  return true;
}

void GazeboNoisyDepth::PublishCameraInfo() {

  // first publish parent camera info (ir camera)
  GazeboRosCameraUtils::PublishCameraInfo();

  if (this->depth_info_connect_count_ > 0) {
    this->sensor_update_time_ = this->parentSensor_->LastMeasurementTime();
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time cur_time = this->world_->SimTime();
#else
    common::Time cur_time = this->world_->GetSimTime();
#endif

    if (this->sensor_update_time_ -
            this->last_depth_image_camera_info_update_time_ >= this->update_period_) {
      this->GazeboRosCameraUtils::PublishCameraInfo(this->depth_image_camera_info_pub_);
      this->last_depth_image_camera_info_update_time_ = this->sensor_update_time_;
    }
  }
}
}
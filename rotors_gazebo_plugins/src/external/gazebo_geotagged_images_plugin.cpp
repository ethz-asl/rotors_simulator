/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include "rotors_gazebo_plugins/external/gazebo_geotagged_images_plugin.h"

#include <math.h>
#include <string>
#include <iostream>

#include <boost/filesystem.hpp>
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>

#include "rotors_gazebo_plugins/common.h"

using namespace std;
using namespace gazebo;
using namespace cv;

GZ_REGISTER_SENSOR_PLUGIN(GeotaggedImagesPlugin)


GeotaggedImagesPlugin::GeotaggedImagesPlugin() :
    SensorPlugin(), width_(0), height_(0), depth_(0), imageCounter_(0) {}

GeotaggedImagesPlugin::~GeotaggedImagesPlugin()
{
  this->parentSensor_.reset();
  this->camera_.reset();
}

void GeotaggedImagesPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  if(kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (!sensor)
    gzerr << "Invalid sensor pointer.\n";

  this->parentSensor_ =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);

  if (!this->parentSensor_)
  {
    gzerr << "GeotaggedImagesPlugin requires a CameraSensor.\n";
  }

  this->camera_ = this->parentSensor_->Camera();

  if (!this->parentSensor_)
  {
    gzerr << "GeotaggedImagesPlugin not attached to a camera sensor\n";
    return;
  }
  scene_ = camera_->GetScene();
  lastImageTime_ = scene_->SimTime();

  this->width_ = this->camera_->ImageWidth();
  this->height_ = this->camera_->ImageHeight();
  this->depth_ = this->camera_->ImageDepth();
  this->format_ = this->camera_->ImageFormat();

  if (sdf->HasElement("robotNamespace")) {
    namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzwarn << "[gazebo_geotagging_images_camera_plugin] Please specify a robotNamespace.\n";
  }

  this->storeIntervalSec_ = 1;
  if (sdf->HasElement("interval")) {
	this->storeIntervalSec_ = sdf->GetElement("interval")->Get<float>();
  }

  destWidth_ = width_;
  if (sdf->HasElement("width")) {
	destWidth_ = sdf->GetElement("width")->Get<int>();
  }
  destHeight_ = height_;
  if (sdf->HasElement("height")) {
	destHeight_ = sdf->GetElement("height")->Get<int>();
  }


  //check if exiftool exists
  if (system("exiftool -ver &>/dev/null") != 0) {
    gzerr << "exiftool not found. geotagging_images plugin will be disabled" << endl;
    gzerr << "On Ubuntu, use 'sudo apt-get install libimage-exiftool-perl' to install" << endl;
    return;
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  this->parentSensor_->SetActive(true);

  this->newFrameConnection_ = this->camera_->ConnectNewImageFrame(
    boost::bind(&GeotaggedImagesPlugin::OnNewFrame, this, _1));

  // This topic is published to by gazebo_mavlink_interface.cpp
  /// \todo Should this be an absolute topic!?!
  gpsSub_ = node_handle_->Subscribe("~/gps_position", &GeotaggedImagesPlugin::OnNewGpsPosition, this);

  storageDir_ = "frames";
  boost::filesystem::remove_all(storageDir_); //clear existing images
  boost::filesystem::create_directory(storageDir_);
}

void GeotaggedImagesPlugin::OnNewGpsPosition(ConstVector3dPtr& v)
{
  lastGpsPosition_ = *v;
  //gzdbg << "got gps pos: "<<lastGpsPosition_.x()<<", "<<lastGpsPosition.y()<<endl;
}

void GeotaggedImagesPlugin::OnNewFrame(const unsigned char * image)
{

  image = this->camera_->ImageData(0);

  common::Time currentTime = scene_->SimTime();
  if (currentTime.Double() - lastImageTime_.Double() < storeIntervalSec_) {
    return;
  }

  Mat frame = Mat(height_, width_, CV_8UC3);
  Mat frameBGR = Mat(height_, width_, CV_8UC3);
  frame.data = (uchar*)image; //frame has not the right color format yet -> convert
  cvtColor(frame, frameBGR, CV_RGB2BGR);

  char file_name[256];
  snprintf(file_name, sizeof(file_name), "%s/DSC%05i.jpg", storageDir_.c_str(), imageCounter_);

  if (destWidth_ != width_ || destHeight_ != height_) {
    Mat frameResized;
    cv::Size size(destWidth_, destHeight_);
    cv::resize(frameBGR, frameResized, size);
    imwrite(file_name, frameResized);
  } else {
    imwrite(file_name, frameBGR);
  }

  char gps_tag_command[1024];
  double lat = lastGpsPosition_.x();
  char north_south = 'N', east_west = 'E';
  double lon = lastGpsPosition_.y();
  if (lat < 0.) {
    lat = -lat;
    north_south = 'S';
  }
  if (lon < 0.) {
    lon = -lon;
    east_west = 'W';
  }
  snprintf(gps_tag_command, sizeof(gps_tag_command),
    "exiftool -gpslatituderef=%c -gpslongituderef=%c -gpsaltituderef=above -gpslatitude=%.9lf -gpslongitude=%.9lf"
//    " -gpsdatetime=now -gpsmapdatum=WGS-84"
    " -datetimeoriginal=now -gpsdop=0.8"
    " -gpsmeasuremode=3-d -gpssatellites=13 -gpsaltitude=%.3lf -overwrite_original %s &>/dev/null",
    north_south, east_west, lat, lon, lastGpsPosition_.z(), file_name);

  system(gps_tag_command);

  ++imageCounter_;
  lastImageTime_ = currentTime;

}

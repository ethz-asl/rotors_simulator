/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2017, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <velodyne_gazebo_plugins/GazeboRosVelodyneLaser.h>

#include <algorithm>
#include <assert.h>

#include <gazebo/physics/World.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/Node.hh>

#include <sensor_msgs/PointCloud2.h>

#include <tf/tf.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosVelodyneLaser)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosVelodyneLaser::GazeboRosVelodyneLaser() : nh_(NULL), gaussian_noise_(0), min_range_(0), max_range_(0)
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosVelodyneLaser::~GazeboRosVelodyneLaser()
{
  ////////////////////////////////////////////////////////////////////////////////
  // Finalize the controller / Custom Callback Queue
  laser_queue_.clear();
  laser_queue_.disable();
  if (nh_) {
    nh_->shutdown();
    delete nh_;
    nh_ = NULL;
  }
  callback_laser_queue_thread_.join();
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosVelodyneLaser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Load plugin
  RayPlugin::Load(_parent, _sdf);

  // Initialize Gazebo node
  gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  gazebo_node_->Init();

  // Get the parent ray sensor
#if GAZEBO_MAJOR_VERSION >= 7
  parent_ray_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);
#else
  parent_ray_sensor_ = boost::dynamic_pointer_cast<sensors::RaySensor>(_parent);
#endif
  if (!parent_ray_sensor_) {
    gzthrow("GazeboRosVelodyneLaser controller requires a Ray Sensor as its parent");
  }

  robot_namespace_ = "/";
  if (_sdf->HasElement("robotNamespace")) {
    robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  }

  if (!_sdf->HasElement("frameName")) {
    ROS_INFO("Velodyne laser plugin missing <frameName>, defaults to /world");
    frame_name_ = "/world";
  } else {
    frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();
  }

  if (!_sdf->HasElement("min_range")) {
    ROS_INFO("Velodyne laser plugin missing <min_range>, defaults to 0");
    min_range_ = 0;
  } else {
    min_range_ = _sdf->GetElement("min_range")->Get<double>();
  }

  if (!_sdf->HasElement("max_range")) {
    ROS_INFO("Velodyne laser plugin missing <max_range>, defaults to infinity");
    max_range_ = INFINITY;
  } else {
    max_range_ = _sdf->GetElement("max_range")->Get<double>();
  }

  if (!_sdf->HasElement("topicName")) {
    ROS_INFO("Velodyne laser plugin missing <topicName>, defaults to /points");
    topic_name_ = "/points";
  } else {
    topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
  }

  if (!_sdf->HasElement("gaussianNoise")) {
    ROS_INFO("Velodyne laser plugin missing <gaussianNoise>, defaults to 0.0");
    gaussian_noise_ = 0;
  } else {
    gaussian_noise_ = _sdf->GetElement("gaussianNoise")->Get<double>();
  }

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Create node handle
  nh_ = new ros::NodeHandle(robot_namespace_);

  // Resolve tf prefix
  std::string prefix;
  nh_->getParam(std::string("tf_prefix"), prefix);
  if (robot_namespace_ == "/") {
    frame_name_ = tf::resolve(prefix, frame_name_);
  } else {
    frame_name_ = tf::resolve(robot_namespace_, frame_name_);
  }

  // Advertise publisher with a custom callback queue
  if (topic_name_ != "") {
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
        topic_name_, 1,
        boost::bind(&GazeboRosVelodyneLaser::ConnectCb, this),
        boost::bind(&GazeboRosVelodyneLaser::ConnectCb, this),
        ros::VoidPtr(), &laser_queue_);
    pub_ = nh_->advertise(ao);
  }

  // Sensor generation off by default
  parent_ray_sensor_->SetActive(false);

  // Start custom queue for laser
  callback_laser_queue_thread_ = boost::thread( boost::bind( &GazeboRosVelodyneLaser::laserQueueThread,this ) );

#if GAZEBO_MAJOR_VERSION >= 7
  ROS_INFO("Velodyne laser plugin ready, %i lasers", parent_ray_sensor_->VerticalRangeCount());
#else
  ROS_INFO("Velodyne laser plugin ready, %i lasers", parent_ray_sensor_->GetVerticalRangeCount());
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Subscribe on-demand
void GazeboRosVelodyneLaser::ConnectCb()
{
  boost::lock_guard<boost::mutex> lock(lock_);
  if (pub_.getNumSubscribers()) {
    if (!sub_) {
#if GAZEBO_MAJOR_VERSION >= 7
      sub_ = gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(), &GazeboRosVelodyneLaser::OnScan, this);
#else
      sub_ = gazebo_node_->Subscribe(this->parent_ray_sensor_->GetTopic(), &GazeboRosVelodyneLaser::OnScan, this);
#endif
    }
    parent_ray_sensor_->SetActive(true);
  } else {
#if GAZEBO_MAJOR_VERSION >= 7
    if (sub_) {
      sub_->Unsubscribe();
      sub_.reset();
    }
#endif
    parent_ray_sensor_->SetActive(false);
  }
}

void GazeboRosVelodyneLaser::OnScan(ConstLaserScanStampedPtr& _msg)
{
#if GAZEBO_MAJOR_VERSION >= 7
  const math::Angle maxAngle = parent_ray_sensor_->AngleMax();
  const math::Angle minAngle = parent_ray_sensor_->AngleMin();

  const double maxRange = parent_ray_sensor_->RangeMax();
  const double minRange = parent_ray_sensor_->RangeMin();

  const int rayCount = parent_ray_sensor_->RayCount();
  const int rangeCount = parent_ray_sensor_->RangeCount();
  assert(rayCount == rangeCount);

  const int verticalRayCount = parent_ray_sensor_->VerticalRayCount();
  const int verticalRangeCount = parent_ray_sensor_->VerticalRangeCount();
  assert(verticalRayCount == verticalRangeCount);

  const math::Angle verticalMaxAngle = parent_ray_sensor_->VerticalAngleMax();
  const math::Angle verticalMinAngle = parent_ray_sensor_->VerticalAngleMin();
#else
  math::Angle maxAngle = parent_ray_sensor_->GetAngleMax();
  math::Angle minAngle = parent_ray_sensor_->GetAngleMin();

  const double maxRange = parent_ray_sensor_->GetRangeMax();
  const double minRange = parent_ray_sensor_->GetRangeMin();

  const int rayCount = parent_ray_sensor_->GetRayCount();
  const int rangeCount = parent_ray_sensor_->GetRangeCount();
  assert(rayCount == rangeCount);

  const int verticalRayCount = parent_ray_sensor_->GetVerticalRayCount();
  const int verticalRangeCount = parent_ray_sensor_->GetVerticalRangeCount();
  assert(verticalRayCount == verticalRangeCount);

  const math::Angle verticalMaxAngle = parent_ray_sensor_->GetVerticalAngleMax();
  const math::Angle verticalMinAngle = parent_ray_sensor_->GetVerticalAngleMin();
#endif

  const double yDiff = maxAngle.Radian() - minAngle.Radian();
  const double pDiff = verticalMaxAngle.Radian() - verticalMinAngle.Radian();

  const double MIN_RANGE = std::max(min_range_, minRange);
  const double MAX_RANGE = std::min(max_range_, maxRange - minRange - 0.01);

  // Populate message fields
  const uint32_t POINT_STEP = 32;
  sensor_msgs::PointCloud2 msg;
  msg.header.frame_id = frame_name_;
  msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
  msg.fields.resize(5);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.fields[3].name = "intensity";
  msg.fields[3].offset = 16;
  msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[3].count = 1;
  msg.fields[4].name = "ring";
  msg.fields[4].offset = 20;
  msg.fields[4].datatype = sensor_msgs::PointField::UINT16;
  msg.fields[4].count = 1;
  msg.data.resize(verticalRangeCount * rangeCount * POINT_STEP);

  int i, j;
  uint8_t *ptr = msg.data.data();
  for (j = 0; j < verticalRangeCount; j++) {
    for (i = 0; i < rangeCount; i++) {

      // Range and noise
      double r = std::min(_msg->scan().ranges(i + j * rangeCount), maxRange-minRange);
      if (gaussian_noise_ != 0.0) {
        r += gaussianKernel(0,gaussian_noise_);
      }

      // Intensity
      double intensity = _msg->scan().intensities(i + j * rangeCount);

      // Get angles of ray to get xyz for point
      double yAngle = i * yDiff / (rayCount -1) + minAngle.Radian();
      double pAngle = j * pDiff / (verticalRayCount -1) + verticalMinAngle.Radian();

      // pAngle is rotated by yAngle:
      if ((MIN_RANGE < r) && (r < MAX_RANGE)) {
        *((float*)(ptr + 0)) = r * cos(pAngle) * cos(yAngle);
        *((float*)(ptr + 4)) = r * cos(pAngle) * sin(yAngle);
#if GAZEBO_MAJOR_VERSION > 2
        *((float*)(ptr + 8)) = r * sin(pAngle);
#else
        *((float*)(ptr + 8)) = -r * sin(pAngle);
#endif
        *((float*)(ptr + 16)) = intensity;
#if GAZEBO_MAJOR_VERSION > 2
        *((uint16_t*)(ptr + 20)) = j; // ring
#else
        *((uint16_t*)(ptr + 20)) = verticalRangeCount - 1 - j; // ring
#endif
        ptr += POINT_STEP;
      }
    }
  }

  // Populate message with number of valid points
  msg.point_step = POINT_STEP;
  msg.row_step = ptr - msg.data.data();
  msg.height = 1;
  msg.width = msg.row_step / POINT_STEP;
  msg.is_bigendian = false;
  msg.is_dense = true;
  msg.data.resize(msg.row_step); // Shrink to actual size

  // Publish output
  pub_.publish(msg);
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// Custom callback queue thread
void GazeboRosVelodyneLaser::laserQueueThread()
{
  while (nh_->ok()) {
    laser_queue_.callAvailable(ros::WallDuration(0.01));
  }
}

} // namespace gazebo


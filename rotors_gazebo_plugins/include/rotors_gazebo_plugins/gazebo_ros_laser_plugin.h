/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2018, Dataspeed Inc.
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

#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_ROS_LASER_PLUGIN_H_
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_ROS_LASER_PLUGIN_H_

// Use the same source code for CPU and GPU plugins
#ifndef GAZEBO_GPU_RAY
#define GAZEBO_GPU_RAY 0
#endif

// Custom Callback Queue
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/Node.hh>
#include <sdf/Param.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/sensors/SensorTypes.hh>
#if GAZEBO_GPU_RAY
#include <gazebo/plugins/GpuRayPlugin.hh>
#else
#include <gazebo/plugins/RayPlugin.hh>
#endif

#include <boost/algorithm/string/trim.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/lock_guard.hpp>
#include <boost/thread/mutex.hpp>

#if GAZEBO_GPU_RAY
#define RosVelodyneLaser GazeboRosVelodyneGpuLaser
#define RayPlugin GpuRayPlugin
#define RaySensorPtr GpuRaySensorPtr
#endif

namespace gazebo {

#define M2_PI 2.0 * M_PI

/// \brief A plugin to read data from a laser sensor one vertical line at a time
class ROSLaserPlugin : public RayPlugin {
  /// \brief Constructor
  /// \param parent The parent entity, must be a Model or a Sensor
 public:
  ROSLaserPlugin();

  /// \brief Destructor
  ~ROSLaserPlugin();

  /// \brief Load the plugin
  /// \param take in SDF root element
  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

 private:
  /// \brief Subscribe on-demand
  void ConnectCb();

  /// \brief Normalizes an angle into the [-pi,pi) domain
  static double ConstrainAngle(double angle_rad);

  /// \brief Gaussian noise generator
  static double GaussianKernel(double mu, double sigma);

  /// \brief Custom callback queue thread
  void LaserQueueThread();

  /// \brief Called when a new scan is provided by the simulation
  void OnScan(const ConstLaserScanStampedPtr &_msg);

  /// \brief The parent ray sensor
  sensors::RaySensorPtr parent_ray_sensor_;

  /// \brief Pointer to ROS node
  ros::NodeHandle *nh_;

  /// \brief ROS publisher
  ros::Publisher pub_;

  /// \brief topic name
  std::string topic_name_;

  /// \brief frame transform name, should match link name
  std::string frame_name_;

  /// \brief joint the laser is attached to
  physics::JointPtr joint_;

  /// \brief Minimum range to publish
  double min_range_;

  /// \brief Maximum range to publish
  double max_range_;

  /// \brief Gaussian noise
  double gaussian_noise_;

  /// \brief A mutex to lock access
  boost::mutex lock_;

  /// \brief For setting ROS name space
  std::string robot_namespace_;

  /// \brief Custom Callback Queue
  ros::CallbackQueue laser_queue_;

  /// \brief Custom Callback Queue
  boost::thread callback_laser_queue_thread_;

  /// \brief A pointer to the active gazebo node
  gazebo::transport::NodePtr gazebo_node_;

  /// \brief Subscribe to gazebo laserscan
  gazebo::transport::SubscriberPtr sub_;
};

}  // namespace gazebo

#endif  // ROTORS_GAZEBO_PLUGINS_GAZEBO_ROS_LASER_PLUGIN_H_

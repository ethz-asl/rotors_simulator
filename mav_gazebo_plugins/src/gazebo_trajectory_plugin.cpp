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

#include <mav_gazebo_plugins/common.h>
#include <mav_gazebo_plugins/gazebo_trajectory_plugin.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <chrono>

#include <mav_planning_utils/conversions.h>
#include <mav_planning_utils/motion4D.h>
#include <mav_planning_utils/mav_state.h>

#include <planning_msgs/WaypointType.h>

#include <std_srvs/Empty.h>

namespace gazebo {

GazeboCameraTrajectoryPlugin::GazeboCameraTrajectoryPlugin()
    : ModelPlugin(),
      node_handle_(0),
      gazebo_seq_(0),
      pose_seq_(0),
      follow_path_(false),
      path_time_(0)
{
}

GazeboCameraTrajectoryPlugin::~GazeboCameraTrajectoryPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}


// void GazeboCameraTrajectoryPlugin::InitializeParams() {};
// void GazeboCameraTrajectoryPlugin::Publish() {};

void GazeboCameraTrajectoryPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  // default params
  namespace_.clear();
  imu_topic_ = "imu";

  gazebo_seq_ = 0;
  pose_seq_ = 0;

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_pose_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_camera_trajectory_plugin] Please specify a linkName.\n";
  // Get the pointer to the link
  link_ = this->model_->GetLink(link_name_);

  getSdfParam<std::string>(_sdf, "imuTopic", imu_topic_, "imu");
  getSdfParam<std::string>(_sdf, "pathSegmentsTopic", path_segments_topic_, "/path_segments");

  frame_id_ = namespace_ + "/" + link_name_;
  last_time_ = world_->GetSimTime();

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboCameraTrajectoryPlugin::OnUpdate, this, _1));

  imu_pub_ = node_handle_->advertise<sensor_msgs::Imu>(imu_topic_, 10);
  path_sub_ = node_handle_->subscribe(path_segments_topic_, 10, &GazeboCameraTrajectoryPlugin::pathCb, this);
}

// Called by the world update start event
void GazeboCameraTrajectoryPlugin::OnUpdate(const common::UpdateInfo& _info) {
  common::Time current_time  = world_->GetSimTime();
  double time_diff = (current_time - last_time_).Double();
  last_time_ = current_time;
  double t = current_time.Double();

  // Get linear/angular velocities and accelerations from paths at current timestep
  Eigen::Matrix<double, 4, 1> acceleration;
  Eigen::Matrix<double, 4, 1> velocity;
  Eigen::Matrix<double, 3, 1> p;

  if (t >= path_received_time_.Double() && t <= path_received_time_.Double() + path_time_) {
    t -= path_received_time_.Double();

    mav_planning_utils::path_planning::samplePath(p, sx_, t);
    velocity[0] = p[1];
    acceleration[0] = p[2];
    mav_planning_utils::path_planning::samplePath(p, sy_, t);
    velocity[1] = p[1];
    acceleration[1] = p[2];
    mav_planning_utils::path_planning::samplePath(p, sz_, t);
    velocity[2] = p[1];
    acceleration[2] = p[2];
    mav_planning_utils::path_planning::samplePath(p, syaw_, t);
    velocity[3] = p[1];
    acceleration[3] = p[2];

    // Set linear/angular velocities and accelerations
    model_->SetLinearAccel(math::Vector3(acceleration[0], acceleration[1], acceleration[2]));
    model_->SetLinearVel(math::Vector3(velocity[0], velocity[1], velocity[2]));
    model_->SetAngularVel(math::Vector3(0, 0, velocity[3]));
    model_->SetAngularAccel(math::Vector3(0, 0, velocity[3]));
  }
  else {
    model_->SetLinearAccel(math::Vector3(0, 0, 0));
    model_->SetLinearVel(math::Vector3(0, 0, 0));
    model_->SetAngularVel(math::Vector3(0, 0, 0));
    model_->SetAngularAccel(math::Vector3(0, 0, 0));
  }

  ++gazebo_seq_;
}

void GazeboCameraTrajectoryPlugin::pathCb(const planning_msgs::WayPointArrayConstPtr& msg) {
  const planning_msgs::WayPoint::_type_type type = msg->waypoints[0].type;

  if (type == planning_msgs::WaypointType::POLYNOMIAL_12)
  {
    if (!createSegments(msg->waypoints))
      return;
  }
  else
  {
    ROS_WARN("[polynomial sampling]: waypoints must have type POLYNOMIAL_12, got %d instead ", type);
    return;
  }

  if (!follow_path_) {
    std_srvs::Empty srv;
    bool ret = ros::service::call("/gazebo/unpause_physics", srv);
    follow_path_ = true;
  }

  path_received_time_ = world_->GetSimTime();
}

bool GazeboCameraTrajectoryPlugin::createSegments(const planning_msgs::WayPointArray::_waypoints_type& wpts) {
  if (wpts.empty())
  {
    ROS_WARN("[polynomial sampling]: got 0 path segments");
    return false;
  }

  const size_t n_segments = wpts.size();

  typename mav_planning_utils::path_planning::Segment<N_>::Vector sx(n_segments);
  typename mav_planning_utils::path_planning::Segment<N_>::Vector sy(n_segments);
  typename mav_planning_utils::path_planning::Segment<N_>::Vector sz(n_segments);
  typename mav_planning_utils::path_planning::Segment<N_>::Vector syaw(n_segments);

  double path_time = 0;
  for (size_t s = 0; s < n_segments; ++s)
  {
    const planning_msgs::WayPoint& wps = wpts[s];

    bool ok = true;
    ok = ok && (wps.x.size() == N_);
    ok = ok && (wps.y.size() == N_);
    ok = ok && (wps.z.size() == N_);
    ok = ok && (wps.yaw.size() == N_);

    if (!ok)
    {
      ROS_WARN("[polynomial sampling]: waypoints have different numbers of coefficients: %lu %lu %lu %lu",
               wps.x.size(), wps.y.size(), wps.z.size(), wps.yaw.size());
      return false;
    }

    sx[s].p.setCoefficients(Eigen::Map<const typename mav_planning_utils::Polynomial<N_>::VectorR>(&wps.x[0]));
    sy[s].p.setCoefficients(Eigen::Map<const typename mav_planning_utils::Polynomial<N_>::VectorR>(&wps.y[0]));
    sz[s].p.setCoefficients(Eigen::Map<const typename mav_planning_utils::Polynomial<N_>::VectorR>(&wps.z[0]));
    syaw[s].p.setCoefficients(Eigen::Map<const typename mav_planning_utils::Polynomial<N_>::VectorR>(&wps.yaw[0]));

    path_time += wps.time;
    sx[s].t = sy[s].t = sz[s].t = syaw[s].t = wps.time;
  }
  path_time_ = path_time;

  sx_ = sx;
  sy_ = sy;
  sz_ = sz;
  syaw_ = syaw;

  return true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboCameraTrajectoryPlugin);
}

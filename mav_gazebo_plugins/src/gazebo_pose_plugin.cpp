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


#include <mav_gazebo_plugins/gazebo_pose_plugin.h>
#include <mav_gazebo_plugins/common.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <chrono>

namespace gazebo {

/// Computes a quaternion from the 3-element small angle approximation theta.
template<class Derived>
Eigen::Quaternion<typename Derived::Scalar> QuaternionFromSmallAngle(const Eigen::MatrixBase<Derived> & theta) {
  typedef typename Derived::Scalar Scalar;
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  const Scalar q_squared = theta.squaredNorm() / 4.0;

  if (q_squared < 1) {
    return Eigen::Quaternion<Scalar>(sqrt(1 - q_squared), theta[0] * 0.5, theta[1] * 0.5, theta[2] * 0.5);
  }
  else {
    const Scalar w = 1.0 / sqrt(1 + q_squared);
    const Scalar f = w * 0.5;
    return Eigen::Quaternion<Scalar>(w, theta[0] * f, theta[1] * f, theta[2] * f);
  }
}

template<class In, class Out>
void copyPosition(const In& in, Out& out) {
  out.x = in.x;
  out.y = in.y;
  out.z = in.z;
}

GazeboPosePlugin::GazeboPosePlugin()
    : ModelPlugin(),
      node_handle_(0),
      gen_(rd_()),
      measurement_delay_(0),
      measurement_divisor_(1),
      unknown_delay_(0),
      gazebo_seq_(0),
      pose_seq_(0),
      covariance_image_scale_(1) {
}

GazeboPosePlugin::~GazeboPosePlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}
;

// void GazeboPosePlugin::InitializeParams() {};
// void GazeboPosePlugin::Publish() {};

void GazeboPosePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  // world_ = physics::get_world(model_->world.name);
  world_ = model_->GetWorld();

  // default params
  namespace_.clear();
  pose_topic_ = "pose";
  measurement_divisor_ = 1;
  measurement_delay_ = 0;
  unknown_delay_ = 0;

  sdf::Vector3 noise_normal_p;
  sdf::Vector3 noise_normal_q;
  sdf::Vector3 noise_uniform_p;
  sdf::Vector3 noise_uniform_q;
  const sdf::Vector3 zeros3(0.0, 0.0, 0.0);

  gazebo_seq_ = 0;
  pose_seq_ = 0;

  pose_queue_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_pose_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_pose_plugin] Please specify a linkName.\n";
  // Get the pointer to the link
  link_ = this->model_->GetLink(link_name_);

  if (_sdf->HasElement("poseTopic"))
    pose_topic_ = _sdf->GetElement("poseTopic")->Get<std::string>();

  if (_sdf->HasElement("measurementDelay"))
    measurement_delay_ = _sdf->GetElement("measurementDelay")->Get<int>();

  if (_sdf->HasElement("measurementDivisor"))
    measurement_divisor_ = _sdf->GetElement("measurementDivisor")->Get<double>();

  getSdfParam(_sdf, "noiseNormalP", noise_normal_p, zeros3);
  getSdfParam(_sdf, "noiseNormalQ", noise_normal_q, zeros3);
  getSdfParam(_sdf, "noiseUniformP", noise_uniform_p, zeros3);
  getSdfParam(_sdf, "noiseUniformQ", noise_uniform_q, zeros3);

  getSdfParam(_sdf, "unknownDelay", unknown_delay_, 0.0);

  if (_sdf->HasElement("covarianceImage")) {
    std::string image_name = _sdf->GetElement("covarianceImage")->Get<std::string>();
    covariance_image_ = cv::imread(image_name, CV_LOAD_IMAGE_GRAYSCALE);
    if (covariance_image_.data == NULL)
      gzerr << "loading covariance image " << image_name << " failed" << std::endl;
    else
      gzlog << "loading covariance image " << image_name << " successful" << std::endl;
  }

  getSdfParam(_sdf, "covarianceImageScale", covariance_image_scale_, 1.0);

  if (_sdf->HasElement("randomEngineSeed")) {
    gen_.seed(_sdf->GetElement("randomEngineSeed")->Get<unsigned int>());
  }
  else {
    gen_.seed(std::chrono::system_clock::now().time_since_epoch().count());
  }

  pos_n_[0] = NormalDistribution(0, noise_normal_p.x);
  pos_n_[1] = NormalDistribution(0, noise_normal_p.y);
  pos_n_[2] = NormalDistribution(0, noise_normal_p.z);

  att_n_[0] = NormalDistribution(0, noise_normal_q.x);
  att_n_[1] = NormalDistribution(0, noise_normal_q.y);
  att_n_[2] = NormalDistribution(0, noise_normal_q.z);

  pos_u_[0] = UniformDistribution(-noise_uniform_p.x, noise_uniform_p.x);
  pos_u_[1] = UniformDistribution(-noise_uniform_p.y, noise_uniform_p.y);
  pos_u_[2] = UniformDistribution(-noise_uniform_p.z, noise_uniform_p.z);

  att_u_[0] = UniformDistribution(-noise_uniform_q.x, noise_uniform_q.x);
  att_u_[1] = UniformDistribution(-noise_uniform_q.y, noise_uniform_q.y);
  att_u_[2] = UniformDistribution(-noise_uniform_q.z, noise_uniform_q.z);

  // Fill in covariance. We omit uniform noise here, to make it more exciting for the challengers :).
  Eigen::Map<Eigen::Matrix<double, 6, 6> > cov(covariance_matrix_.data());
  Eigen::Matrix<double, 6, 1> covd;

  covd << noise_normal_p.x * noise_normal_p.x, noise_normal_p.y * noise_normal_p.y, noise_normal_p.z * noise_normal_p.z, noise_normal_q
      .x * noise_normal_q.x, noise_normal_q.y * noise_normal_q.y, noise_normal_q.z * noise_normal_q.z;
  cov = covd.asDiagonal();

  frame_id_ = namespace_ + "/" + link_name_;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboPosePlugin::OnUpdate, this, _1));

  pose_pub_ = node_handle_->advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic_, 10);
}

// Called by the world update start event
void GazeboPosePlugin::OnUpdate(const common::UpdateInfo& _info) {
  math::Pose gazebo_pose = link_->GetWorldCoGPose();
  bool publish_pose = true;

  // first, determine whether we should publish a pose
  if (covariance_image_.data != NULL) {
    // we have an image

    // Image is always centered around the origin:
    int width = covariance_image_.cols;
    int height = covariance_image_.rows;
    int x = static_cast<int>(std::floor(gazebo_pose.pos.x / covariance_image_scale_)) + width / 2;
    int y = static_cast<int>(std::floor(gazebo_pose.pos.y / covariance_image_scale_)) + height / 2;

    if (x >= 0 && x < width && y >= 0 && y < height) {
      uint8_t val = covariance_image_.at<uint8_t>(y, x);
      if (val == 0)
        publish_pose = false;
      // TODO: covariance scaling, according to the intensity values could be implemented here.
    }
  }

  if (gazebo_seq_ % measurement_divisor_ == 0) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id_;
    pose.header.seq = pose_seq_++;
    pose.header.stamp.sec = (world_->GetSimTime()).sec + ros::Duration(unknown_delay_).sec;
    pose.header.stamp.nsec = (world_->GetSimTime()).nsec + ros::Duration(unknown_delay_).nsec;

    copyPosition(gazebo_pose.pos, pose.pose.position);
    pose.pose.orientation.w = gazebo_pose.rot.w;
    pose.pose.orientation.x = gazebo_pose.rot.x;
    pose.pose.orientation.y = gazebo_pose.rot.y;
    pose.pose.orientation.z = gazebo_pose.rot.z;

    if (publish_pose)
      pose_queue_.push_back(std::make_pair(gazebo_seq_ + measurement_delay_, pose));
  }

  // Is it time to publish the front element ?
  if (gazebo_seq_ == pose_queue_.front().first) {
    geometry_msgs::PoseWithCovarianceStampedPtr pose(new geometry_msgs::PoseWithCovarianceStamped);

    pose->header = pose_queue_.front().second.header;
    pose->pose.pose = pose_queue_.front().second.pose;
    pose_queue_.pop_front();

    //calculate position distortions
    Eigen::Vector3d pos_n;
    pos_n << pos_n_[0](gen_) + pos_u_[0](gen_), pos_n_[1](gen_) + pos_u_[1](gen_), pos_n_[2](gen_) + pos_u_[2](gen_);
    geometry_msgs::Point& p = pose->pose.pose.position;
    p.x += pos_n[0];
    p.y += pos_n[1];
    p.z += pos_n[2];

    //calculate attitude distortions
    Eigen::Vector3d theta;
    theta << att_n_[0](gen_) + att_u_[0](gen_), att_n_[1](gen_) + att_u_[1](gen_), att_n_[2](gen_) + att_u_[2](gen_);
    Eigen::Quaterniond q_n = QuaternionFromSmallAngle(theta);
    q_n.normalize();
    geometry_msgs::Quaternion& q = pose->pose.pose.orientation;
    Eigen::Quaterniond _q(q.w, q.x, q.y, q.z);
    _q = _q * q_n;
    q.w = _q.w();
    q.x = _q.x();
    q.y = _q.y();
    q.z = _q.z();

    pose->pose.covariance = covariance_matrix_;

    pose_pub_.publish(pose);
//    std::cout << "published pose with timestamp " << pose->header.stamp << "at time t" << world_->GetSimTime().Double()
//        << "delay should be " << measurement_delay_ << "sim cycles" << std::endl;
  }

  ++gazebo_seq_;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboPosePlugin);
}

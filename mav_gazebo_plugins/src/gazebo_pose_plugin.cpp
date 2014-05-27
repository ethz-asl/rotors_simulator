//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>
// All rights reserved.
//
// TODO(ff): Enter some license
//==============================================================================
#include <mav_gazebo_plugins/gazebo_pose_plugin.h>
#include <iostream>

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
  } else {
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

  GazeboPosePlugin::GazeboPosePlugin() : 
    ModelPlugin(), node_handle_(0), gen_(rd_()) {}

  GazeboPosePlugin::~GazeboPosePlugin() {
    event::Events::DisconnectWorldUpdateBegin(updateConnection_);
    if (node_handle_) {
      node_handle_->shutdown();
      delete node_handle_;
    }
  };

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
    frame_id_ = "/pose_sensor";
    measurement_divisor_ = 1;
    measurement_delay_ = 0;
    noise_normal_p_ = 0.01;
    noise_normal_q_ = 0.02;
    noise_uniform_p_ = 0;
    noise_uniform_q_ = 0;

    gazebo_seq_ = 0;
    pose_seq_ = 0;

    pose_queue_.clear();

    if (_sdf->HasElement("robotNamespace"))
      namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
      gzerr << "[gazebo_pose_plugin] Please specify a robotNamespace.\n";
    node_handle_ = new ros::NodeHandle(namespace_);

    if (_sdf->HasElement("frameId"))
    frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();

    if (_sdf->HasElement("linkName"))
      link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
    else
      gzerr << "[gazebo_pose_plugin] Please specify a linkName.\n";
    // Get the pointer to the link
    link_ = this->model_->GetLink(link_name_);

    if (_sdf->HasElement("poseTopic"))
      pose_topic_ = _sdf->GetElement("poseTopic")->Get<std::string>();

    if (_sdf->HasElement("measurementRate"))
      measurement_rate_ = _sdf->GetElement("measurementRate")->Get<double>();

    if (_sdf->HasElement("measurementDelay"))
      measurement_delay_ = _sdf->GetElement("measurementDelay")->Get<int>();

    if (_sdf->HasElement("measurementDivisor"))
      measurement_divisor_ = _sdf->GetElement(
        "measurementDivisor")->Get<double>();

    if (_sdf->HasElement("noiseNormalQ")) {
      noise_normal_q_ = _sdf->GetElement(
        "noiseNormalQ")->Get<double>();
    }

    if (_sdf->HasElement("noiseNormalP")) {
      noise_normal_p_ = _sdf->GetElement(
        "noiseNormalP")->Get<double>();
    }

    if (_sdf->HasElement("noiseUniformQ")) {
      noise_uniform_q_ = _sdf->GetElement(
        "noiseUniformQ")->Get<double>();
    }

    if (_sdf->HasElement("noiseUniformP")) {
      noise_uniform_p_ = _sdf->GetElement(
        "noiseUniformP")->Get<double>();
    }

    for (int i = 0; i < 3; ++i) {
      pos_n_[i] = NormalDistribution(0, noise_normal_p_);
      att_n_[i] = NormalDistribution(0, noise_normal_q_);
      pos_u_[i] = UniformDistribution(-noise_uniform_p_, noise_uniform_p_);
      att_u_[i] = UniformDistribution(-noise_uniform_q_, noise_uniform_q_);
    }

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GazeboPosePlugin::OnUpdate, this, _1));

    pose_pub_ = node_handle_->advertise<geometry_msgs::PoseWithCovarianceStamped>(
      pose_topic_, 10);
  }

  // Called by the world update start event
  void GazeboPosePlugin::OnUpdate(const common::UpdateInfo& _info) {
    if (gazebo_seq_ % measurement_divisor_ == 0) {
      geometry_msgs::PoseStamped pose;
      math::Pose gazebo_pose = link_->GetWorldCoGPose();
      pose.header.frame_id = frame_id_;
      pose.header.seq = pose_seq_++;
      pose.header.stamp.sec = (world_->GetSimTime()).sec;
      pose.header.stamp.nsec = (world_->GetSimTime()).nsec;

      copyPosition(gazebo_pose.pos, pose.pose.position);
      pose.pose.orientation.w = gazebo_pose.rot.w;
      pose.pose.orientation.x = gazebo_pose.rot.x;
      pose.pose.orientation.y = gazebo_pose.rot.y;
      pose.pose.orientation.z = gazebo_pose.rot.z;

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

      // Fill in covariance. We omit uniform noise here, to make it more exciting for the challengers :).
      double cp = noise_normal_p_ * noise_normal_p_;
      double cq = noise_normal_q_ * noise_normal_q_;
      Eigen::Map<Eigen::Matrix<double, 6, 6>> cov(pose->pose.covariance.data());
      Eigen::Matrix<double, 6, 1> covd;
      covd << cp, cp, cp, cq, cq, cq;
      cov = covd.asDiagonal();

    pose_pub_.publish(pose);
//    std::cout << "published pose with timestamp " << pose->header.stamp << "at time t" << world_->GetSimTime().Double()
//        << "delay should be " << measurement_delay_ << "sim cycles" << std::endl;
    }

    gazebo_seq_ ++;
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboPosePlugin);
}

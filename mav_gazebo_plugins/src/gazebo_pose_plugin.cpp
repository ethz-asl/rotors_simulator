//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>
// All rights reserved.
//
// TODO(ff): Enter some license
//==============================================================================
#include <mav_gazebo_plugins/gazebo_pose_plugin.h>

namespace gazebo {

  template<class In, class Out>
  void copyPosition(const In& in, Out& out) {
    out.x = in.x;
    out.y = in.y;
    out.z = in.z;
  }

  GazeboPosePlugin::GazeboPosePlugin() : 
    ModelPlugin(), node_handle_(0) {}

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
    noise_normal_p_ = 0.01;
    noise_normal_q_ = 0.02;

    gazebo_seq_ = 0;
    pose_seq_ = 0;

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
      measurement_delay_ = _sdf->GetElement("measurementDelay")->Get<double>();

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

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GazeboPosePlugin::OnUpdate, this, _1));

    pose_pub_ = node_handle_->advertise<geometry_msgs::PoseWithCovarianceStamped>(
      pose_topic_, 10);
  }

  // Called by the world update start event
  void GazeboPosePlugin::OnUpdate(const common::UpdateInfo& _info) {
    if(gazebo_seq_++ % measurement_divisor_ != 0)
      return;

    geometry_msgs::PoseWithCovarianceStamped pose_msg;

    math::Pose pose = link_->GetWorldCoGPose();
    pose_msg.header.frame_id = frame_id_;
    pose_msg.header.seq = pose_seq_++;
    pose_msg.header.stamp.sec  = (world_->GetSimTime()).sec;
    pose_msg.header.stamp.nsec = (world_->GetSimTime()).nsec;

    double np = noise_normal_p_ * noise_normal_p_;
    double nq = noise_normal_q_ * noise_normal_q_;

    copyPosition(pose.pos, pose_msg.pose.pose.position);
    pose_msg.pose.pose.orientation.w = pose.rot.w;
    pose_msg.pose.pose.orientation.x = pose.rot.x;
    pose_msg.pose.pose.orientation.y = pose.rot.y;
    pose_msg.pose.pose.orientation.z = pose.rot.z;

    Eigen::Map<Eigen::Matrix<double, 6, 6>> cov(pose_msg.pose.covariance.data());
    Eigen::Matrix<double, 6, 1> covd;
    covd << np, np, np, nq, nq, nq;
    cov = covd.asDiagonal();

    pose_pub_.publish(pose_msg);
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboPosePlugin);
}

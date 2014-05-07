//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>
// All rights reserved.
//
// TODO(ff): Enter some license
//==============================================================================
#include <mav_gazebo_plugins/gazebo_bag_plugin.h>
#include <ctime>


namespace gazebo
{
  GazeboBagPlugin::GazeboBagPlugin() : 
    ModelPlugin(), node_handle_(0) {}

  GazeboBagPlugin::~GazeboBagPlugin() {
    event::Events::DisconnectWorldUpdateBegin(update_connection_);
    if (node_handle_) {
      node_handle_->shutdown();
      delete node_handle_;
    }
  };

  // void GazeboBagPlugin::InitializeParams() {};
  // void GazeboBagPlugin::Publish() {};

  void GazeboBagPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Store the pointer to the model
    model_ = _model;
    // world_ = physics::get_world(model_->world.name);
    world_ = model_->GetWorld();

    // default params
    namespace_.clear();
    pose_topic_ = "ground_truth";
    frame_id_ = "/ground_truth_pose";
    link_name_ = "base_link";

    if (_sdf->HasElement("robotNamespace"))
      namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
      gzerr << "[gazebo_bag_plugin] Please specify a robotNamespace.\n";
    node_handle_ = new ros::NodeHandle(namespace_);

    if (_sdf->HasElement("bagFileName"))
      bag_filename_ = _sdf->GetElement("bagFileName")->Get<std::string>();
    else
      gzerr << "[gazebo_bag_plugin] Please specify a bagFileName.\n";

    if (_sdf->HasElement("frameId"))
    frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();

    if (_sdf->HasElement("linkName"))
      link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
    else
      gzwarn << "[gazebo_bag_plugin] No linkName specified, using default "
        << link_name_ <<".\n";
    // Get the pointer to the link
    link_ = this->model_->GetLink(link_name_);

    if (_sdf->HasElement("poseTopic"))
      pose_topic_ = _sdf->GetElement("poseTopic")->Get<std::string>();

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GazeboBagPlugin::OnUpdate, this, _1));

    ground_truth_pose_pub_ = node_handle_->advertise<geometry_msgs::PoseStamped>(
      pose_topic_, 10);
    pose_msg_.header.frame_id = frame_id_;
    time_t rawtime;
    struct tm *timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,80,"%Y-%m-%d-%H-%M-%S",timeinfo);
    std::string date_time_str(buffer);
    // TODO(ff): use put_time here, once it's supported in gcc
    bag_filename_ = date_time_str + std::string("_") + bag_filename_;

    // Open a bag file and store it in ~/.ros/<bag_filename_>
    bag_.open(bag_filename_, rosbag::bagmode::Write);
  }

  // Called by the world update start event
  void GazeboBagPlugin::OnUpdate(const common::UpdateInfo& _info)
  {
    common::Time now = world_->GetSimTime();
    ros::Time ros_now = ros::Time(now.sec, now.nsec);
    pose_ = link_->GetWorldCoGPose();
    pose_msg_.header.stamp.sec  = now.sec;
    pose_msg_.header.stamp.nsec = now.nsec;
    pose_msg_.pose.position.x = pose_.pos.x;
    pose_msg_.pose.position.y = pose_.pos.y;
    pose_msg_.pose.position.z = pose_.pos.z;
    pose_msg_.pose.orientation.w = pose_.rot.w;
    pose_msg_.pose.orientation.x = pose_.rot.x;
    pose_msg_.pose.orientation.y = pose_.rot.y;
    pose_msg_.pose.orientation.z = pose_.rot.z;
    bag_.write("ground_truth/pose", ros_now, pose_msg_);
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboBagPlugin);
}

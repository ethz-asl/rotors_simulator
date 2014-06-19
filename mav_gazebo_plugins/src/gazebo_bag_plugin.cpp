//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>
// All rights reserved.
//
// TODO(ff): Enter some license
//==============================================================================
#include <mav_gazebo_plugins/gazebo_bag_plugin.h>
#include <ctime>
#include <mav_msgs/MotorSpeed.h>
#include <mav_gazebo_plugins/common.h>


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
    bag_.close();
  };

  // void GazeboBagPlugin::InitializeParams() {};
  // void GazeboBagPlugin::Publish() {};

  void GazeboBagPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Store the pointer to the model
    model_ = _model;
    // world_ = physics::get_world(model_->world.name);
    world_ = model_->GetWorld();
    namespace_.clear();

    if (_sdf->HasElement("robotNamespace"))
      namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
      gzerr << "[gazebo_bag_plugin] Please specify a robotNamespace.\n";
    node_handle_ = new ros::NodeHandle(namespace_);

    // default params
    ground_truth_pose_pub_topic_ = "/" + namespace_ + "/ground_truth/pose";
    ground_truth_twist_pub_topic_ = "/" + namespace_ + "/ground_truth/twist";
    imu_pub_topic_ = "/" + namespace_ + "/imu";
    imu_sub_topic_ = "/" + namespace_ + "/imu";
    control_attitude_thrust_sub_topic_ = "/" + namespace_ + "/mav_attitude_cmd";
    control_attitude_thrust_pub_topic_ = "/" + namespace_ + "/mav_attitude_cmd";
    control_motor_speed_sub_topic_ = "/" + namespace_ + "/mav_motor_cmd";
    control_motor_speed_pub_topic_ = "/" + namespace_ + "/mav_motor_cmd";
    control_rate_thrust_sub_topic_ = "/" + namespace_ + "/mav_rate_cmd";
    control_rate_thrust_pub_topic_ = "/" + namespace_ + "/mav_rate_cmd";
    motor_pub_topic_ = "/" + namespace_ + "/motor_vel";
    frame_id_ = "ground_truth_pose";
    link_name_ = "base_link";

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

    if (_sdf->HasElement("imuSubTopic"))
      imu_sub_topic_ = _sdf->GetElement("imuSubTopic")->Get<std::string>();

    if (_sdf->HasElement("imuPubTopic"))
      imu_pub_topic_ = _sdf->GetElement("imuPubTopic")->Get<std::string>();

    if (_sdf->HasElement("controlAttitudeThrustSubTopic"))
      control_attitude_thrust_sub_topic_ = _sdf->GetElement(
        "controlAttitudeThrustSubTopic")->Get<std::string>();

    if (_sdf->HasElement("controlAttitudeThrustPubTopic"))
      control_attitude_thrust_pub_topic_ = _sdf->GetElement(
        "controlAttitudeThrustPubTopic")->Get<std::string>();

    if (_sdf->HasElement("controlMotorSpeedSubTopic"))
      control_motor_speed_sub_topic_ = _sdf->GetElement(
        "controlMotorSpeedSubTopic")->Get<std::string>();

    if (_sdf->HasElement("controlMotorSpeedPubTopic"))
      control_motor_speed_pub_topic_ = _sdf->GetElement(
        "controlMotorSpeedPubTopic")->Get<std::string>();

    if (_sdf->HasElement("controlRateThrustSubTopic"))
      control_rate_thrust_sub_topic_ = _sdf->GetElement(
        "controlRateThrustSubTopic")->Get<std::string>();

    if (_sdf->HasElement("controlRateThrustPubTopic"))
      control_rate_thrust_pub_topic_ = _sdf->GetElement(
        "controlRateThrustPubTopic")->Get<std::string>();

    if (_sdf->HasElement("motorPubTopic"))
      motor_pub_topic_ = _sdf->GetElement("motorPubTopic")->Get<std::string>();

    if (_sdf->HasElement("poseTopic"))
      ground_truth_pose_pub_topic_ =
        _sdf->GetElement("poseTopic")->Get<std::string>();

    getSdfParam<double>(_sdf, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_, 10);

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GazeboBagPlugin::OnUpdate, this, _1));

    time_t rawtime;
    struct tm *timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,80,"%Y-%m-%d-%H-%M-%S",timeinfo);
    std::string date_time_str(buffer);
    // TODO(ff): use put_time here, once it's supported in gcc

    std::string key(".bag");
    size_t pos = bag_filename_.rfind(key);
    if (pos != std::string::npos)
      bag_filename_.erase(pos, key.length());
    bag_filename_ = bag_filename_ + "_" + date_time_str + ".bag";

    // Open a bag file and store it in ~/.ros/<bag_filename_>
    bag_.open(bag_filename_, rosbag::bagmode::Write);


    // // Get the contact manager.
    // contact_mgr_ = world_->GetPhysicsEngine()->GetContactManager();

    // for (unsigned int j = 0; j < link_->GetChildCount(); ++j)
    // {
    //   physics::CollisionPtr collision = link_->GetCollision(j);
    //   std::map<std::string, physics::CollisionPtr>::iterator coll_iter
    //     = this->collisions.find(collision->GetScopedName());
    //   if (coll_iter != this->collisions.end())
    //     continue;

    //   this->collisions[collision->GetScopedName()] = collision;
    // }

    // if (!this->collisions.empty())
    // {
    //   // request the contact manager to publish messages to a custom topic for
    //   // this sensor
    //   physics::ContactManager *mgr =
    //       this->world->GetPhysicsEngine()->GetContactManager();
    //   std::string topic = mgr->CreateFilter(this->GetName(), this->collisions);
    //   if (!this->contactSub)
    //   {
    //     this->contactSub = this->node->Subscribe(topic,
    //         &GazeboBagPlugin::OnContacts, this);
    //   }
    // }

    child_links_ = link_->GetChildJointsLinks();
    for (unsigned int i = 0; i < child_links_.size(); i++) {
      std::string link_name = child_links_[i]->GetScopedName();

      // Check if link contains rotor_ in its name
      int pos = link_name.find("rotor_");
      if (pos != link_name.npos) {
        std::string motor_number_str = link_name.substr(pos + 6);
        unsigned int motor_number = std::stoi(motor_number_str);
        std::string joint_name = child_links_[i]->GetName() + "_joint";
        physics::JointPtr joint = this->model_->GetJoint(joint_name);
        motor_joints_.insert(MotorNumberToJointPair(motor_number, joint));
      }
    }

    // Subscriber to IMU Sensor
    imu_sub_ = node_handle_->subscribe(imu_sub_topic_, 10,
      &GazeboBagPlugin::ImuCallback, this);

    // Subscriber to Control Attitude Thrust Message
    control_attitude_thrust_sub_ = node_handle_->subscribe(
      control_attitude_thrust_sub_topic_, 10,
      &GazeboBagPlugin::ControlAttitudeThrustCallback, this);

    // Subscriber to Control Motor Speed Message
    control_motor_speed_sub_ = node_handle_->subscribe(
      control_motor_speed_sub_topic_, 10,
      &GazeboBagPlugin::ControlMotorSpeedCallback, this);

    // Subscriber to Control Rate Thrust Message
    control_rate_thrust_sub_ = node_handle_->subscribe(
      control_rate_thrust_sub_topic_, 10,
      &GazeboBagPlugin::ControlRateThrustCallback, this);
  }

  // Called by the world update start event
  void GazeboBagPlugin::OnUpdate(const common::UpdateInfo& _info)
  {
    // Get the current simulation time.
    common::Time now = world_->GetSimTime();

    // TODO(ff): Make this work, currently it gives the wrong contacts and
    //           collisions, there is always only one collision.

    // unsigned int contact_count = contact_mgr_->GetContactCount();
    // std::cout << "contact_count: " << contact_count << "\n";
    // physics::Collision_V collisions = link_->GetCollisions();
    // for (int i = 0; i < collisions.size(); i++) {
    //   std::cout<<"link_collision[" << i << "]: "
    //     <<collisions[i]->GetScopedName()<<"\n";
    // }

    this->LogGroundTruth(now);
    this->LogMotorVelocities(now);
  }

  void GazeboBagPlugin::ImuCallback(const sensor_msgs::ImuPtr& imu_msg) {
    ros::Time t(imu_msg->header.stamp.sec,imu_msg->header.stamp.nsec);
    writeBag(imu_pub_topic_, imu_msg->header.stamp, imu_msg);
  }

  void GazeboBagPlugin::ControlAttitudeThrustCallback(const mav_msgs::ControlAttitudeThrustPtr& control_msg) {
    ros::Time t(control_msg->header.stamp.sec,control_msg->header.stamp.nsec);
    writeBag(control_attitude_thrust_pub_topic_, t, control_msg);
  }

  void GazeboBagPlugin::ControlMotorSpeedCallback(const mav_msgs::ControlMotorSpeedPtr& control_msg) {
    ros::Time t(control_msg->header.stamp.sec,control_msg->header.stamp.nsec);
    writeBag(control_motor_speed_pub_topic_, t, control_msg);
  }

  void GazeboBagPlugin::ControlRateThrustCallback(const mav_msgs::ControlRateThrustPtr& control_msg) {
    ros::Time t(control_msg->header.stamp.sec,control_msg->header.stamp.nsec);
    writeBag(control_rate_thrust_pub_topic_, t, control_msg);
  }

  void GazeboBagPlugin::LogMotorVelocities(const common::Time now) {
    ros::Time ros_now = ros::Time(now.sec, now.nsec);

    mav_msgs::MotorSpeed rot_velocities_msg;
    rot_velocities_msg.motor_speed.resize(motor_joints_.size());

    MotorNumberToJointMap::iterator m;
    for (m = motor_joints_.begin(); m != motor_joints_.end(); ++m) {
      double motor_rot_vel = m->second->GetVelocity(0) * rotor_velocity_slowdown_sim_;
      rot_velocities_msg.motor_speed[m->first] = motor_rot_vel;
    }
    rot_velocities_msg.header.stamp.sec  = now.sec;
    rot_velocities_msg.header.stamp.nsec = now.nsec;

    writeBag(motor_pub_topic_, ros_now, rot_velocities_msg);
  }

  void GazeboBagPlugin::LogGroundTruth(const common::Time now) {
    ros::Time ros_now = ros::Time(now.sec, now.nsec);

    geometry_msgs::PoseStamped pose_msg;
    geometry_msgs::TwistStamped twist_msg;

    // Get pose and update the message.
    math::Pose pose = link_->GetWorldPose();
    pose_msg.header.frame_id = frame_id_;
    pose_msg.header.stamp.sec  = now.sec;
    pose_msg.header.stamp.nsec = now.nsec;
    pose_msg.pose.position.x = pose.pos.x;
    pose_msg.pose.position.y = pose.pos.y;
    pose_msg.pose.position.z = pose.pos.z;
    pose_msg.pose.orientation.w = pose.rot.w;
    pose_msg.pose.orientation.x = pose.rot.x;
    pose_msg.pose.orientation.y = pose.rot.y;
    pose_msg.pose.orientation.z = pose.rot.z;

    writeBag(ground_truth_pose_pub_topic_, ros_now, pose_msg);

    // Get twist and update the message.
    math::Vector3 linear_veloctiy = link_->GetWorldLinearVel();
    math::Vector3 angular_veloctiy = link_->GetWorldAngularVel();
    twist_msg.header.frame_id = frame_id_;
    twist_msg.header.stamp.sec  = now.sec;
    twist_msg.header.stamp.nsec = now.nsec;
    twist_msg.twist.linear.x = linear_veloctiy.x;
    twist_msg.twist.linear.y = linear_veloctiy.y;
    twist_msg.twist.linear.z = linear_veloctiy.z;
    twist_msg.twist.angular.x = angular_veloctiy.x;
    twist_msg.twist.angular.y = angular_veloctiy.y;
    twist_msg.twist.angular.z = angular_veloctiy.z;

    writeBag(ground_truth_twist_pub_topic_, ros_now, twist_msg);
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboBagPlugin);
}

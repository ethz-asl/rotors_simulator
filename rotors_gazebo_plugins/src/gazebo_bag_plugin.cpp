/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "rotors_gazebo_plugins/gazebo_bag_plugin.h"

#include <ctime>

#include <mav_msgs/Actuators.h>

namespace gazebo {

GazeboBagPlugin::~GazeboBagPlugin() {
  event::Events::DisconnectWorldUpdateBegin(update_connection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
  bag_.close();
}

// void GazeboBagPlugin::InitializeParams() {};
// void GazeboBagPlugin::Publish() {};

void GazeboBagPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  // world_ = physics::get_world(model_->world.name);
  world_ = model_->GetWorld();

  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  }
  else {
    gzerr << "[gazebo_bag_plugin] Please specify a robotNamespace.\n";
  }
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("bagFileName")) {
    bag_filename_ = _sdf->GetElement("bagFileName")->Get<std::string>();
  }
  else {
    gzerr << "[gazebo_bag_plugin] Please specify a bagFileName.\n";
  }

  if (_sdf->HasElement("linkName")) {
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  }
  else {
    gzwarn << "[gazebo_bag_plugin] No linkName specified, using default " << link_name_ << ".\n";
  }

  // Get the pointer to the link
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL) {
    gzthrow("[gazebo_bag_plugin] Couldn't find specified link \"" << link_name_ << "\".");
  }

  getSdfParam<std::string>(_sdf, "frameId", frame_id_, frame_id_);
  getSdfParam<std::string>(_sdf, "imuTopic", imu_topic_, imu_topic_);
  getSdfParam<std::string>(_sdf, "commandAttitudeThrustTopic", control_attitude_thrust_topic_,
                           control_attitude_thrust_topic_);
  getSdfParam<std::string>(_sdf, "commandMotorSpeedTopic", control_motor_speed_topic_,
                           control_motor_speed_topic_);
  getSdfParam<std::string>(_sdf, "commandRateThrustTopic", control_rate_thrust_topic_,
                           control_rate_thrust_topic_);
  getSdfParam<std::string>(_sdf, "motorTopic", motor_topic_, motor_topic_);
  getSdfParam<std::string>(_sdf, "poseTopic", ground_truth_pose_topic_, ground_truth_pose_topic_);
  getSdfParam<std::string>(_sdf, "twistTopic", ground_truth_twist_topic_, ground_truth_twist_topic_);
  getSdfParam<std::string>(_sdf, "wrenchesTopic", wrench_topic_, wrench_topic_);
  getSdfParam<std::string>(_sdf, "windTopic", wind_topic_, wind_topic_);
  getSdfParam<std::string>(_sdf, "waypointTopic", waypoint_topic_, waypoint_topic_);
  getSdfParam<std::string>(_sdf, "commandPoseTopic", command_pose_topic_, command_pose_topic_);
  getSdfParam<std::string>(_sdf, "recordingServiceName", recording_service_name_, recording_service_name_);

  getSdfParam<double>(_sdf, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_,
                      rotor_velocity_slowdown_sim_);

  getSdfParam<bool>(_sdf, "waitToRecordBag", wait_to_record_, wait_to_record_);

  recording_service_ = node_handle_->advertiseService(recording_service_name_,
                                                      &GazeboBagPlugin::RecordingServiceCallback,
                                                      this);

  // Get the motor joints.
  child_links_ = link_->GetChildJointsLinks();
  for (unsigned int i = 0; i < child_links_.size(); i++) {
    std::string link_name = child_links_[i]->GetScopedName();

    // Check if the link contains rotor_ in its name.
    int pos = link_name.find("rotor_");
    if (pos != link_name.npos) {
      std::string motor_number_str = link_name.substr(pos + 6);
      unsigned int motor_number = std::stoi(motor_number_str);
      std::string joint_name = child_links_[i]->GetName() + "_joint";
      physics::JointPtr joint = model_->GetJoint(joint_name);
      motor_joints_.insert(MotorNumberToJointPair(motor_number, joint));
    }
  }

  // Get the contact manager.
  std::vector<std::string> collisions;
  contact_mgr_ = world_->GetPhysicsEngine()->GetContactManager();
  for (unsigned int i = 0; i < link_->GetCollisions().size(); ++i) {
    physics::CollisionPtr collision = link_->GetCollision(i);
    collisions.push_back(collision->GetScopedName());
  }
  for (unsigned int j = 0; j < child_links_.size(); ++j) {
    unsigned int zero = 0;
    for (unsigned int i = 0; i < child_links_[j]->GetCollisions().size(); ++i) {
      collisions.push_back(child_links_[j]->GetCollision(i)->GetScopedName());
    }
  }

  if (!collisions.empty()) {
    contact_mgr_->CreateFilter(link_->GetName(), collisions);
  }

  // If we do not need to wait for user command, we start recording right away
  if (!wait_to_record_)
    StartRecording();
}

// This gets called by the world update start event.
void GazeboBagPlugin::OnUpdate(const common::UpdateInfo& _info) {
  // Get the current simulation time.
  common::Time now = world_->GetSimTime();
  LogWrenches(now);
  LogGroundTruth(now);
  LogMotorVelocities(now);
}

void GazeboBagPlugin::StartRecording() {
  time_t rawtime;
  struct tm *timeinfo;
  char buffer[80];

  time(&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer, 80, "%Y-%m-%d-%H-%M-%S", timeinfo);
  std::string date_time_str(buffer);

  std::string key(".bag");
  size_t pos = bag_filename_.rfind(key);
  if (pos != std::string::npos) {
    bag_filename_.erase(pos, key.length());
  }
  std::string full_bag_filename = bag_filename_ + "_" + date_time_str + ".bag";

  // Open a bag file and store it in ~/.ros/<full_bag_filename>.
  bag_.open(full_bag_filename, rosbag::bagmode::Write);

  // Subscriber to IMU sensor_msgs::Imu Message.
  imu_sub_ = node_handle_->subscribe(imu_topic_, 10, &GazeboBagPlugin::ImuCallback, this);

  // Subscriber to Wind WrenchStamped Message.
  wind_sub_ = node_handle_->subscribe(wind_topic_, 10, &GazeboBagPlugin::WindCallback, this);

  // Subscriber to Waypoint MultiDOFJointTrajectory Message.
  waypoint_sub_ = node_handle_->subscribe(waypoint_topic_, 10, &GazeboBagPlugin::WaypointCallback, this);

  // Subscriber to PoseStamped pose command message.
  command_pose_sub_ = node_handle_->subscribe(command_pose_topic_, 10, &GazeboBagPlugin::CommandPoseCallback, this);

  // Subscriber to Control Attitude Thrust Message.
  control_attitude_thrust_sub_ = node_handle_->subscribe(control_attitude_thrust_topic_, 10,
                                                           &GazeboBagPlugin::AttitudeThrustCallback, this);

  // Subscriber to Control Motor Speed Message.
  control_motor_speed_sub_ = node_handle_->subscribe(control_motor_speed_topic_, 10,
                                                       &GazeboBagPlugin::ActuatorsCallback, this);

  // Subscriber to Control Rate Thrust Message.
  control_rate_thrust_sub_ = node_handle_->subscribe(control_rate_thrust_topic_, 10,
                                                       &GazeboBagPlugin::RateThrustCallback, this);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboBagPlugin::OnUpdate,
                                                                          this, _1));

  // Set the flag that we are actively recording.
  is_recording_ = true;

  ROS_INFO("GazeboBagPlugin START recording bagfile %s", full_bag_filename.c_str());
}

void GazeboBagPlugin::StopRecording() {
  // Shutdown all the subscribers.
  imu_sub_.shutdown();
  wind_sub_.shutdown();
  waypoint_sub_.shutdown();
  command_pose_sub_.shutdown();
  control_attitude_thrust_sub_.shutdown();
  control_motor_speed_sub_.shutdown();
  control_rate_thrust_sub_.shutdown();

  // Disconnect the update event.
  event::Events::DisconnectWorldUpdateBegin(update_connection_);

  // Close the bag.
  bag_.close();

  // Clear the flag to show that we are not actively recording
  is_recording_ = false;

  ROS_INFO("GazeboBagPlugin STOP recording bagfile");
}

void GazeboBagPlugin::ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {
  common::Time now = world_->GetSimTime();
  ros::Time ros_now = ros::Time(now.sec, now.nsec);
  writeBag(namespace_ + "/" + imu_topic_, ros_now, imu_msg);
}

void GazeboBagPlugin::WindCallback(const geometry_msgs::WrenchStampedConstPtr& wind_msg) {
  common::Time now = world_->GetSimTime();
  ros::Time ros_now = ros::Time(now.sec, now.nsec);
  writeBag(namespace_ + "/" + wind_topic_, ros_now, wind_msg);
}

void GazeboBagPlugin::WaypointCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_msg) {
  common::Time now = world_->GetSimTime();
  ros::Time ros_now = ros::Time(now.sec, now.nsec);
  writeBag(namespace_ + "/" + waypoint_topic_, ros_now, trajectory_msg);
}

void GazeboBagPlugin::CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  common::Time now = world_->GetSimTime();
  ros::Time ros_now = ros::Time(now.sec, now.nsec);
  writeBag(namespace_ + "/" + command_pose_topic_, ros_now, pose_msg);
}

void GazeboBagPlugin::AttitudeThrustCallback(
    const mav_msgs::AttitudeThrustConstPtr& control_msg) {
  common::Time now = world_->GetSimTime();
  ros::Time ros_now = ros::Time(now.sec, now.nsec);
  writeBag(namespace_ + "/" + control_attitude_thrust_topic_, ros_now, control_msg);
}

void GazeboBagPlugin::ActuatorsCallback(const mav_msgs::ActuatorsConstPtr& control_msg) {
  common::Time now = world_->GetSimTime();
  ros::Time ros_now = ros::Time(now.sec, now.nsec);
  writeBag(namespace_ + "/" + control_motor_speed_topic_, ros_now, control_msg);
}

void GazeboBagPlugin::RateThrustCallback(const mav_msgs::RateThrustConstPtr& control_msg) {
  common::Time now = world_->GetSimTime();
  ros::Time ros_now = ros::Time(now.sec, now.nsec);
  writeBag(namespace_ + "/" + control_rate_thrust_topic_, ros_now, control_msg);
}

void GazeboBagPlugin::LogMotorVelocities(const common::Time now) {
  ros::Time ros_now = ros::Time(now.sec, now.nsec);

  mav_msgs::Actuators rot_velocities_msg;
  rot_velocities_msg.angular_velocities.resize(motor_joints_.size());

  MotorNumberToJointMap::iterator m;
  for (m = motor_joints_.begin(); m != motor_joints_.end(); ++m) {
    double motor_rot_vel = m->second->GetVelocity(0) * rotor_velocity_slowdown_sim_;
    rot_velocities_msg.angular_velocities[m->first] = motor_rot_vel;
  }
  rot_velocities_msg.header.stamp.sec = now.sec;
  rot_velocities_msg.header.stamp.nsec = now.nsec;

  writeBag(namespace_ + "/" + motor_topic_, ros_now, rot_velocities_msg);
}

void GazeboBagPlugin::LogGroundTruth(const common::Time now) {
  ros::Time ros_now = ros::Time(now.sec, now.nsec);

  geometry_msgs::PoseStamped pose_msg;
  geometry_msgs::TwistStamped twist_msg;

  // Get pose and update the message.
  math::Pose pose = link_->GetWorldPose();
  pose_msg.header.frame_id = frame_id_;
  pose_msg.header.stamp.sec = now.sec;
  pose_msg.header.stamp.nsec = now.nsec;
  pose_msg.pose.position.x = pose.pos.x;
  pose_msg.pose.position.y = pose.pos.y;
  pose_msg.pose.position.z = pose.pos.z;
  pose_msg.pose.orientation.w = pose.rot.w;
  pose_msg.pose.orientation.x = pose.rot.x;
  pose_msg.pose.orientation.y = pose.rot.y;
  pose_msg.pose.orientation.z = pose.rot.z;

  writeBag(namespace_ + "/" + ground_truth_pose_topic_, ros_now, pose_msg);

  // Get twist and update the message.
  math::Vector3 linear_veloctiy = link_->GetWorldLinearVel();
  math::Vector3 angular_veloctiy = link_->GetWorldAngularVel();
  twist_msg.header.frame_id = frame_id_;
  twist_msg.header.stamp.sec = now.sec;
  twist_msg.header.stamp.nsec = now.nsec;
  twist_msg.twist.linear.x = linear_veloctiy.x;
  twist_msg.twist.linear.y = linear_veloctiy.y;
  twist_msg.twist.linear.z = linear_veloctiy.z;
  twist_msg.twist.angular.x = angular_veloctiy.x;
  twist_msg.twist.angular.y = angular_veloctiy.y;
  twist_msg.twist.angular.z = angular_veloctiy.z;

  writeBag(namespace_ + "/" + ground_truth_twist_topic_, ros_now, twist_msg);
}

void GazeboBagPlugin::LogWrenches(const common::Time now) {
  geometry_msgs::WrenchStamped wrench_msg;
  std::vector<physics::Contact *> contacts = contact_mgr_->GetContacts();
  for (int i = 0; i < contact_mgr_->GetContactCount(); ++i) {
    std::string collision2_name = contacts[i]->collision2->GetLink()->GetScopedName();
    double body1_force = contacts[i]->wrench->body1Force.GetLength();

    // Exclude extremely small forces.
    if (body1_force < 1e-10)
      continue;
    // Do this, such that all the contacts are logged.
    // (publishing on the same topic with the same time stamp is impossible)
    ros::Time ros_now = ros::Time(now.sec, now.nsec + i*1000);
    std::string collision1_name = contacts[i]->collision1->GetLink()->GetScopedName();
    wrench_msg.header.frame_id = collision1_name + "--" + collision2_name;
    wrench_msg.header.stamp.sec = now.sec;
    wrench_msg.header.stamp.nsec = now.nsec;
    wrench_msg.wrench.force.x = contacts[i]->wrench->body1Force.x;
    wrench_msg.wrench.force.y = contacts[i]->wrench->body1Force.y;
    wrench_msg.wrench.force.z = contacts[i]->wrench->body1Force.z;
    wrench_msg.wrench.torque.x = contacts[i]->wrench->body1Torque.x;
    wrench_msg.wrench.torque.y = contacts[i]->wrench->body1Torque.y;
    wrench_msg.wrench.torque.z = contacts[i]->wrench->body1Torque.z;

    writeBag(namespace_ + "/" + wrench_topic_, ros_now, wrench_msg);
  }
}

bool GazeboBagPlugin::RecordingServiceCallback(rotors_comm::RecordRosbag::Request& req,
                                               rotors_comm::RecordRosbag::Response& res) {
  if (req.record && !is_recording_) {
    StartRecording();
    res.success = true;
  }
  else if (req.record && is_recording_) {
    gzwarn << "[gazebo_bag_plugin] Already recording rosbag, ignoring start command.\n";
    res.success = false;
  }
  else if (!req.record && is_recording_) {
    StopRecording();
    res.success = true;
  }
  else if (!req.record && !is_recording_) {
    gzwarn << "[gazebo_bag_plugin] Already not recording rosbag, ignoring stop command.\n";
    res.success = false;
  }

  return res.success;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboBagPlugin);
}

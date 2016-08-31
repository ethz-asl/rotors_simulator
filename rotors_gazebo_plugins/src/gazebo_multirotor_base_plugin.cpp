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

#include "rotors_gazebo_plugins/gazebo_multirotor_base_plugin.h"

#include <ctime>

#include <mav_msgs/Actuators.h>

namespace gazebo {

GazeboMultirotorBasePlugin::~GazeboMultirotorBasePlugin() {
  event::Events::DisconnectWorldUpdateBegin(update_connection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboMultirotorBasePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model_ = _model;
  world_ = model_->GetWorld();
  namespace_.clear();
  wind_speed_W_ = math::Vector3::Zero;
  const sdf::Vector3 zeros3_sdf(0.0, 0.0, 0.0);
  sdf::Vector3 main_body_drag_sdf;

  getSdfParam<std::string>(_sdf, "robotNamespace", namespace_, namespace_, true);
  getSdfParam<std::string>(_sdf, "linkName", link_name_, link_name_, true);
  getSdfParam<std::string>(_sdf, "motorPubTopic", motor_pub_topic_, motor_pub_topic_);
  getSdfParam<double>(_sdf, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_,
                      rotor_velocity_slowdown_sim_);
  getSdfParam<std::string>(_sdf, "windSpeedSubTopic", wind_speed_sub_topic_, wind_speed_sub_topic_);

  getSdfParam<sdf::Vector3>(_sdf, "mainBodyDragCoefficient", main_body_drag_sdf, zeros3_sdf);
  main_body_drag_coefficient_ = math::Vector3(main_body_drag_sdf.x, main_body_drag_sdf.y, main_body_drag_sdf.z);

  node_handle_ = new ros::NodeHandle(namespace_);
  motor_pub_ = node_handle_->advertise<mav_msgs::Actuators>(motor_pub_topic_, 10);
  joint_state_pub_ = node_handle_->advertise<sensor_msgs::JointState>(joint_state_pub_topic_, 1);

  wind_speed_sub_ = node_handle_->subscribe(wind_speed_sub_topic_, 1, &GazeboMultirotorBasePlugin::WindSpeedCallback, this);

  frame_id_ = link_name_;

  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_multirotor_base_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboMultirotorBasePlugin::OnUpdate, this, _1));

  child_links_ = link_->GetChildJointsLinks();

  double total_mass = 0;
  Eigen::Vector3d weighted_position = Eigen::Vector3d::Zero();
  Eigen::Matrix3d total_inertia_B = Eigen::Matrix3d::Zero();

  double mass = link_->GetInertial()->GetMass();
  math::Pose T_WB = link_->GetRelativePose();
  math::Pose T_BC = link_->GetInertial()->GetPose(); // Get CoG relative to link.
  math::Pose T_WC = T_WB + T_BC;
  Eigen::Vector3d position_eigen;
  vector3ToEigen(T_WC.pos, &position_eigen);

  Eigen::Matrix3d inertia_B;
  getInertiaAtPosition(link_, T_WC.pos, &inertia_B);

  total_mass = mass;
  weighted_position = weighted_position + mass * position_eigen;
  total_inertia_B += inertia_B;

  for (unsigned int i = 0; i < child_links_.size(); i++) {
    std::string link_name = child_links_[i]->GetScopedName();

    math::Pose T_BL = child_links_[i]->GetRelativePose();
    math::Pose T_LC = child_links_[i]->GetInertial()->GetPose(); // Get CoG relative to link.
    math::Pose T_BC = T_BL + T_LC;

    math::Vector3 position = T_BC.pos;
    Eigen::Vector3d position_eigen;
    vector3ToEigen(position, &position_eigen);
    double mass = child_links_[i]->GetInertial()->GetMass();

    Eigen::Matrix3d inertia_B;
    getInertiaAtPosition(child_links_[i], position, &inertia_B);

    weighted_position = weighted_position + mass * position_eigen;
    total_mass += mass;
    total_inertia_B += inertia_B;

    // Check if link contains rotor_ in its name.
    int pos = link_name.find("rotor_");
    if (pos != link_name.npos) {
      std::string motor_number_str = link_name.substr(pos + 6);
      unsigned int motor_number = std::stoi(motor_number_str);
      std::string joint_name = child_links_[i]->GetName() + "_joint";
      physics::JointPtr joint = this->model_->GetJoint(joint_name);
      motor_joints_.insert(MotorNumberToJointPair(motor_number, joint));
    }
  }

  Eigen::Vector3d position_B_C = 1/total_mass * weighted_position;

  std::cout << "[Multirotor base]:" << std::endl;
  std::cout << "CoG full body: ";
  std::cout << "[" << position_B_C.transpose() << "]" << std::endl;
  std::cout << "Total mass: " << total_mass << "kg" << std::endl;
  std::cout << "Total inertia at B: " << std::endl;
  std::cout << total_inertia_B << std::endl;

  Eigen::Matrix3d total_inertia_C;
  Eigen::Matrix3d position_B_C_skew;

  getSkewMatrix(position_B_C, &position_B_C_skew);

  total_inertia_C = total_inertia_B - total_mass*position_B_C_skew*position_B_C_skew.transpose();

  std::cout << "Total inertia at C:" << std::endl;
  std::cout << total_inertia_C << std::endl;

}

void GazeboMultirotorBasePlugin::getInertiaAtPosition(physics::LinkPtr link, math::Vector3 position_B_L, Eigen::Matrix3d* inertia_B) {

  math::Matrix3 inertia_L = link->GetInertial()->GetMOI(link->GetInertial()->GetPose());
  // Gazebo also provides a function to shift inertia. Not sure if this works on old gazebo though.
  // TODO(burrimi) Check and switch back.
  // math::Matrix3 inertia_B_gazebo = link->GetInertial()->GetMOI(link->GetRelativePose());

  double mass = link->GetInertial()->GetMass();

  Eigen::Matrix3d inertia_L_eigen;
  Eigen::Vector3d position_B_L_eigen;
  Eigen::Matrix3d position_B_L_skew;

  vector3ToEigen(position_B_L, &position_B_L_eigen);
  matrix3ToEigen(inertia_L, &inertia_L_eigen);
  getSkewMatrix(position_B_L_eigen, &position_B_L_skew);

  *inertia_B = inertia_L_eigen + mass * position_B_L_skew * position_B_L_skew.transpose();
}

void GazeboMultirotorBasePlugin::WindSpeedCallback(const rotors_comm::WindSpeedConstPtr& wind_speed) {
  // TODO(burrimi): Transform velocity to world frame if frame_id is set to something else.
  wind_speed_W_.x = wind_speed->velocity.x;
  wind_speed_W_.y = wind_speed->velocity.y;
  wind_speed_W_.z = wind_speed->velocity.z;
}

void GazeboMultirotorBasePlugin::ApplyForceOnMainBody() {
  math::Vector3 total_force_B(0, 0, 0);

  math::Vector3 velocity_current_W = link_->GetWorldLinearVel();
  math::Pose T_W_B = link_->GetWorldPose(); //TODO(burrimi): Check tf.
  math::Quaternion q_W_B = T_W_B.rot;

  // Calculate main body drag.
  math::Vector3 relative_velocity_W = velocity_current_W - wind_speed_W_;
  math::Vector3 relative_velocity_B = q_W_B.RotateVectorReverse(relative_velocity_W);
  math::Vector3 main_body_drag_force_B = main_body_drag_coefficient_*relative_velocity_B.GetAbs()*relative_velocity_B;
  // Subtract main body drag force from total forces on body.
  total_force_B -= main_body_drag_force_B;

  link_->AddRelativeForce(total_force_B);
}

// This gets called by the world update start event.
void GazeboMultirotorBasePlugin::OnUpdate(const common::UpdateInfo& _info) {
  // Get the current simulation time.
  common::Time now = world_->GetSimTime();

  ApplyForceOnMainBody();

  mav_msgs::ActuatorsPtr msg(new mav_msgs::Actuators);
  msg->angular_velocities.resize(motor_joints_.size());
  sensor_msgs::JointStatePtr joint_state_msg(new sensor_msgs::JointState);
  joint_state_msg->name.resize(motor_joints_.size());
  joint_state_msg->position.resize(motor_joints_.size());
  MotorNumberToJointMap::iterator m;
  for (m = motor_joints_.begin(); m != motor_joints_.end(); ++m) {
    double motor_rot_vel = m->second->GetVelocity(0) * rotor_velocity_slowdown_sim_;
    msg->angular_velocities[m->first] = motor_rot_vel;
    joint_state_msg->name[m->first] = m->second->GetName();
    joint_state_msg->position[m->first] = m->second->GetAngle(0).Radian();
  }
  joint_state_msg->header.stamp.sec = now.sec;
  joint_state_msg->header.stamp.nsec = now.nsec;
  joint_state_msg->header.frame_id = frame_id_;
  msg->header.stamp.sec = now.sec;
  msg->header.stamp.nsec = now.nsec;
  msg->header.frame_id = frame_id_;
  // TODO(burrimi): Add a flag to publish rotor joint state.
  //joint_state_pub_.publish(joint_state_msg);
  motor_pub_.publish(msg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMultirotorBasePlugin);
}

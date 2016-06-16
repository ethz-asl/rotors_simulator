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

#include "rotors_gazebo_plugins/gazebo_motor_model.h"

namespace gazebo {

GazeboMotorModel::~GazeboMotorModel() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboMotorModel::InitializeParams() {}

void GazeboMotorModel::Publish() {
  turning_velocity_msg_.data = motor_rot_vel_;// joint_->GetVelocity(0);
  mav_msgs::MotorStatus msg;
  msg.header.stamp.sec = (world_->GetSimTime()).sec;
  msg.header.stamp.nsec = (world_->GetSimTime()).nsec;
  msg.speed_command = ref_motor_rot_vel_;
  msg.speed_measurement = std::fabs(motor_rot_vel_*rotor_velocity_slowdown_sim_);

  msg.wind_velocity_W.x = wind_speed_W_.x;
  msg.wind_velocity_W.y = wind_speed_W_.y;
  msg.wind_velocity_W.z = wind_speed_W_.z;

  msg.velocity_W.x = velocity_current_W_.x;
  msg.velocity_W.y = velocity_current_W_.y;
  msg.velocity_W.z = velocity_current_W_.z;

  msg.force_B.x = total_force_B_.x;
  msg.force_B.y = total_force_B_.y;
  msg.force_B.z = total_force_B_.z;

  msg.torque_B.x = total_torque_B_.x;
  msg.torque_B.y = total_torque_B_.y;
  msg.torque_B.z = total_torque_B_.z;

  motor_velocity_pub_.publish(msg);
}

void GazeboMotorModel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model_ = _model;
  world_ = model_->GetWorld();

  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("jointName"))
    joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a jointName, where the rotor is attached.\n";
  // Get the pointer to the joint.
  joint_ = model_->GetJoint(joint_name_);
  if (joint_ == NULL)
    gzthrow("[gazebo_motor_model] Couldn't find specified joint \"" << joint_name_ << "\".");


  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a linkName of the rotor.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_motor_model] Couldn't find specified link \"" << link_name_ << "\".");


  if (_sdf->HasElement("motorNumber"))
    motor_number_ = _sdf->GetElement("motorNumber")->Get<int>();
  else
    gzerr << "[gazebo_motor_model] Please specify a motorNumber.\n";

  if (_sdf->HasElement("turningDirection")) {
    std::string turning_direction = _sdf->GetElement("turningDirection")->Get<std::string>();
    if (turning_direction == "cw")
      turning_direction_ = turning_direction::CW;
    else if (turning_direction == "ccw")
      turning_direction_ = turning_direction::CCW;
    else
      gzerr << "[gazebo_motor_model] Please only use 'cw' or 'ccw' as turningDirection.\n";
  }
  else
    gzerr << "[gazebo_motor_model] Please specify a turning direction ('cw' or 'ccw').\n";

  getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic_, command_sub_topic_);
  getSdfParam<std::string>(_sdf, "windSpeedSubTopic", wind_speed_sub_topic_, wind_speed_sub_topic_);
  getSdfParam<std::string>(_sdf, "motorSpeedPubTopic", motor_speed_pub_topic_,
                           motor_speed_pub_topic_);

  getSdfParam<double>(_sdf, "rotorDragCoefficient", rotor_drag_coefficient_, rotor_drag_coefficient_);
  getSdfParam<double>(_sdf, "rollingMomentCoefficient", rolling_moment_coefficient_,
                      rolling_moment_coefficient_);
  getSdfParam<double>(_sdf, "maxRotVelocity", max_rot_velocity_, max_rot_velocity_);
  getSdfParam<double>(_sdf, "motorConstant", motor_constant_, motor_constant_);
  getSdfParam<double>(_sdf, "momentConstant", moment_constant_, moment_constant_);

  getSdfParam<double>(_sdf, "timeConstantUp", time_constant_up_, time_constant_up_);
  getSdfParam<double>(_sdf, "timeConstantDown", time_constant_down_, time_constant_down_);
  getSdfParam<double>(_sdf, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_, 10);

  // Set the maximumForce on the joint. This is deprecated from V5 on, and the joint won't move.
#if GAZEBO_MAJOR_VERSION < 5
  joint_->SetMaxForce(0, max_force_);
#endif
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboMotorModel::OnUpdate, this, _1));

  command_sub_ = node_handle_->subscribe(command_sub_topic_, 1, &GazeboMotorModel::VelocityCallback, this);
  wind_speed_sub_ = node_handle_->subscribe(wind_speed_sub_topic_, 1, &GazeboMotorModel::WindSpeedCallback, this);
  motor_velocity_pub_ = node_handle_->advertise<mav_msgs::MotorStatus>(motor_speed_pub_topic_, 10);

  wind_speed_W_ = math::Vector3::Zero;

  // Create the first order filter.
  rotor_velocity_filter_.reset(new FirstOrderFilter<double>(time_constant_up_, time_constant_down_, ref_motor_rot_vel_));
}

// This gets called by the world update start event.
void GazeboMotorModel::OnUpdate(const common::UpdateInfo& _info) {
  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();
  UpdateForcesAndMoments();
  Publish();
}

void GazeboMotorModel::VelocityCallback(const mav_msgs::ActuatorsConstPtr& rot_velocities) {
  ROS_ASSERT_MSG(rot_velocities->angular_velocities.size() > motor_number_,
                 "You tried to access index %d of the MotorSpeed message array which is of size %d.",
                 motor_number_, rot_velocities->angular_velocities.size());
  ref_motor_rot_vel_ = std::min(rot_velocities->angular_velocities[motor_number_], static_cast<double>(max_rot_velocity_));
  ref_timestamp_ = rot_velocities->header.stamp;
}

void GazeboMotorModel::WindSpeedCallback(const rotors_comm::WindSpeedConstPtr& wind_speed) {
  // TODO(burrimi): Transform velocity to world frame if frame_id is set to something else.
  wind_speed_W_.x = wind_speed->velocity.x;
  wind_speed_W_.y = wind_speed->velocity.y;
  wind_speed_W_.z = wind_speed->velocity.z;
}

void GazeboMotorModel::UpdateForcesAndMoments() {
  //motor_rot_vel_ = joint_->GetVelocity(0);
  if (motor_rot_vel_ / (2 * M_PI) > 1 / (2 * sampling_time_)) {
    gzerr << "Aliasing on motor [" << motor_number_ << "] might occur. Consider making smaller simulation time steps or raising the rotor_velocity_slowdown_sim_ param.\n";
  }

  velocity_current_W_ = link_->GetWorldLinearVel();
  math::Vector3 joint_axis = joint_->GetGlobalAxis(0);

  double real_motor_velocity = motor_rot_vel_ * rotor_velocity_slowdown_sim_;
  double thrust_force = real_motor_velocity * real_motor_velocity * motor_constant_;
  // Apply a force to the link.

  math::Vector3 thrust_force_B = math::Vector3(0, 0, thrust_force);

  // Forces from Philppe Martin's and Erwan Salaün's
  // 2010 IEEE Conference on Robotics and Automation paper
  // The True Role of Accelerometer Feedback in Quadrotor Control
  // - \omega * \lambda_1 * V_A^{\perp}
  math::Vector3 relative_wind_velocity_W = velocity_current_W_ - wind_speed_W_;
  math::Vector3 body_velocity_perpendicular = relative_wind_velocity_W - (relative_wind_velocity_W.Dot(joint_axis) * joint_axis);
  math::Vector3 air_drag = -std::abs(real_motor_velocity) * rotor_drag_coefficient_ * body_velocity_perpendicular;
  // Apply air_drag to link.
 // link_->AddForce(air_drag);

  math::Pose T_W_B = link_->GetWorldPose(); //TODO(burrimi): Check tf.
  math::Quaternion q_W_B = T_W_B.rot;

  math::Vector3 relative_velocity_W = velocity_current_W_ - wind_speed_W_;
  math::Vector3 relative_velocity_B = q_W_B.RotateVectorReverse(relative_velocity_W);

  math::Vector3 projected_velocity_B(relative_velocity_B[0],relative_velocity_B[1],0.0);
  math::Vector3 air_drag_B = -thrust_force * rotor_drag_coefficient_ * projected_velocity_B;

  total_force_B_ = air_drag_B + thrust_force_B;

  math::Vector3 diff = air_drag_B - q_W_B.RotateVectorReverse(air_drag);

  link_->AddRelativeForce(total_force_B_);


  // Moments
  // Getting the parent link, such that the resulting torques can be applied to it.
  physics::Link_V parent_links = link_->GetParentJointsLinks();
  // The tansformation from the parent_link to the link_.
  math::Pose pose_difference = link_->GetWorldCoGPose() - parent_links.at(0)->GetWorldCoGPose();
  math::Vector3 drag_torque(0, 0, -turning_direction_ * moment_constant_ * real_motor_velocity * real_motor_velocity);

  total_torque_B_ = drag_torque;

  // Transforming the drag torque into the parent frame to handle arbitrary rotor orientations.
  math::Vector3 total_torque_parent_frame = pose_difference.rot.RotateVector(total_torque_B_);
  parent_links.at(0)->AddRelativeTorque(total_torque_parent_frame);

//  math::Vector3 rolling_moment;
  // - \omega * \mu_1 * V_A^{\perp}
//  rolling_moment = -std::abs(real_motor_velocity) * rolling_moment_coefficient_ * body_velocity_perpendicular;
//  parent_links.at(0)->AddTorque(rolling_moment);
  // Apply the filter on the motor's velocity.
  double motor_rot_vel = rotor_velocity_filter_->updateFilter(ref_motor_rot_vel_, sampling_time_);
  joint_->SetVelocity(0, turning_direction_ * ref_motor_rot_vel_ / rotor_velocity_slowdown_sim_);
  motor_rot_vel_ = turning_direction_ * motor_rot_vel / rotor_velocity_slowdown_sim_;

}

GZ_REGISTER_MODEL_PLUGIN(GazeboMotorModel);
}

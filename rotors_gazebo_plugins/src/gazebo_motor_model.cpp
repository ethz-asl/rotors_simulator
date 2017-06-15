/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.M
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "rotors_gazebo_plugins/gazebo_motor_model.h"

#include "CommandMotorSpeed.pb.h"
#include "ConnectGazeboToRosTopic.pb.h"
#include "ConnectRosToGazeboTopic.pb.h"

namespace gazebo {

GazeboMotorModel::~GazeboMotorModel() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
}

void GazeboMotorModel::InitializeParams() {}

void GazeboMotorModel::Publish() {
  turning_velocity_msg_.set_data(joint_->GetVelocity(0));

  motor_velocity_pub_->Publish(turning_velocity_msg_);
}

void GazeboMotorModel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  model_ = _model;

  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";

  node_handle_ = gazebo::transport::NodePtr(new transport::Node());

  // Initialise with default namespace (typically /gazebo/default/)
  node_handle_->Init();

  if (_sdf->HasElement("jointName"))
    joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a jointName, where the rotor "
             "is attached.\n";

  // Get the pointer to the joint.
  joint_ = model_->GetJoint(joint_name_);
  if (joint_ == NULL)
    gzthrow("[gazebo_motor_model] Couldn't find specified joint \""
            << joint_name_ << "\".");

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a linkName of the rotor.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_motor_model] Couldn't find specified link \"" << link_name_
                                                                   << "\".");

  if (_sdf->HasElement("motorNumber"))
    motor_number_ = _sdf->GetElement("motorNumber")->Get<int>();
  else
    gzerr << "[gazebo_motor_model] Please specify a motorNumber.\n";

  if (_sdf->HasElement("turningDirection")) {
    std::string turning_direction =
        _sdf->GetElement("turningDirection")->Get<std::string>();
    if (turning_direction == "cw")
      turning_direction_ = turning_direction::CW;
    else if (turning_direction == "ccw")
      turning_direction_ = turning_direction::CCW;
    else
      gzerr << "[gazebo_motor_model] Please only use 'cw' or 'ccw' as "
               "turningDirection.\n";
  } else
    gzerr << "[gazebo_motor_model] Please specify a turning direction ('cw' or "
             "'ccw').\n";

  getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic_,
                           command_sub_topic_);
  getSdfParam<std::string>(_sdf, "windSpeedSubTopic", wind_speed_sub_topic_,
                           wind_speed_sub_topic_);
  getSdfParam<std::string>(_sdf, "motorSpeedPubTopic", motor_speed_pub_topic_,
                           motor_speed_pub_topic_);

  getSdfParam<double>(_sdf, "rotorDragCoefficient", rotor_drag_coefficient_,
                      rotor_drag_coefficient_);
  getSdfParam<double>(_sdf, "rollingMomentCoefficient",
                      rolling_moment_coefficient_, rolling_moment_coefficient_);
  getSdfParam<double>(_sdf, "maxRotVelocity", max_rot_velocity_,
                      max_rot_velocity_);
  getSdfParam<double>(_sdf, "motorConstant", motor_constant_, motor_constant_);
  getSdfParam<double>(_sdf, "momentConstant", moment_constant_,
                      moment_constant_);

  getSdfParam<double>(_sdf, "timeConstantUp", time_constant_up_,
                      time_constant_up_);
  getSdfParam<double>(_sdf, "timeConstantDown", time_constant_down_,
                      time_constant_down_);
  getSdfParam<double>(_sdf, "rotorVelocitySlowdownSim",
                      rotor_velocity_slowdown_sim_, 10);

// Set the maximumForce on the joint. This is deprecated from V5 on, and the
// joint won't move.
#if GAZEBO_MAJOR_VERSION < 5
  joint_->SetMaxForce(0, max_force_);
#endif

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboMotorModel::OnUpdate, this, _1));

  // Create the first order filter.
  rotor_velocity_filter_.reset(new FirstOrderFilter<double>(
      time_constant_up_, time_constant_down_, ref_motor_rot_vel_));
}

// This gets called by the world update start event.
void GazeboMotorModel::OnUpdate(const common::UpdateInfo& _info) {
  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();
  UpdateForcesAndMoments();
  Publish();
}

void GazeboMotorModel::CreatePubsAndSubs() {
  gzdbg << __PRETTY_FUNCTION__ << " called." << std::endl;

  // Create temporary "ConnectGazeboToRosTopic" publisher and message
  gazebo::transport::PublisherPtr gz_connect_gazebo_to_ros_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
          "~/" + kConnectGazeboToRosSubtopic, 1);
  gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;

  // Create temporary "ConnectRosToGazeboTopic" publisher and message
  gazebo::transport::PublisherPtr gz_connect_ros_to_gazebo_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectRosToGazeboTopic>(
          "~/" + kConnectRosToGazeboSubtopic, 1);
  gz_std_msgs::ConnectRosToGazeboTopic connect_ros_to_gazebo_topic_msg;

  // ============================================ //
  //  ACTUAL MOTOR SPEED MSG SETUP (GAZEBO->ROS)  //
  // ============================================ //

  motor_velocity_pub_ = node_handle_->Advertise<gz_std_msgs::Float32>(
      "~/" + namespace_ + "/" + motor_speed_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   motor_speed_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                motor_speed_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::FLOAT_32);
  gz_connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                              true);

  // ============================================ //
  // = CONTROL VELOCITY MSG SETUP (ROS->GAZEBO) = //
  // ============================================ //

  command_sub_ =
      node_handle_->Subscribe("~/" + namespace_ + "/" + command_sub_topic_,
                              &GazeboMotorModel::ControlVelocityCallback, this);

  connect_ros_to_gazebo_topic_msg.set_ros_topic(namespace_ + "/" +
                                                command_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   command_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_msgtype(
      gz_std_msgs::ConnectRosToGazeboTopic::COMMAND_MOTOR_SPEED);
  gz_connect_ros_to_gazebo_topic_pub->Publish(connect_ros_to_gazebo_topic_msg,
                                              true);

  // ============================================ //
  // ==== WIND SPEED MSG SETUP (ROS->GAZEBO) ==== //
  // ============================================ //

  /// TODO(gbmhunter): Do we need this? There is a separate Gazebo wind plugin.
  wind_speed_sub_ =
      node_handle_->Subscribe("~/" + namespace_ + "/" + wind_speed_sub_topic_,
                              &GazeboMotorModel::WindSpeedCallback, this);

  connect_ros_to_gazebo_topic_msg.set_ros_topic(namespace_ + "/" +
                                                wind_speed_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   wind_speed_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_msgtype(
      gz_std_msgs::ConnectRosToGazeboTopic::WIND_SPEED);
  gz_connect_ros_to_gazebo_topic_pub->Publish(connect_ros_to_gazebo_topic_msg,
                                              true);
}

void GazeboMotorModel::ControlVelocityCallback(
    GzCommandMotorSpeedMsgPtr& command_motor_speed_msg) {
  if (kPrintOnMsgCallback) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (motor_number_ > command_motor_speed_msg->motor_speed_size() - 1) {
    gzerr << "You tried to access index " << motor_number_
          << " of the MotorSpeed message array which is of size "
          << command_motor_speed_msg->motor_speed_size();
  }

  ref_motor_rot_vel_ = std::min(
      static_cast<double>(command_motor_speed_msg->motor_speed(motor_number_)),
      static_cast<double>(max_rot_velocity_));
}

void GazeboMotorModel::WindSpeedCallback(GzWindSpeedMsgPtr& wind_speed_msg) {
  if (kPrintOnMsgCallback) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  // TODO(burrimi): Transform velocity to world frame if frame_id is set to
  // something else.
  wind_speed_W_.x = wind_speed_msg->velocity().x();
  wind_speed_W_.y = wind_speed_msg->velocity().y();
  wind_speed_W_.z = wind_speed_msg->velocity().z();
}

void GazeboMotorModel::UpdateForcesAndMoments() {
  motor_rot_vel_ = joint_->GetVelocity(0);
  if (motor_rot_vel_ / (2 * M_PI) > 1 / (2 * sampling_time_)) {
    gzerr << "Aliasing on motor [" << motor_number_
          << "] might occur. Consider making smaller simulation time steps or "
             "raising the rotor_velocity_slowdown_sim_ param.\n";
  }
  double real_motor_velocity = motor_rot_vel_ * rotor_velocity_slowdown_sim_;
  double force = real_motor_velocity * real_motor_velocity * motor_constant_;

// TODO(ff): remove this?
// Code from sitl_gazebo version of GazeboMotorModel.
// Not active as model is imprecise, and does not take
// into account the direction of the wind (e.g. is it moving
// in the direction of propulsion, against?)
#if 0
  // scale down force linearly with forward speed
  // XXX this has to be modelled better
  math::Vector3 body_velocity = link_->GetWorldLinearVel();
  double vel = body_velocity.GetLength();
  double scalar = 1 - vel / 25.0; // at 50 m/s the rotor will not produce any force anymore
  scalar = math::clamp(scalar, 0.0, 1.0);
  // Apply a force to the link.
  link_->AddRelativeForce(math::Vector3(0, 0, force * scalar));
#endif

  // Apply a force to the link.
  link_->AddRelativeForce(math::Vector3(0, 0, force));

  // Forces from Philppe Martin's and Erwan Salaün's
  // 2010 IEEE Conference on Robotics and Automation paper
  // The True Role of Accelerometer Feedback in Quadrotor Control
  // - \omega * \lambda_1 * V_A^{\perp}
  math::Vector3 joint_axis = joint_->GetGlobalAxis(0);
  math::Vector3 body_velocity_W = link_->GetWorldLinearVel();
  math::Vector3 relative_wind_velocity_W = body_velocity_W - wind_speed_W_;
  math::Vector3 body_velocity_perpendicular =
      relative_wind_velocity_W -
      (relative_wind_velocity_W.Dot(joint_axis) * joint_axis);
  math::Vector3 air_drag = -std::abs(real_motor_velocity) *
                           rotor_drag_coefficient_ *
                           body_velocity_perpendicular;
  // Apply air_drag to link.
  link_->AddForce(air_drag);
  // Moments
  // Getting the parent link, such that the resulting torques can be applied to
  // it.
  physics::Link_V parent_links = link_->GetParentJointsLinks();
  // The tansformation from the parent_link to the link_.
  math::Pose pose_difference =
      link_->GetWorldCoGPose() - parent_links.at(0)->GetWorldCoGPose();
  math::Vector3 drag_torque(0, 0,
                            -turning_direction_ * force * moment_constant_);
  // Transforming the drag torque into the parent frame to handle arbitrary
  // rotor orientations.
  math::Vector3 drag_torque_parent_frame =
      pose_difference.rot.RotateVector(drag_torque);
  parent_links.at(0)->AddRelativeTorque(drag_torque_parent_frame);

  math::Vector3 rolling_moment;
  // - \omega * \mu_1 * V_A^{\perp}
  rolling_moment = -std::abs(real_motor_velocity) *
                   rolling_moment_coefficient_ * body_velocity_perpendicular;
  parent_links.at(0)->AddTorque(rolling_moment);
  // Apply the filter on the motor's velocity.
  double ref_motor_rot_vel;
  ref_motor_rot_vel =
      rotor_velocity_filter_->updateFilter(ref_motor_rot_vel_, sampling_time_);

  // Make sure max force is set, as it may be reset to 0 by a world reset any
  // time. (This cannot be done during Reset() because the change will be undone
  // by the Joint's reset function afterwards.)
  #if GAZEBO_MAJOR_VERSION < 5
    joint_->SetMaxForce(0, max_force_);
  #endif
  joint_->SetVelocity(
      0, turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMotorModel);
}

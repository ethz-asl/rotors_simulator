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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "rotors_gazebo_plugins/gazebo_controller_interface.h"

#include "ConnectGazeboToRosTopic.pb.h"
#include "ConnectRosToGazeboTopic.pb.h"

namespace gazebo {

GazeboControllerInterface::~GazeboControllerInterface() {
}

void GazeboControllerInterface::Load(physics::ModelPtr _model,
                                     sdf::ElementPtr _sdf) {
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  // Store the pointer to the model.
  model_ = _model;

  world_ = model_->GetWorld();

  namespace_.clear();

  //==============================================//
  //========== READ IN PARAMS FROM SDF ===========//
  //==============================================//

  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";
  }

  node_handle_ = gazebo::transport::NodePtr(new transport::Node());

  // Initialise with default namespace (typically /gazebo/default/)
  node_handle_->Init();

  getSdfParam<std::string>(_sdf, "commandMotorSpeedSubTopic",
                           command_motor_speed_sub_topic_,
                           command_motor_speed_sub_topic_);
  getSdfParam<std::string>(_sdf, "motorSpeedCommandPubTopic",
                           motor_velocity_reference_pub_topic_,
                           motor_velocity_reference_pub_topic_);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboControllerInterface::OnUpdate, this, _1));
}

void GazeboControllerInterface::OnUpdate(const common::UpdateInfo& /*_info*/) {
  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  if (!received_first_reference_) {
    return;
  }

  common::Time now = world_->GetSimTime();

  gz_sensor_msgs::Actuators turning_velocities_msg;

  for (int i = 0; i < input_reference_.size(); i++) {
    turning_velocities_msg.add_angular_velocities((double)input_reference_[i]);
  }

  turning_velocities_msg.mutable_header()->mutable_stamp()->set_sec(now.sec);
  turning_velocities_msg.mutable_header()->mutable_stamp()->set_nsec(now.nsec);

  // Frame ID is not used for this particular message
  turning_velocities_msg.mutable_header()->set_frame_id("");

  motor_velocity_reference_pub_->Publish(turning_velocities_msg);
}

void GazeboControllerInterface::CreatePubsAndSubs() {
  gzdbg << __FUNCTION__ << "() called." << std::endl;

  // Create temporary "ConnectGazeboToRosTopic" publisher and message
  gazebo::transport::PublisherPtr gz_connect_gazebo_to_ros_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
          "~/" + kConnectGazeboToRosSubtopic, 1);

  // Create temporary "ConnectRosToGazeboTopic" publisher and message
  gazebo::transport::PublisherPtr gz_connect_ros_to_gazebo_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectRosToGazeboTopic>(
          "~/" + kConnectRosToGazeboSubtopic, 1);

  // ============================================================ //
  // === ACTUATORS (MOTOR VELOCITY) MSG SETUP (GAZEBO -> ROS) === //
  // ============================================================ //

  // TODO This topic is missing the "~" and is in a completely different
  // namespace, fix?

  gzdbg << "GazeboControllerInterface creating Gazebo publisher on \""
        << namespace_ + "/" + motor_velocity_reference_pub_topic_ << "\"."
        << std::endl;
  motor_velocity_reference_pub_ =
      node_handle_->Advertise<gz_sensor_msgs::Actuators>(
          namespace_ + "/" + motor_velocity_reference_pub_topic_, 1);

  // Connect to ROS
  gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;
  connect_gazebo_to_ros_topic_msg.set_gazebo_topic(
      namespace_ + "/" + motor_velocity_reference_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(
      namespace_ + "/" + motor_velocity_reference_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::ACTUATORS);
  gz_connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                              true);

  // ================================================ //
  // ===== MOTOR SPEED MSG SETUP (ROS -> GAZEBO) ==== //
  // ================================================ //
  gzdbg << "Subscribing to Gazebo topic \""
        << "~/" + namespace_ + "/" + command_motor_speed_sub_topic_ << "\"."
        << std::endl;
  cmd_motor_sub_ = node_handle_->Subscribe(
      "~/" + namespace_ + "/" + command_motor_speed_sub_topic_,
      &GazeboControllerInterface::CommandMotorCallback, this);

  // Connect to ROS
  gz_std_msgs::ConnectRosToGazeboTopic connect_ros_to_gazebo_topic_msg;
  connect_ros_to_gazebo_topic_msg.set_ros_topic(namespace_ + "/" +
                                                command_motor_speed_sub_topic_);
  // connect_ros_to_gazebo_topic_msg.set_gazebo_namespace(namespace_);
  connect_ros_to_gazebo_topic_msg.set_gazebo_topic(
      "~/" + namespace_ + "/" + command_motor_speed_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_msgtype(
      gz_std_msgs::ConnectRosToGazeboTopic::ACTUATORS);
  gz_connect_ros_to_gazebo_topic_pub->Publish(connect_ros_to_gazebo_topic_msg,
                                              true);

  gzdbg << __FUNCTION__ << "() called." << std::endl;
}

void GazeboControllerInterface::CommandMotorCallback(
    GzActuatorsMsgPtr& actuators_msg) {
  if (kPrintOnMsgCallback) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  input_reference_.resize(actuators_msg->angular_velocities_size());
  for (int i = 0; i < actuators_msg->angular_velocities_size(); ++i) {
    input_reference_[i] = actuators_msg->angular_velocities(i);
  }

  // We have received a motor command reference (it may not be the first, but
  // this
  // does not matter)
  received_first_reference_ = true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboControllerInterface);
}

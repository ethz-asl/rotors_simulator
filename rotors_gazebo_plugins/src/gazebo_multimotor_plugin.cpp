#include "rotors_gazebo_plugins/gazebo_multimotor_plugin.h"

namespace gazebo {

void GazeboMultimotorPlugin::Load(physics::ModelPtr _model,
                                  sdf::ElementPtr _sdf) {
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
    gzerr << "[multimotor_plugin] Please specify a robotNamespace.\n";
  }

  node_handle_ = gazebo::transport::NodePtr(new transport::Node());

  // Initialise with default namespace (typically /gazebo/default/)
  node_handle_->Init();

  getSdfParam<std::string>(_sdf, "actuatorCommandSubTopic",
                           command_actuator_sub_topic_,
                           command_actuator_sub_topic_);
  getSdfParam<std::string>(_sdf, "actuatorStatePubTopic",
                           motor_state_pub_topic_, motor_state_pub_topic_);

  //=============================================//
  //========== LOAD ROTORS AND SERVOS ===========//
  //==============================================//
  std::string joint_name, link_name;

  // Add rotors.
  if (_sdf->HasElement("rotors")) {
    sdf::ElementPtr motors = _sdf->GetElement("rotors");
    sdf::ElementPtr motor = motors->GetElement("rotor");

    while (motor) {
      // Only load valid motors
      if (IsValidLink(motor) & IsValidJoint(motor)) {
        motors_.push_back(std::make_unique<MotorModelRotor>(model_, motor));
        gzdbg << "[gazebo_multimotor_plugin] Loaded rotor!\n";
      } else {
        gzdbg << "[gazebo_multimotor_plugin] Failed to load rotor!\n";
      }
      motor = motor->GetNextElement("rotor");
    }
  }

  // Add servos.
  if (_sdf->HasElement("servos")) {
    sdf::ElementPtr motors = _sdf->GetElement("servos");
    sdf::ElementPtr motor = motors->GetElement("servo");

    while (motor) {
      // Only load valid motors
      if (IsValidJoint(motor)) {
        motors_.push_back(std::make_unique<MotorModelServo>(model_, motor));
        gzdbg << "[gazebo_multimotor_plugin] Loaded servo!\n";
      } else {
        gzdbg << "[gazebo_multimotor_plugin] Failed to load servo!\n";
      }
      motor = motor->GetNextElement("servo");
    }
  }
  gzdbg << "[gazebo_multimotor_plugin] Loaded " << motors_.size()
        << " actuators.";

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboMultimotorPlugin::OnUpdate, this, _1));
}

void GazeboMultimotorPlugin::OnUpdate(const common::UpdateInfo&) {
  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  gz_sensor_msgs::Actuators actuator_state_msg;
  double position, velocity, effort;

  for (const auto& motor : motors_) {
    motor->UpdatePhysics();
    motor->GetActuatorState(&position, &velocity, &effort);
    actuator_state_msg.add_angles(position);
    actuator_state_msg.add_angular_velocities(velocity);
    actuator_state_msg.add_normalized(effort);
  }

  common::Time now = world_->SimTime();

  actuator_state_msg.mutable_header()->mutable_stamp()->set_sec(now.sec);
  actuator_state_msg.mutable_header()->mutable_stamp()->set_nsec(now.nsec);
  // Frame ID is not used for this particular message
  actuator_state_msg.mutable_header()->set_frame_id("");

  motor_state_pub_->Publish(actuator_state_msg);
}

void GazeboMultimotorPlugin::CreatePubsAndSubs() {
  // Create temporary "ConnectGazeboToRosTopic" publisher and message
  gazebo::transport::PublisherPtr gz_connect_gazebo_to_ros_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
          "~/" + kConnectGazeboToRosSubtopic, 1);

  // Create temporary "ConnectRosToGazeboTopic" publisher and message
  gazebo::transport::PublisherPtr gz_connect_ros_to_gazebo_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectRosToGazeboTopic>(
          "~/" + kConnectRosToGazeboSubtopic, 1);

  // ============================================================ //
  // ========= ACTUATOR STATE MSG SETUP (GAZEBO -> ROS) ========= //
  // ============================================================ //

  gzdbg << "GazeboMultimotorPlugin creating Gazebo publisher on \""
        << namespace_ + "/" + motor_state_pub_topic_ << "\"." << std::endl;
  motor_state_pub_ = node_handle_->Advertise<gz_sensor_msgs::Actuators>(
      namespace_ + "/" + motor_state_pub_topic_, 1);

  // Connect to ROS
  gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;
  connect_gazebo_to_ros_topic_msg.set_gazebo_topic(namespace_ + "/" +
                                                   motor_state_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                motor_state_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::ACTUATORS);
  gz_connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                              true);

  // ===================================================== //
  // ===== ACTUATOR COMMAND MSG SETUP (ROS -> GAZEBO) ==== //
  // ===================================================== //
  //
  gzdbg << "Subscribing to Gazebo topic \""
        << "~/" + namespace_ + "/" + command_actuator_sub_topic_ << "\"."
        << std::endl;
  cmd_motor_sub_ = node_handle_->Subscribe(
      "~/" + namespace_ + "/" + command_actuator_sub_topic_,
      &GazeboMultimotorPlugin::CommandMotorCallback, this);

  // Connect to ROS
  gz_std_msgs::ConnectRosToGazeboTopic connect_ros_to_gazebo_topic_msg;
  connect_ros_to_gazebo_topic_msg.set_ros_topic(namespace_ + "/" +
                                                command_actuator_sub_topic_);
  // connect_ros_to_gazebo_topic_msg.set_gazebo_namespace(namespace_);
  connect_ros_to_gazebo_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   command_actuator_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_msgtype(
      gz_std_msgs::ConnectRosToGazeboTopic::ACTUATORS);
  gz_connect_ros_to_gazebo_topic_pub->Publish(connect_ros_to_gazebo_topic_msg,
                                              true);
}

void GazeboMultimotorPlugin::CommandMotorCallback(
    GzActuatorsMsgPtr& actuators_msg) {
  int num_commands = actuators_msg->angular_velocities_size();
  if (num_commands != motors_.size()) {
    gzwarn << "Received " << std::to_string(num_commands)
           << " motor commands for " << std::to_string(motors_.size())
           << " motors.\n";
    num_commands =
        num_commands < motors_.size() ? num_commands : motors_.size();
  }

  for (int i = 0; i < num_commands; ++i) {
    motors_.at(i)->SetActuatorReference(actuators_msg->angles(i),
                                        actuators_msg->angular_velocities(i),
                                        actuators_msg->normalized(i));
  }

  received_first_reference_ = true;
}

bool GazeboMultimotorPlugin::IsValidLink(const sdf::ElementPtr motor) {
  // Check that link name is valid!
  std::string link_name;

  if (motor->HasElement("linkName")) {
    std::string link_name = motor->GetElement("linkName")->Get<std::string>();
    physics::LinkPtr link = model_->GetLink(link_name);
    if (link == NULL) {
      gzthrow("[multimotor_plugin] Couldn't find specified link \"" << link_name
                                                                    << "\".");
      return false;
    }
  } else {
    gzerr << "[multimotor_plugin] Please specify a linkName, where the "
             "rotor is attached.\n";
    return false;
  }
  return true;
}

bool GazeboMultimotorPlugin::IsValidJoint(const sdf::ElementPtr motor) {
  // Check that joint name is valid!
  std::string joint_name;
  if (motor->HasElement("jointName")) {
    std::string joint_name = motor->GetElement("jointName")->Get<std::string>();
    physics::JointPtr joint = model_->GetJoint(joint_name);
    gzdbg << "Loaded motor on joint " << joint_name << std::endl;
    if (joint == NULL) {
      gzthrow("[multimotor_plugin] Couldn't find specified joint \""
              << joint_name << "\".");
      return false;
    }
  } else {
    gzerr << "[multimotor_plugin] Please specify a jointName, where the "
             "rotor is attached.\n";
    return false;
  }
  return true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMultimotorPlugin);

}  // namespace gazebo

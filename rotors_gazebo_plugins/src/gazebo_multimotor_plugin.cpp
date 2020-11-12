#include "rotors_gazebo_plugins/gazebo_multimotor_plugin.h"

namespace gazebo {

void GazeboMultimotorPlugin::Publish() {
  common::Time now = world_->SimTime();
  gz_sensor_msgs::Actuators actuator_state_msg;

  for (int i = 0; i < motors_.size(); i++) {
    actuator_state_msg.add_angles((double)0);
    actuator_state_msg.add_angular_velocities((double)0);
  }

  actuator_state_msg.mutable_header()->mutable_stamp()->set_sec(now.sec);
  actuator_state_msg.mutable_header()->mutable_stamp()->set_nsec(now.nsec);
  // Frame ID is not used for this particular message
  actuator_state_msg.mutable_header()->set_frame_id("");

  motor_state_pub_->Publish(actuator_state_msg);
}

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

  // Add motors
  if (_sdf->HasElement("motors")) {
    sdf::ElementPtr motors = _sdf->GetElement("motors");
    sdf::ElementPtr motor = motors->GetElement("motor");

    std::string joint_name, link_name;

    //==================================//
    //===== Iterate through motors =====//
    //==================================//
    while (motor) {
      physics::JointPtr joint;
      physics::LinkPtr link;

      // Only load valid motors
      if (GetValidMotor(motor, joint, link)) {
        motors_.push_back(std::make_unique<SingleMotorModel>(motor, joint, link));
      }

      motor = motor->GetNextElement("motor");
    }
  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboMultimotorPlugin::OnUpdate, this, _1));
}

void GazeboMultimotorPlugin::OnUpdate(const common::UpdateInfo &) {
  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  if (!received_first_reference_) {
    return;
  }

  
  Publish();
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
    GzActuatorsMsgPtr &actuators_msg) {
  

  received_first_reference_ = true;
}

bool GazeboMultimotorPlugin::GetValidMotor(const sdf::ElementPtr motor, physics::JointPtr joint,
                                           physics::LinkPtr link) {
  // Check that joint name and link name are valid!
  std::string joint_name, link_name;
  if (motor->HasElement("jointName")) {
    std::string joint_name = motor->GetElement("jointName")->Get<std::string>();
    joint = model_->GetJoint(joint_name);
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

  if (motor->HasElement("linkName")) {
    std::string link_name = motor->GetElement("linkName")->Get<std::string>();
    link = model_->GetLink(link_name);
    if (link == NULL) {
      gzthrow("[multimotor_plugin] Couldn't find specified link \"" << link_name
                                                                    << "\".");
      return false;
    }
  } else {
    gzerr << "[multimotor_plugin] Please specify a linkName of the rotor.\n";
    return false;
  }
  return true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMultimotorPlugin);

}  // namespace gazebo

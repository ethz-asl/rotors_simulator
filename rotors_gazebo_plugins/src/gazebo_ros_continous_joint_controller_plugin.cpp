#include "rotors_gazebo_plugins/gazebo_ros_continous_joint_controller_plugin.h"
#include "rotors_gazebo_plugins/common.h"

namespace gazebo {

void ContinousJointControllerPlugin::Load(physics::ModelPtr _model,
                                          sdf::ElementPtr _sdf) {
  // Safety check
  if (_model->GetJointCount() == 0) {
    std::cerr << "Invalid joint count, joint controller plugin not loaded\n";
    return;
  }

  // Store the model pointer for convenience.
  this->model_ = _model;

  // Get the first joint. We are making an assumption about the model
  // having one joint that is the rotational joint.
  if (_sdf->HasElement("joint")) {
    std::string jointName = _sdf->Get<std::string>("joint");
    this->joint_ = _model->GetJoint(jointName);
  } else {
    this->joint_ = _model->GetJoints()[0];
    ROS_INFO("No joint name provided, therefore the first joint is used.");
  }

  double p_term, i_term, d_term;
  getSdfParam<double>(_sdf, "p_term", p_term, 0.1);
  getSdfParam<double>(_sdf, "i_term", i_term, 0.0);
  getSdfParam<double>(_sdf, "d_term", d_term, 0.0);

  // Setup a PID-controller.
  this->pid_ = common::PID(p_term, i_term, d_term);

  // Apply the P-controller to the joint.
  this->model_->GetJointController()->SetVelocityPID(
      this->joint_->GetScopedName(), this->pid_);

  // Check that the velocity element exists, then read the value
  if (_sdf->HasElement("spin_frequency_hz")) {
    spin_frequency_ = _sdf->Get<double>("spin_frequency_hz");
  } else {
    spin_frequency_ = 0.0;
  }

  SetVelocity(spin_frequency_);

  // Create the node
  this->node_ = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION < 8
  this->node_->Init(this->model_->GetWorld()->GetName());
#else
  this->node->Init(this->model->GetWorld()->Name());
#endif

  // Create a topic name
  std::string serviceName = "/" + this->model_->GetName() + "/frequency_cmd";

  // Initialize ros, if it has not already bee initialized.
  if (!ros::isInitialized()) {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
  }

  // Create our ROS node. This acts in a similar manner to
  // the Gazebo node
  this->ros_node_.reset(new ros::NodeHandle("gazebo_client"));

  // Create a service to alter the velocity
  ros_service_ = this->ros_node_->advertiseService(
      serviceName, &ContinousJointControllerPlugin::OnServiceCall, this);
  ROS_INFO("Joint controller plugin for continous joint <%s> loaded.", this->joint_->GetScopedName().c_str());
}

bool ContinousJointControllerPlugin::OnServiceCall(
    rotors_comm::SetFrequencyRequest &request,
    rotors_comm::SetFrequencyResponse &response) {
  this->SetVelocity(request.frequency);
  return true;
}

void ContinousJointControllerPlugin::SetVelocity(
    const double spin_frequency_hz) {
  // calculate update rate and set it
  this->model_->GetJointController()->SetVelocityTarget(
      this->joint_->GetScopedName(), 2.0 * M_PI * spin_frequency_hz);
}

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(ContinousJointControllerPlugin)
}

/*
 * Copyright 2017 Pavel Vechersky, ASL, ETH Zurich, Switzerland
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

// MODULE HEADER
#include "rotors_gazebo_plugins/gazebo_fw_dynamics_plugin.h"

#include "ConnectRosToGazeboTopic.pb.h"

namespace gazebo {

GazeboFwDynamicsPlugin::GazeboFwDynamicsPlugin()
    : ModelPlugin(),
      node_handle_(0),
      W_wind_speed_W_B_(0, 0, 0),
      delta_aileron_left_(0.0),
      delta_aileron_right_(0.0),
      delta_elevator_(0.0),
      delta_flap_(0.0),
      delta_rudder_(0.0),
      throttle_(0.0),
      pubs_and_subs_created_(false) {
}

GazeboFwDynamicsPlugin::~GazeboFwDynamicsPlugin() {
}

void GazeboFwDynamicsPlugin::Load(physics::ModelPtr _model,
                                  sdf::ElementPtr _sdf) {
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  gzdbg << "_model = " << _model->GetName() << std::endl;

  // Store the pointer to the model.
  model_ = _model;
  world_ = model_->GetWorld();

  namespace_.clear();

  // Get the robot namespace.
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_fw_dynamics_plugin] Please specify a robotNamespace.\n";

  // Create the node handle.
  node_handle_ = transport::NodePtr(new transport::Node());

  // Initialise with default namespace (typically /gazebo/default/).
  node_handle_->Init();

  // Get the link name.
  std::string link_name;
  if (_sdf->HasElement("linkName"))
    link_name = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_fw_dynamics_plugin] Please specify a linkName.\n";
  // Get the pointer to the link.
  link_ = model_->GetLink(link_name);
  if (link_ == NULL) {
    gzthrow("[gazebo_fw_dynamics_plugin] Couldn't find specified link \""
            << link_name << "\".");
  }

  // Get the path to fixed-wing aerodynamics parameters YAML file. If not
  // provided, default Techpod parameters are used.
  if (_sdf->HasElement("aeroParamsYAML")) {
    std::string aero_params_yaml =
        _sdf->GetElement("aeroParamsYAML")->Get<std::string>();

    aero_params_.LoadAeroParamsYAML(aero_params_yaml);
  } else {
    gzwarn << "[gazebo_fw_dynamics_plugin] No aerodynamic paramaters YAML file"
        << " specified, using default Techpod parameters.\n";
  }

  // Get the path to fixed-wing vehicle parameters YAML file. If not provided,
  // default Techpod parameters are used.
  if (_sdf->HasElement("vehicleParamsYAML")) {
    std::string vehicle_params_yaml =
        _sdf->GetElement("vehicleParamsYAML")->Get<std::string>();

    vehicle_params_.LoadVehicleParamsYAML(vehicle_params_yaml);
  } else {
    gzwarn << "[gazebo_fw_dynamics_plugin] No vehicle paramaters YAML file"
        << " specified, using default Techpod parameters.\n";
  }

  // Get the rest of the sdf parameters.
  getSdfParam<bool>(_sdf, "isInputJoystick", is_input_joystick_,
                    kDefaultIsInputJoystick);
  getSdfParam<std::string>(_sdf, "actuatorsSubTopic",
                           actuators_sub_topic_,
                           mav_msgs::default_topics::COMMAND_ACTUATORS);
  getSdfParam<std::string>(_sdf, "rollPitchYawrateThrustSubTopic",
                           roll_pitch_yawrate_thrust_sub_topic_,
                           mav_msgs::default_topics::
                               COMMAND_ROLL_PITCH_YAWRATE_THRUST);
  getSdfParam<std::string>(_sdf, "windSpeedSubTopic",
                           wind_speed_sub_topic_,
                           mav_msgs::default_topics::WIND_SPEED);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboFwDynamicsPlugin::OnUpdate, this, _1));
}

void GazeboFwDynamicsPlugin::OnUpdate(const common::UpdateInfo& _info) {
  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  UpdateForcesAndMoments();
}

void GazeboFwDynamicsPlugin::UpdateForcesAndMoments() {
  // Express the air speed and angular velocity in the body frame.
  // B denotes body frame and W world frame ... e.g., W_rot_W_B denotes
  // rotation of B wrt. W expressed in W.
  ignition::math::Quaterniond W_rot_W_B = link_->WorldPose().Rot();
  ignition::math::Vector3d B_air_speed_W_B = W_rot_W_B.RotateVectorReverse(
      link_->WorldLinearVel() - W_wind_speed_W_B_);
  ignition::math::Vector3d B_angular_velocity_W_B = link_->RelativeAngularVel();

  // Traditionally, fixed-wing aerodynamics use NED (North-East-Down) frame,
  // but since our model's body frame is in North-West-Up frame we rotate the
  // linear and angular velocities by 180 degrees around the X axis.
  double u = B_air_speed_W_B.X();
  double v = -B_air_speed_W_B.Y();
  double w = -B_air_speed_W_B.Z();

  double p = B_angular_velocity_W_B.X();
  double q = -B_angular_velocity_W_B.Y();
  double r = -B_angular_velocity_W_B.Z();

  // Compute the angle of attack (alpha) and the sideslip angle (beta). To
  // avoid division by zero, there is a minimum air speed threshold below which
  // alpha and beta are zero.
  double V = B_air_speed_W_B.Length();
  double beta = (V < kMinAirSpeedThresh) ? 0.0 : asin(v / V);
  double alpha = (u < kMinAirSpeedThresh) ? 0.0 : atan(w / u);

  // Bound the angle of attack.
  if (alpha > aero_params_.alpha_max)
    alpha = aero_params_.alpha_max;
  else if (alpha < aero_params_.alpha_min)
    alpha = aero_params_.alpha_min;

  // Pre-compute the common component in the force and moment calculations.
  const double q_bar_S = 0.5 * kAirDensity * V * V * vehicle_params_.wing_surface;

  // Combine some of the control surface deflections.
  double aileron_sum = delta_aileron_left_ + delta_aileron_right_;
  double aileron_diff = delta_aileron_left_ - delta_aileron_right_;
  double flap_sum = 2.0 * delta_flap_;
  double flap_diff = 0.0;

  // Compute the forces in the wind frame.
  const double drag = q_bar_S *
      (aero_params_.c_drag_alpha.dot(
           Eigen::Vector3d(1.0, alpha, alpha * alpha)) +
       aero_params_.c_drag_beta.dot(
           Eigen::Vector3d(0.0, beta, beta * beta)) +
       aero_params_.c_drag_delta_ail.dot(
           Eigen::Vector3d(0.0, aileron_sum, aileron_sum * aileron_sum)) +
       aero_params_.c_drag_delta_flp.dot(
           Eigen::Vector3d(0.0, flap_sum, flap_sum * flap_sum)));

  const double side_force = q_bar_S *
      (aero_params_.c_side_force_beta.dot(
           Eigen::Vector2d(0.0, beta)));

  const double lift = q_bar_S *
      (aero_params_.c_lift_alpha.dot(
           Eigen::Vector4d(1.0, alpha, alpha * alpha, alpha * alpha * alpha)) +
       aero_params_.c_lift_delta_ail.dot(
           Eigen::Vector2d(0.0, aileron_sum)) +
       aero_params_.c_lift_delta_flp.dot(
           Eigen::Vector2d(0.0, flap_sum)));

  const Eigen::Vector3d forces_Wind(-drag, side_force, -lift);

  // Non-dimensionalize the angular rates for inclusion in the computation of
  // moments. To avoid division by zero, there is a minimum air speed threshold
  // below which the values are zero.
  const double p_hat = (V < kMinAirSpeedThresh) ? 0.0 :
      p * vehicle_params_.wing_span / (2.0 * V);
  const double q_hat = (V < kMinAirSpeedThresh) ? 0.0 :
      q * vehicle_params_.chord_length / (2.0 * V);
  const double r_hat = (V < kMinAirSpeedThresh) ? 0.0 :
      r * vehicle_params_.wing_span / (2.0 * V);

  // Compute the moments in the wind frame.
  const double rolling_moment = q_bar_S * vehicle_params_.wing_span *
      (aero_params_.c_roll_moment_beta.dot(
           Eigen::Vector2d(0.0, beta)) +
       aero_params_.c_roll_moment_p.dot(
           Eigen::Vector2d(0.0, p_hat)) +
       aero_params_.c_roll_moment_r.dot(
           Eigen::Vector2d(0.0, r_hat)) +
       aero_params_.c_roll_moment_delta_ail.dot(
           Eigen::Vector2d(0.0, aileron_diff)) +
       aero_params_.c_roll_moment_delta_flp.dot(
           Eigen::Vector2d(0.0, flap_diff)));

  const double pitching_moment = q_bar_S * vehicle_params_.chord_length *
      (aero_params_.c_pitch_moment_alpha.dot(
           Eigen::Vector2d(1.0, alpha)) +
       aero_params_.c_pitch_moment_q.dot(
           Eigen::Vector2d(0.0, q_hat)) +
       aero_params_.c_pitch_moment_delta_elv.dot(
           Eigen::Vector2d(0.0, delta_elevator_)));

  const double yawing_moment = q_bar_S * vehicle_params_.wing_span *
      (aero_params_.c_yaw_moment_beta.dot(
           Eigen::Vector2d(0.0, beta)) +
       aero_params_.c_yaw_moment_r.dot(
           Eigen::Vector2d(0.0, r_hat)) +
       aero_params_.c_yaw_moment_delta_rud.dot(
           Eigen::Vector2d(0.0, delta_rudder_)));

  const Eigen::Vector3d moments_Wind(rolling_moment,
                                     pitching_moment,
                                     yawing_moment);

  // Compute the thrust force in the body frame.
  const double thrust = aero_params_.c_thrust.dot(
      Eigen::Vector3d(1.0, throttle_, throttle_ * throttle_));

  const Eigen::Vector3d force_thrust_B = thrust * Eigen::Vector3d(
      cos(vehicle_params_.thrust_inclination),
      0.0,
      sin(vehicle_params_.thrust_inclination));

  // Compute the transform between the body frame and the wind frame.
  double ca = cos(alpha);
  double sa = sin(alpha);
  double cb = cos(beta);
  double sb = sin(beta);

  Eigen::Matrix3d R_Wind_B;
  R_Wind_B << ca * cb, sb, sa * cb,
              -sb * ca, cb, -sa * sb,
              -sa, 0.0, ca;

  const Eigen::Matrix3d R_Wind_B_t = R_Wind_B.transpose();

  // Transform all the forces and moments into the body frame
  const Eigen::Vector3d forces_B = R_Wind_B_t * forces_Wind + force_thrust_B;
  const Eigen::Vector3d moments_B = R_Wind_B_t * moments_Wind;

  // Once again account for the difference between our body frame orientation
  // and the traditional aerodynamics frame.
  const ignition::math::Vector3d forces =
      ignition::math::Vector3d (forces_B[0], -forces_B[1], -forces_B[2]);
  const ignition::math::Vector3d moments =
      ignition::math::Vector3d (moments_B[0], -moments_B[1], -moments_B[2]);

  // Apply the calculated forced and moments to the main body link.
  link_->AddRelativeForce(forces);
  link_->AddRelativeTorque(moments);
}

double GazeboFwDynamicsPlugin::NormalizedInputToAngle(
    const ControlSurface& surface, double input) {
  return (surface.deflection_max + surface.deflection_min) * 0.5 +
      (surface.deflection_max - surface.deflection_min) * 0.5 * input;
}

void GazeboFwDynamicsPlugin::CreatePubsAndSubs() {
  gzdbg << __PRETTY_FUNCTION__ << " called." << std::endl;

  // Create temporary "ConnectRosToGazeboTopic" publisher and message
  gazebo::transport::PublisherPtr gz_connect_ros_to_gazebo_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectRosToGazeboTopic>(
          "~/" + kConnectRosToGazeboSubtopic, 1);
  gz_std_msgs::ConnectRosToGazeboTopic connect_ros_to_gazebo_topic_msg;

  // ============================================ //
  // ==== WIND SPEED MSG SETUP (ROS->GAZEBO) ==== //
  // ============================================ //

  // Wind speed subscriber (Gazebo).
  wind_speed_sub_ =
      node_handle_->Subscribe("~/" + namespace_ + "/" + wind_speed_sub_topic_,
                              &GazeboFwDynamicsPlugin::WindSpeedCallback,
                              this);

  connect_ros_to_gazebo_topic_msg.set_ros_topic(namespace_ + "/" +
                                                wind_speed_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   wind_speed_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_msgtype(
      gz_std_msgs::ConnectRosToGazeboTopic::WIND_SPEED);
  gz_connect_ros_to_gazebo_topic_pub->Publish(connect_ros_to_gazebo_topic_msg,
                                              true);

  // If we are using a joystick for control inputs then subscribe to the
  // RollPitchYawrateThrust msgs, otherwise subscribe to the Actuator msgs.
  if (is_input_joystick_) {
    // ========================================================= //
    // === ROLL_PITCH_YAWRATE_THRUST MSG SETUP (ROS->GAZEBO) === //
    // ========================================================= //

    roll_pitch_yawrate_thrust_sub_ =
        node_handle_->Subscribe("~/" + namespace_ + "/" +
            roll_pitch_yawrate_thrust_sub_topic_,
            &GazeboFwDynamicsPlugin::RollPitchYawrateThrustCallback, this);

    connect_ros_to_gazebo_topic_msg.set_ros_topic(namespace_ + "/" +
        roll_pitch_yawrate_thrust_sub_topic_);
    connect_ros_to_gazebo_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
        roll_pitch_yawrate_thrust_sub_topic_);
    connect_ros_to_gazebo_topic_msg.set_msgtype(
        gz_std_msgs::ConnectRosToGazeboTopic::ROLL_PITCH_YAWRATE_THRUST);
  } else {
    // ============================================ //
    // ===== ACTUATORS MSG SETUP (ROS->GAZEBO) ==== //
    // ============================================ //

    actuators_sub_ =
        node_handle_->Subscribe("~/" + namespace_ + "/" + actuators_sub_topic_,
                                &GazeboFwDynamicsPlugin::ActuatorsCallback,
                                this);

    connect_ros_to_gazebo_topic_msg.set_ros_topic(namespace_ + "/" +
                                                  actuators_sub_topic_);
    connect_ros_to_gazebo_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                     actuators_sub_topic_);
    connect_ros_to_gazebo_topic_msg.set_msgtype(
        gz_std_msgs::ConnectRosToGazeboTopic::ACTUATORS);
  }

  gz_connect_ros_to_gazebo_topic_pub->Publish(connect_ros_to_gazebo_topic_msg,
                                              true);
}

void GazeboFwDynamicsPlugin::ActuatorsCallback(
    GzActuatorsMsgPtr &actuators_msg) {
  if (kPrintOnMsgCallback) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  delta_aileron_left_ = NormalizedInputToAngle(vehicle_params_.aileron_left,
      actuators_msg->normalized(vehicle_params_.aileron_left.channel));
  delta_aileron_right_ = NormalizedInputToAngle(vehicle_params_.aileron_right,
      actuators_msg->normalized(vehicle_params_.aileron_right.channel));
  delta_elevator_ = NormalizedInputToAngle(vehicle_params_.elevator,
      actuators_msg->normalized(vehicle_params_.elevator.channel));
  delta_flap_ = NormalizedInputToAngle(vehicle_params_.flap,
      actuators_msg->normalized(vehicle_params_.flap.channel));
  delta_rudder_ = NormalizedInputToAngle(vehicle_params_.rudder,
      actuators_msg->normalized(vehicle_params_.rudder.channel));

  throttle_ = actuators_msg->normalized(vehicle_params_.throttle_channel);
}

void GazeboFwDynamicsPlugin::RollPitchYawrateThrustCallback(
    GzRollPitchYawrateThrustMsgPtr& roll_pitch_yawrate_thrust_msg) {
  if (kPrintOnMsgCallback) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  delta_aileron_left_ = NormalizedInputToAngle(vehicle_params_.aileron_left,
      roll_pitch_yawrate_thrust_msg->roll());
  delta_aileron_right_ = -NormalizedInputToAngle(vehicle_params_.aileron_right,
      roll_pitch_yawrate_thrust_msg->roll());
  delta_elevator_ = NormalizedInputToAngle(vehicle_params_.elevator,
      roll_pitch_yawrate_thrust_msg->pitch());
  delta_rudder_ = NormalizedInputToAngle(vehicle_params_.rudder,
      roll_pitch_yawrate_thrust_msg->yaw_rate());

  throttle_ = roll_pitch_yawrate_thrust_msg->thrust().x();
}

void GazeboFwDynamicsPlugin::WindSpeedCallback(
    GzWindSpeedMsgPtr& wind_speed_msg) {
  if (kPrintOnMsgCallback) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  W_wind_speed_W_B_.X() = wind_speed_msg->velocity().x();
  W_wind_speed_W_B_.Y() = wind_speed_msg->velocity().y();
  W_wind_speed_W_B_.Z() = wind_speed_msg->velocity().z();
}

GZ_REGISTER_MODEL_PLUGIN(GazeboFwDynamicsPlugin);

}  // namespace gazebo

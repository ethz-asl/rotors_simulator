#include <rotors_control/PID_attitude_controller.h>


namespace rotors_control{

PIDAttitudeController::PIDAttitudeController()
    : gravity_(9.81),
      mass_(1.6),
      initialized_params_(false){
}

PIDAttitudeController::~PIDAttitudeController() {
}



void PIDAttitudeController::InitializeParams() {

  //Get parameters from RosParam server
  std::string MAV_param_prefix = "MPC_position_controller/MAV_parameters/";
  std::string LowLevel_prefix = "Low_level_attitude_controller/";

  std::vector<double> temporary_Q, temporary_R, temporary_P, temporary_R_omega;
  std::vector<double> temporary_inertia_matrix, temporary_allocation_matrix;

   double rotor_force_constant;  //F_i = k_n * rotor_velocity_i^2
   double rotor_moment_constant;  // M_i = k_m * F_i
   double arm_length;



  if(!ros::param::get(LowLevel_prefix + "roll_gain",  roll_gain_))
     ROS_ERROR("roll_gain in MPC position controller is not loaded from ros parameter server");

  if(!ros::param::get(LowLevel_prefix + "pitch_gain",  pitch_gain_))
    ROS_ERROR("pitch_gain in MPC position controller is not loaded from ros parameter server");


  if(!ros::param::get(LowLevel_prefix + "p_gain",  p_gain_))
       ROS_ERROR("p_gain in MPC position controller is not loaded from ros parameter server");

  if(!ros::param::get(LowLevel_prefix + "q_gain",  q_gain_))
      ROS_ERROR("q_gain in MPC position controller is not loaded from ros parameter server");

  if(!ros::param::get(LowLevel_prefix + "r_gain",  r_gain_))
      ROS_ERROR("r_gain in MPC position controller is not loaded from ros parameter server");

  if (!ros::param::get(MAV_param_prefix + "mass", mass_))
      ROS_ERROR("mass in MPC position controller is not loaded from ros parameter server");

  if(!ros::param::get(MAV_param_prefix + "rotor_force_constant",  rotor_force_constant))
    ROS_ERROR("rotor_force_constant in MPC position controller is not loaded from ros parameter server");

  if(!ros::param::get(MAV_param_prefix + "rotor_moment_constant",  rotor_moment_constant))
    ROS_ERROR("rotor_moment_constant in MPC position controller is not loaded from ros parameter server");

  if(!ros::param::get(MAV_param_prefix + "arm_length",  arm_length))
      ROS_ERROR("arm_length in MPC position controller is not loaded from ros parameter server");

  if(!ros::param::get(MAV_param_prefix + "inertia",  temporary_inertia_matrix))
        ROS_ERROR("arm_length in MPC position controller is not loaded from ros parameter server");

  if(!ros::param::get(MAV_param_prefix + "allocation_matrix",  temporary_allocation_matrix))
        ROS_ERROR("arm_length in MPC position controller is not loaded from ros parameter server");


  Eigen::Map<Eigen::MatrixXd> allocation_matrix_map(temporary_allocation_matrix.data(), 4, 6);
  Eigen::Map<Eigen::MatrixXd> inertia_matrix_map(temporary_inertia_matrix.data(), 3, 3);

  Eigen::MatrixXd allocation_matrix(4,6);
  allocation_matrix = allocation_matrix_map;


  inertia_ = inertia_matrix_map;


  Eigen::Matrix4d I;
  I.setZero();
  I.block<3, 3>(0, 0) = inertia_;
  I(3, 3) = 1;

  Eigen::Matrix4d K;
  K.setZero();
  K(0, 0) = arm_length * rotor_force_constant;
  K(1, 1) = arm_length * rotor_force_constant;
  K(2, 2) = rotor_force_constant * rotor_moment_constant;
  K(3, 3) = rotor_force_constant;

  angular_acc_to_rotor_velocities_ = allocation_matrix.transpose()
        * (allocation_matrix * allocation_matrix.transpose()).inverse() * K.inverse()*I;

  attitude_thrust_reference_.setZero();

  initialized_params_ = true;
}


void PIDAttitudeController::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}



void PIDAttitudeController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const {
  assert(rotor_velocities);
  assert(initialized_params_);


 // printf("MPC_1\n");

  rotor_velocities->resize(6);




  Eigen::Vector3d angular_acceleration;
  ComputeDesiredAngularAcc(&angular_acceleration);

  Eigen::Vector4d angular_acceleration_thrust;
  angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
  angular_acceleration_thrust(3) = attitude_thrust_reference_(3);

  Eigen::Vector4d cross_term;
  cross_term << inertia_.inverse() * (odometry_.angular_velocity.cross(inertia_*odometry_.angular_velocity)), 0;


  *rotor_velocities = angular_acc_to_rotor_velocities_ * (angular_acceleration_thrust  - cross_term);
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
}



void PIDAttitudeController::ComputeDesiredAngularAcc(Eigen::Vector3d* angular_acc) const {
  assert(angular_acc);


  Eigen::Vector3d current_rpy ;
  quat2rpy(odometry_.orientation, &current_rpy);

  double error_roll = attitude_thrust_reference_(0) - current_rpy(0);
  double error_pitch = attitude_thrust_reference_(1) - current_rpy(1);

  //desired omega = [0;0;0]
  double error_p = 0 - odometry_.angular_velocity(0);
  double error_q = 0 - odometry_.angular_velocity(1);
  double error_r = attitude_thrust_reference_(2) - odometry_.angular_velocity(2);


  *angular_acc << roll_gain_*error_roll + p_gain_*error_p,
                  pitch_gain_*error_pitch + q_gain_*error_q,
                  r_gain_*error_r;

}







void PIDAttitudeController::quat2rpy(Eigen::Quaternion<double> q, Eigen::Vector3d* rpy) const {
  assert(rpy);

  *rpy << atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y())),
          asin(2.0 * (q.w() * q.y() - q.z() * q.x())), atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
          1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

}

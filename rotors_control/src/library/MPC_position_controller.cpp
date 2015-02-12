#include <rotors_control/MPC_position_controller.h>




namespace rotors_control{


MPCPositionController::MPCPositionController()
    : gravity_(9.81),
      initialized_params_(false){
}

MPCPositionController::~MPCPositionController() {
}



void MPCPositionController::InitializeParams() {

  StateObserver.reset(new KFDisturbanceObserver());
  SteadyStateCalculator.reset(new SteadyStateCalculation());

  StateObserver->Initialize();
  SteadyStateCalculator->Initialize();


  //Get parameters from RosParam server

  std::string MPC_controller_prefix = "MPC_position_controller/controller/";
  std::string MAV_param_prefix = "MPC_position_controller/MAV_parameters/";

  std::vector<double> temporary_Q, temporary_R, temporary_P, temporary_R_omega;
  std::vector<double> temporary_inertia_matrix, temporary_allocation_matrix;

   double rotor_force_constant;  //F_i = k_n * rotor_velocity_i^2
   double rotor_moment_constant;  // M_i = k_m * F_i
   double arm_length;




  if(!ros::param::get(MPC_controller_prefix + "yaw_gain",  yaw_gain_))
    ROS_ERROR("yaw_gain in MPC position controller is not loaded from ros parameter server");

  if (!ros::param::get(MAV_param_prefix + "mass", mass_))
      ROS_ERROR("mass in MPC position controller is not loaded from ros parameter server");

  if (!ros::param::get(MPC_controller_prefix + "offset_free_controller", offset_free_controller_))
    ROS_ERROR("offset_free_controller in MPC position controller is not loaded from ros parameter server");

  if (!ros::param::get(MPC_controller_prefix + "use_KF_state", use_KF_state_))
    ROS_ERROR("use_KF_state in MPC position controller is not loaded from ros parameter server");

  if (!ros::param::get(MPC_controller_prefix + "n_state", state_size_))
    ROS_ERROR("state_size_ in MPC position controller is not loaded from ros parameter server");

  if (!ros::param::get(MPC_controller_prefix + "n_input", input_size_))
    ROS_ERROR("input_size_ in MPC position controller is not loaded from ros parameter server");

  if (!ros::param::get(MPC_controller_prefix + "n_disturbance", disturbance_size_))
    ROS_ERROR("disturbance_size_ in MPC position controller is not loaded from ros parameter server");

  if (!ros::param::get(MPC_controller_prefix + "n_measurement", measurement_size_))
    ROS_ERROR("measurement_size_ in MPC position controller is not loaded from ros parameter server");

  if (!ros::param::get(MPC_controller_prefix + "Q", temporary_Q))
    ROS_ERROR("Q in MPC position controller is not loaded from ros parameter server");

  if (!ros::param::get(MPC_controller_prefix + "R", temporary_R))
    ROS_ERROR("R in MPC position controller is not loaded from ros parameter server");

  if (!ros::param::get(MPC_controller_prefix + "R_omega", temporary_R_omega))
    ROS_ERROR("R_omega in MPC position controller is not loaded from ros parameter server");

  if (!ros::param::get(MPC_controller_prefix + "P", temporary_P))
    ROS_ERROR("P in MPC position controller is not loaded from ros parameter server");


  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;
  Eigen::MatrixXd R_omega;
  Eigen::MatrixXd P;

  Eigen::Map<Eigen::MatrixXd> Q_(temporary_Q.data(), state_size_, state_size_);
  Eigen::Map<Eigen::MatrixXd> R_(temporary_R.data(), input_size_, input_size_);
  Eigen::Map<Eigen::MatrixXd> R_omega_(temporary_R_omega.data(), input_size_, input_size_);
  Eigen::Map<Eigen::MatrixXd> P_(temporary_P.data(), state_size_, state_size_);

  Q = Q_;
  R = R_;
  R_omega = R_omega_;
  P = P_;

#ifdef UseForcesSolver
  cost_Hessian_ = Eigen::MatrixXd::Zero(state_size_ + disturbance_size_ + 2*input_size_,state_size_ + disturbance_size_ + 2*input_size_);
  cost_Hessian_.block(0,0,state_size_,state_size_) = 2*Q;
  cost_Hessian_.block(state_size_ + disturbance_size_ + input_size_,state_size_ + disturbance_size_ + input_size_,input_size_,input_size_) = 2.0*(R + R_omega);
  cost_Hessian_.block(state_size_ +disturbance_size_, state_size_ + disturbance_size_, input_size_, input_size_) = 2*R_omega;
  cost_Hessian_.block(state_size_ + disturbance_size_ + input_size_, state_size_ + disturbance_size_, input_size_, input_size_) = -2.0*R_omega;
  cost_Hessian_.block(state_size_ + disturbance_size_ , state_size_ + disturbance_size_ + input_size_, input_size_, input_size_) = -2.0*R_omega;

  std::cout << "H = " << cost_Hessian_ << std::endl;
  cost_Hessian_final_ = Eigen::MatrixXd::Zero(state_size_ + disturbance_size_ + input_size_,state_size_ + disturbance_size_ + input_size_);
  cost_Hessian_final_.block(0,0,state_size_,state_size_) = 2.0*P;

  std::cout << "H_N = " << cost_Hessian_final_ << std::endl;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(FORCES_params_.H_N), state_size_ + disturbance_size_ + input_size_,state_size_ + disturbance_size_ + input_size_)= cost_Hessian_final_;
  printf("1\n");
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(FORCES_params_.H), state_size_ + disturbance_size_ + 2*input_size_,state_size_ + disturbance_size_ + 2*input_size_) = cost_Hessian_;

#endif

#ifdef UseCVXGENSolver

  //CVXGEN initialization
  set_defaults();
  setup_indexing();

  //CVXGEN settings
  settings.verbose = 0;

  std::vector<double> temporary_A, temporary_B, temporary_Bd, temporary_umin, temporary_umax;
  if (!ros::param::get(MPC_controller_prefix + "A", temporary_A))
    ROS_ERROR("A in MPC position controller is not loaded from ros parameter server");

  if (!ros::param::get(MPC_controller_prefix + "B", temporary_B))
    ROS_ERROR("B in MPC position controller is not loaded from ros parameter server");

  if (!ros::param::get(MPC_controller_prefix + "Bd", temporary_Bd))
    ROS_ERROR("Bd in MPC position controller is not loaded from ros parameter server");

  if (!ros::param::get(MPC_controller_prefix + "umin", temporary_umin))
    ROS_ERROR("umin in MPC position controller is not loaded from ros parameter server");

  if (!ros::param::get(MPC_controller_prefix + "umax", temporary_umax))
    ROS_ERROR("umax in MPC position controller is not loaded from ros parameter server");

  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd Bd;

  Eigen::Map<Eigen::MatrixXd> A_map(temporary_A.data(), state_size_, state_size_);
  Eigen::Map<Eigen::MatrixXd> B_map(temporary_B.data(), state_size_, input_size_);
  Eigen::Map<Eigen::MatrixXd> Bd_map(temporary_Bd.data(), state_size_, disturbance_size_);

  A = A_map;
  B = B_map;
  Bd = Bd_map;

  std::cout << "CVX A = \n" << A << std::endl;
  std::cout << "CVX A = \n" << B << std::endl;
  std::cout << "CVX A = \n" << Bd << std::endl;


  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.A), state_size_, state_size_) = A;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.B), state_size_, input_size_) = B;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.Bd), state_size_, disturbance_size_) = Bd;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.Q), state_size_, state_size_) = Q;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.Q_final), state_size_, state_size_) = P;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.R), input_size_, input_size_) = R;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.R_omega), input_size_, input_size_) = R_omega;



  for (int i = 0; i < input_size_; i++) {
    params.u_max[i] = temporary_umax.at(i);
    params.u_min[i] = temporary_umin.at(i);
  }


#endif

  std::cout << "Q = \n" << Q << std::endl;
  std::cout << "R = \n" << R << std::endl;
  std::cout << "R_omega = \n " << R_omega << std::endl;
  std::cout << "P = \n" << P << std::endl;


  Eigen::Vector3d initial_rpy;
  quat2rpy(odometry_.orientation, &initial_rpy);

  StateObserver->Reset(odometry_.position, odometry_.velocity, initial_rpy, odometry_.angular_velocity, Eigen::Vector3d::Zero(),
                       Eigen::Vector3d::Zero());

  initialized_params_ = true;


  ROS_INFO("MPC position controller initialized correctly");
}


void MPCPositionController::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}


void MPCPositionController::SetCommandTrajectory(
    const mav_msgs::EigenCommandTrajectory& command_trajectory) {
  command_trajectory_ = command_trajectory;
}



void MPCPositionController::CalculateAttitudeThrust(Eigen::Vector4d *ref_attitude_thrust) const {
  assert(initialized_params_);
  assert(ref_attitude_thrust);
  ros::WallTime total_begin_time = ros::WallTime::now();
  //Declare variables
  static Eigen::Vector4d attitude_command;  //actual roll, pitch, yaw, thrust command
  static Eigen::Vector3d linearized_roll_pitch_thrust_cmd;
  static int counter = 0;
  static double SolveTime_avg = 0.0;
  Eigen::VectorXd reference(6);

  reference << command_trajectory_.position, command_trajectory_.velocity;

  std::cout << "linearized_roll_pitch_thrust_cmd  " << linearized_roll_pitch_thrust_cmd << std::endl;
  std::cout << "attitude_command  " << attitude_command << std::endl;

  //mav_msgs::ObserverState observer_state;

  Eigen::Matrix3d R_rot;
  Eigen::VectorXd KF_estimated_state;
  Eigen::Vector2d roll_pitch_inertial_frame;
  Eigen::VectorXd target_state(state_size_);
  Eigen::VectorXd target_input(input_size_);

  Eigen::VectorXd estimated_disturbnces(disturbance_size_);
  Eigen::VectorXd x_0(state_size_);

  double yaw_rate_cmd;
  double roll;
  double pitch;
  double yaw;

#ifdef UseForcesSolver
  Eigen::VectorXd z_ss(state_size_ + disturbance_size_ + 2*input_size_);
  Eigen::VectorXd z1(2*(state_size_ + disturbance_size_ + input_size_));
  Eigen::VectorXd f(state_size_ + disturbance_size_ + 2*input_size_);
  Eigen::VectorXd z_ss_N(state_size_ + disturbance_size_ + input_size_);
  Eigen::VectorXd f_N(state_size_ + disturbance_size_ + input_size_);

  FireFlyOffsetFreeMPC_output output;
  FireFlyOffsetFreeMPC_info info;

  static std::deque<Eigen::VectorXd> f_queue;

  if(f_queue.empty()){
	  z_ss << command_trajectory_.position, command_trajectory_.velocity , 0 , 0, Eigen::VectorXd::Zero(disturbance_size_ + 2*input_size_);
	  for(int i=0; i<19; i++) {
	    Eigen::VectorXd tmp_vector(state_size_ + disturbance_size_ + 2*input_size_);
	    tmp_vector = -cost_Hessian_.transpose() * z_ss;
	    f_queue.push_back(tmp_vector);
	  }
  }


#endif


#ifdef UseCVXGENSolver
  static  std::deque<Eigen::VectorXd> x_ss_queue;
  if(x_ss_queue.empty()){
	  Eigen::VectorXd x_ss(state_size_);
	  x_ss << command_trajectory_.position, command_trajectory_.velocity , 0, 0;
	  for (int i = 0; i < 20; i++) {
	    Eigen::VectorXd tmp_vector;
	    x_ss_queue.push_back(tmp_vector);
	    x_ss_queue[i].resize(state_size_);
	    x_ss_queue[i] = x_ss;
	  }
  }
#endif



  R_rot = odometry_.orientation.toRotationMatrix();




  StateObserver->FeedAttitudeCommand(attitude_command);
  StateObserver->FeedPositionMeasurement(odometry_.position);
  StateObserver->FeedVelocityMeasurement(odometry_.velocity );
  StateObserver->FeedRotationMatrix(R_rot);

  StateObserver->UpdateEstimator(ros::Time::now());

  StateObserver->GetEstimatedState(&KF_estimated_state);

  if (offset_free_controller_ == true) {
    estimated_disturbnces = KF_estimated_state.segment(12, disturbance_size_);
  } else {
    estimated_disturbnces.setZero(disturbance_size_);
  }

  if (use_KF_state_ == true) {

    roll = KF_estimated_state(6);
    pitch = KF_estimated_state(7);
    yaw = KF_estimated_state(8);

    roll_pitch_inertial_frame << -sin(yaw) * pitch + cos(yaw) * roll, cos(yaw) * pitch + sin(yaw) * roll;
    x_0 << KF_estimated_state.segment(0, 6), roll_pitch_inertial_frame;

  } else {
    Eigen::Vector3d current_roll_pitch_yaw;
    quat2rpy(odometry_.orientation, &current_roll_pitch_yaw);

    roll = current_roll_pitch_yaw(0);
    pitch = current_roll_pitch_yaw(1);
    yaw = current_roll_pitch_yaw(2);

    roll_pitch_inertial_frame << -sin(yaw) * pitch + cos(yaw) * roll, cos(yaw) * pitch + sin(yaw) * roll;
    x_0 << odometry_.position, odometry_.velocity, roll_pitch_inertial_frame;
  }




  SteadyStateCalculator->ComputeSteadyState(estimated_disturbnces, reference, &target_state, &target_input);

  std::cout << "target state = \n" << target_state << std::endl;
  std::cout << "target input = \n" << target_input << std::endl;
  std::cout << "x0 = \n" << x_0 << std::endl;

  counter++;

#ifdef UseForcesSolver
  //Solve using FORCES
  z_ss << target_state,Eigen::VectorXd::Zero(disturbance_size_ + input_size_),target_input;
  z_ss_N << target_state , Eigen::VectorXd::Zero(disturbance_size_ + input_size_);
  f = -cost_Hessian_.transpose() * z_ss;
  f_N = -cost_Hessian_final_.transpose()*z_ss_N;

  z1 << x_0 , estimated_disturbnces , linearized_roll_pitch_thrust_cmd ,Eigen::VectorXd::Zero(state_size_ + disturbance_size_ + input_size_,1);

  //push new element in the queue

  Eigen::VectorXd tmp_vector(state_size_ + disturbance_size_ + 2*input_size_);
  f_queue.push_back(tmp_vector);
  f_queue.back() = f;
  f_queue.pop_front();


  //For different prediction horizon, change this accordingly!
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_1), state_size_ + disturbance_size_ + 2*input_size_,1) =f_queue[0];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_2), state_size_ + disturbance_size_ + 2*input_size_,1) =f_queue[1];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_3), state_size_ + disturbance_size_ + 2*input_size_,1) =f_queue[2];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_4), state_size_ + disturbance_size_ + 2*input_size_,1) =f_queue[3];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_5), state_size_ + disturbance_size_ + 2*input_size_,1) =f_queue[4];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_6), state_size_ + disturbance_size_ + 2*input_size_,1) = f_queue[5];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_7), state_size_ + disturbance_size_ + 2*input_size_,1) = f_queue[6];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_8), state_size_ + disturbance_size_ + 2*input_size_,1) = f_queue[7];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_9), state_size_ + disturbance_size_ + 2*input_size_,1) = f_queue[8];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_10), state_size_ + disturbance_size_ + 2*input_size_,1)= f_queue[9];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_11), state_size_ + disturbance_size_ + 2*input_size_,1)= f_queue[10];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_12), state_size_ + disturbance_size_ + 2*input_size_,1)= f_queue[11];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_13), state_size_ + disturbance_size_ + 2*input_size_,1)= f_queue[12];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_14), state_size_ + disturbance_size_ + 2*input_size_,1)= f_queue[13];

  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_15), state_size_ + disturbance_size_ + 2*input_size_,1)= f_queue[14];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_16), state_size_ + disturbance_size_ + 2*input_size_,1)= f_queue[15];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_17), state_size_ + disturbance_size_ + 2*input_size_,1)= f_queue[16];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_18), state_size_ + disturbance_size_ + 2*input_size_,1)= f_queue[17];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_19), state_size_ + disturbance_size_ + 2*input_size_,1)= f_queue[18];


  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_N), state_size_ + disturbance_size_ + input_size_,1) = f_N;



  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.z1), 2*(state_size_ + disturbance_size_ + input_size_),1)= z1;



  FireFlyOffsetFreeMPC_solve(const_cast<FireFlyOffsetFreeMPC_params*>(&FORCES_params_),&output,&info);


  linearized_roll_pitch_thrust_cmd << output.output_1[0], output.output_1[1], output.output_1[2];

  SolveTime_avg += info.solvetime;
#endif

#ifdef UseCVXGENSolver
  //Solve using CVXGEN
printf("using CVXGEN\n");
  // KF_estimated_state.segment(0,8), KF_estimated_state.segment(12,3) , linearized_roll_pitch_thrust_cmd

  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(params.d), disturbance_size_, 1) = estimated_disturbnces;
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(params.u_prev), input_size_, 1) =
      linearized_roll_pitch_thrust_cmd;
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(params.x_0), state_size_, 1) = x_0;
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(params.u_ss), input_size_, 1) = target_input;



  //push new element in the queue
  Eigen::VectorXd tmp_vector(state_size_);
  x_ss_queue.push_back(tmp_vector);
  x_ss_queue.back() = target_state;
  x_ss_queue.pop_front();

  for (int i = 0; i < 20; i++){
    Eigen::Map<Eigen::VectorXd>(const_cast<double*>(params.x_ss[i]), state_size_, 1) = x_ss_queue[i];
  }

  tic();
  solve();
  SolveTime_avg += tocq();

  linearized_roll_pitch_thrust_cmd << vars.u_0[0], vars.u_0[1], vars.u_0[2];

#endif

  std::cout << "lin cmd = \n" << linearized_roll_pitch_thrust_cmd << std::endl;


  if (counter > 100) {
    printf("Avg_solve time = %.5f\n", SolveTime_avg / counter);
    SolveTime_avg = 0.0;
    counter = 0;
  }

  attitude_command(3) = (linearized_roll_pitch_thrust_cmd(2) + gravity_) / (cos(roll) * cos(pitch));
  double ux = linearized_roll_pitch_thrust_cmd(1) * (gravity_ / attitude_command(3));
  double uy = linearized_roll_pitch_thrust_cmd(0) * (gravity_ / attitude_command(3));

  attitude_command(0) = ux * sin(yaw) + uy * cos(yaw);
  attitude_command(1) = ux * cos(yaw) - uy * sin(yaw);
  attitude_command(2) = command_trajectory_.yaw;


  int n_rotations = (int)(yaw / (2.0*M_PI));
  yaw = yaw -  2.0*M_PI*n_rotations;

  if(yaw > M_PI)
	  yaw = M_PI - yaw;


  double yaw_error = attitude_command(2) - yaw;

  if(abs(yaw_error) > M_PI){
	  if(yaw_error > 0.0)
		  yaw_error = yaw_error - 2.0*M_PI;
	  else
		  yaw_error = yaw_error + 2.0*M_PI;
  }


  yaw_rate_cmd = yaw_gain_ * yaw_error;


  *ref_attitude_thrust << attitude_command(0), attitude_command(1) , yaw_rate_cmd, attitude_command(3) * mass_;

  double diff_time = (ros::WallTime::now() - total_begin_time).toSec();
  std::cout << "Controller loop time : " << diff_time << " sec" << std::endl;
}




void MPCPositionController::quat2rpy(Eigen::Quaternion<double> q, Eigen::Vector3d* rpy) const {
  assert(rpy);

  *rpy << atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y())),
          asin(2.0 * (q.w() * q.y() - q.z() * q.x())), atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
          1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}
}

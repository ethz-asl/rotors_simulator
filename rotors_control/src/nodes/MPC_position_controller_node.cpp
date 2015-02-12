#include <rotors_control/MPC_position_controller_node.h>



namespace rotors_control{

MPCPositionControllerNode::MPCPositionControllerNode()
    :initialized_params_(false),
     got_first_trajectory_command_(false){


  InitializeParams();
  MPC_position_controller_.InitializeParams();


  ros::NodeHandle nh(namespace_);

  cmd_trajectory_sub_ = nh.subscribe(command_trajectory_sub_topic_, 10,
                                       &MPCPositionControllerNode::CommandTrajectoryCallback, this);

  odometry_sub_ = nh.subscribe(odometry_sub_topic_, 10,
                                 &MPCPositionControllerNode::OdometryCallback, this);


  command_roll_pitch_yawrate_thrust_pub_ = nh.advertise<mav_msgs::CommandRollPitchYawrateThrust>(
      command_roll_pitch_yawrate_thrust_topic_, 10);


  initialized_params_ = true;

}

MPCPositionControllerNode::~MPCPositionControllerNode() { }




void MPCPositionControllerNode::InitializeParams(){

  ros::NodeHandle pnh("~");
  pnh.param<std::string>("robotNamespace", namespace_, kDefaultNamespace);
  pnh.param<std::string>("commandTrajectorySubTopic", command_trajectory_sub_topic_, kDefaultCommandTrajectoryTopic);
  pnh.param<std::string>("odometrySubTopic", odometry_sub_topic_, kDefaultOdometrySubTopic);
  pnh.param<std::string>("motorVelocityCommandPubTopic", command_roll_pitch_yawrate_thrust_topic_, kDefaultCommandRollPitchYawRateThrustTopic);



}


void MPCPositionControllerNode::CommandTrajectoryCallback(
    const mav_msgs::CommandTrajectoryConstPtr& trajectory_reference_msg) {
  mav_msgs::EigenCommandTrajectory trajectory;
  printf("Received trajectory msg\n");
  mav_msgs::eigenCommandTrajectoryFromMsg(trajectory_reference_msg, &trajectory);
  MPC_position_controller_.SetCommandTrajectory(trajectory);

  got_first_trajectory_command_ = true;
}


void MPCPositionControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {


  ROS_INFO_ONCE("LeePositionController got first odometry message.");

  if(!got_first_trajectory_command_)
    return;

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  MPC_position_controller_.SetOdometry(odometry);

  Eigen::Vector4d ref_roll_pitch_yawrate_thrust;
  MPC_position_controller_.CalculateAttitudeThrust(&ref_roll_pitch_yawrate_thrust);

  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::CommandRollPitchYawrateThrust command_roll_pitch_yawrate_thrust_msg;
  command_roll_pitch_yawrate_thrust_msg.header = odometry_msg->header;
  command_roll_pitch_yawrate_thrust_msg.roll = ref_roll_pitch_yawrate_thrust(0);
  command_roll_pitch_yawrate_thrust_msg.pitch = ref_roll_pitch_yawrate_thrust(1);
  command_roll_pitch_yawrate_thrust_msg.yaw_rate = ref_roll_pitch_yawrate_thrust(2);
  command_roll_pitch_yawrate_thrust_msg.thrust = ref_roll_pitch_yawrate_thrust(3);
  
  command_roll_pitch_yawrate_thrust_pub_.publish(command_roll_pitch_yawrate_thrust_msg);
}


}



int main(int argc, char** argv) {
  ros::init(argc, argv, "MPCPositionControllerNode");

  rotors_control::MPCPositionControllerNode mpc_controller_node;

  ros::spin();

  return 0;
}

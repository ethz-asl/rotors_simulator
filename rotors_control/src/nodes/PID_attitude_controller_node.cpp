#include <rotors_control/PID_attitude_controller_node.h>



namespace rotors_control{

PIDAttitudeControllerNode::PIDAttitudeControllerNode()
    :initialized_params_(false),
     got_first_attitude_command_(false){

  InitializeParams();
  PID_attitude_controller_.InitializeParams();
  ros::NodeHandle nh(namespace_);

  command_roll_pitch_yawrate_thrust_sub_ = nh.subscribe(command_roll_pitch_yawrate_thrust_topic_, 10,
                                                        &PIDAttitudeControllerNode::CommandRollPitchYawRateThrustCallback, this);
  odometry_sub_ = nh.subscribe(odometry_sub_topic_, 10,
                               &PIDAttitudeControllerNode::OdometryCallback, this);


  motor_velocity_reference_pub_ = nh.advertise<mav_msgs::MotorSpeed>(
        motor_velocity_reference_pub_topic_, 10);
}

PIDAttitudeControllerNode::~PIDAttitudeControllerNode() { }




void PIDAttitudeControllerNode::InitializeParams(){

  ros::NodeHandle pnh("~");
  pnh.param<std::string>("robotNamespace", namespace_, kDefaultNamespace);
  pnh.param<std::string>("commandRollPitchYawRateThrustSubTopic", command_roll_pitch_yawrate_thrust_topic_, kDefaultCommandRollPitchYawRateThrustTopic);
  pnh.param<std::string>("odometrySubTopic", odometry_sub_topic_, kDefaultOdometrySubTopic);
  pnh.param<std::string>("motorVelocityCommandPubTopic", motor_velocity_reference_pub_topic_, kDefaultMotorVelocityReferencePubTopic);

}


void PIDAttitudeControllerNode::CommandRollPitchYawRateThrustCallback(
    const mav_msgs::CommandRollPitchYawrateThrustConstPtr& roll_pitch_yawrate_thrust_reference){
  
  PID_attitude_controller_.SetDesiredAttitude(roll_pitch_yawrate_thrust_reference->roll, roll_pitch_yawrate_thrust_reference->pitch, roll_pitch_yawrate_thrust_reference->yaw_rate, roll_pitch_yawrate_thrust_reference->thrust);
  got_first_attitude_command_ = true;
}


void PIDAttitudeControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {



  ROS_INFO_ONCE("PIDAttitudeController got first odometry message.");

  if(!got_first_attitude_command_)
      return;


  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  
  
  PID_attitude_controller_.SetOdometry(odometry);



   Eigen::VectorXd ref_rotor_velocities;
   PID_attitude_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  mav_msgs::MotorSpeed turning_velocities_msg;

  turning_velocities_msg.motor_speed.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    turning_velocities_msg.motor_speed.push_back(ref_rotor_velocities[i]);
  turning_velocities_msg.header.stamp = odometry_msg->header.stamp;

  motor_velocity_reference_pub_.publish(turning_velocities_msg);


}




}



int main(int argc, char** argv) {
  ros::init(argc, argv, "PIDAttitudeControllerNode");

  rotors_control::PIDAttitudeControllerNode PID_attitude_controller;

  ros::spin();

  return 0;
}

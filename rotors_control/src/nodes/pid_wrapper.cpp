#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "pid_wrapper.h"

#include <cmath>

namespace rotors_control
{

PIDWrapper::PIDWrapper()
{
    InitializeParams();

    ros::NodeHandle nh;

    // Initialize subscribers
    std::string yaw_pid_output_topic = "yaw_rate_ref";
    std::string height_pid_output_topic = "thrust_ref";
    std::string controller_ref_topic = "command/roll_pitch_yaw_height";
    std::string pose_topic = "odometry_sensor1/pose_with_covariance";
    yaw_pid_output_sub_ = nh.subscribe(yaw_pid_output_topic, 1,
                                       &PIDWrapper::YawPIDOutputCallback, this);
    height_pid_output_sub_ = nh.subscribe(height_pid_output_topic, 1,
                                          &PIDWrapper::HeightPIDOutputCallback, this);
    controller_ref_sub_ = nh.subscribe(controller_ref_topic, 1,
                                       &PIDWrapper::RollPitchYawHeightCallback, this);
    pose_sub_ = nh.subscribe(pose_topic, 1,
                             &PIDWrapper::PoseCallback, this);

    // Initialize publishers
    std::string yaw_ref_topic = "command/yaw_ref";
    std::string height_ref_topic = "command/height_ref";
    std::string yaw_obs_topic = "yaw_state";
    std::string height_obs_topic = "height_state";
    std::string roll_pitch_yawrate_thrust_ref_topic = "command/roll_pitch_yawrate_thrust";
    yaw_ref_pub_ = nh.advertise<std_msgs::Float64>(yaw_ref_topic, 1);
    height_ref_pub_ = nh.advertise<std_msgs::Float64>(height_ref_topic, 1);
    yaw_obs_pub_ = nh.advertise<std_msgs::Float64>(yaw_obs_topic, 1);
    height_obs_pub_ = nh.advertise<std_msgs::Float64>(height_obs_topic, 1);
    roll_pitch_yawrate_thrust_ref_pub_ = nh.advertise<mav_msgs::RollPitchYawrateThrust>(roll_pitch_yawrate_thrust_ref_topic, 1);
}

PIDWrapper::~PIDWrapper()
{
}

void PIDWrapper::InitializeParams()
{
    ros::NodeHandle pnh("~");

    // Read parameters from rosparam.
    pnh.getParam("pid_rate", pid_rate_);
}

void PIDWrapper::PublishPIDRef()
{
    // Publish references to PID controllers
    std_msgs::Float64 yaw_ref_msg;
    yaw_ref_msg.data = yaw_ref_;
    yaw_ref_pub_.publish(yaw_ref_msg);

    std_msgs::Float64 height_ref_msg;
    height_ref_msg.data = height_ref_;
    height_ref_pub_.publish(height_ref_msg);

    // Publish observations to PID controllers
    std_msgs::Float64 yaw_obs_msg;
    yaw_obs_msg.data = yaw_;
    yaw_obs_pub_.publish(yaw_obs_msg);

    std_msgs::Float64 height_obs_msg;
    height_obs_msg.data = height_;
    height_obs_pub_.publish(height_obs_msg);
}

void PIDWrapper::PublishControllerRef()
{
    // Combine PID outputs into new reference command
    mav_msgs::RollPitchYawrateThrust roll_pitch_yawrate_thrust_ref_msg = controller_ref_;
    roll_pitch_yawrate_thrust_ref_msg.yaw_rate = yaw_rate_ref_;
    roll_pitch_yawrate_thrust_ref_msg.thrust.z = thrust_ref_;
    roll_pitch_yawrate_thrust_ref_pub_.publish(roll_pitch_yawrate_thrust_ref_msg);
}

void PIDWrapper::HeightPIDOutputCallback(const std_msgs::Float64ConstPtr &height_pid_output_msg)
{
    // Get output from PID
    thrust_ref_ = height_pid_output_msg->data;
}

void PIDWrapper::YawPIDOutputCallback(const std_msgs::Float64ConstPtr &yaw_pid_output_msg)
{
    // Get output from PID
    yaw_rate_ref_ = yaw_pid_output_msg->data;
}

void PIDWrapper::RollPitchYawHeightCallback(
    const mav_msgs::RollPitchYawrateThrustConstPtr &roll_pitch_yaw_height_ref_msg)
{
    // Get reference commands from controller topic
    controller_ref_ = *roll_pitch_yaw_height_ref_msg;
    height_ref_ = controller_ref_.thrust.z; // NOT ACTUALLY A THRUST, WE'RE REUSING AN EXISTING MESSAGE TYPE
    yaw_ref_ = controller_ref_.yaw_rate;    // NOT ACTUALLY A YAW RATE, WE'RE REUSING AN EXISTING MESSAGE TYPE
}

void PIDWrapper::PoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg)
{
    // Get height estimate
    height_ = pose_msg->pose.pose.position.z;

    // Convert pose quaternion estimate to yaw angle
    const double qw = pose_msg->pose.pose.orientation.w;
    const double qx = pose_msg->pose.pose.orientation.x;
    const double qy = pose_msg->pose.pose.orientation.y;
    const double qz = pose_msg->pose.pose.orientation.z;

    const double qw2 = qw * qw;
    const double qx2 = qx * qx;
    const double qy2 = qy * qy;
    const double qz2 = qz * qz;

    yaw_ = atan2(2 * (qx * qy + qz * qw), (qw2 + qx2 - qy2 - qz2));
}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thrust_pid_wrapper_node");

    rotors_control::PIDWrapper pid_wrapper;

    // Wait until nodes are up
    ros::NodeHandle nh;
    boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> p_msg;
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    p_msg = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("pose_with_covariance", nh);

    // Run at PID rate
    // We'll lag behind PID by 1 timestep but it should be ok because the controller
    // reference changes much slower
    ros::Rate rate(pid_wrapper.GetLoopRate());

    while (ros::ok())
    {
        // Send output to Gazebo controller
        pid_wrapper.PublishControllerRef();

        ros::spinOnce();

        // Send output to PID
        pid_wrapper.PublishPIDRef();

        rate.sleep();
    }

    return 0;
}
#ifndef ROTORS_CONTROL_PID_WRAPPER_H
#define ROTORS_CONTROL_PID_WRAPPER_H

#include <mav_msgs/RollPitchYawrateThrust.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>

#include "rotors_control/common.h"

namespace rotors_control
{

class PIDWrapper
{
public:
    PIDWrapper();
    ~PIDWrapper();

    void InitializeParams();
    void PublishPIDRef();
    void PublishControllerRef();

    double GetLoopRate() { return pid_rate_; }

private:
    double pid_rate_ = 0.;

    // State estimates
    double height_ = 0.;
    double yaw_ = 0.;

    // PID inputs
    double height_ref_ = 0.;
    double yaw_ref_ = 0.;

    // PID outputs;
    double thrust_ref_ = 0.;
    double yaw_rate_ref_ = 0.;

    // Reference command from controller
    mav_msgs::RollPitchYawrateThrust controller_ref_;

    // Subscribers
    ros::Subscriber yaw_pid_output_sub_;
    ros::Subscriber height_pid_output_sub_;
    ros::Subscriber controller_ref_sub_;
    ros::Subscriber pose_sub_;

    // Publishers
    ros::Publisher roll_pitch_yawrate_thrust_ref_pub_;
    ros::Publisher yaw_ref_pub_;
    ros::Publisher height_ref_pub_;
    ros::Publisher yaw_obs_pub_;
    ros::Publisher height_obs_pub_;

    void HeightPIDOutputCallback(const std_msgs::Float64ConstPtr &height_pid_output_msg);

    void YawPIDOutputCallback(const std_msgs::Float64ConstPtr &yaw_pid_output_msg);

    void RollPitchYawHeightCallback(
        const mav_msgs::RollPitchYawrateThrustConstPtr &roll_pitch_yaw_height_ref_msg);

    void PoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg);
};
}

#endif // ROTORS_CONTROL_PID_WRAPPER_H
#ifndef ROTORS_swift_INTERFACE_swift_HPP_
#define ROTORS_swift_INTERFACE_swift_HPP_


#include <memory>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <swift_msgs/msg/swift_msgs.hpp>
#include <mav_msgs/msg/roll_pitch_yawrate_thrust.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"

struct Axes {
    int roll;
    int pitch;
    int thrust;
    int roll_direction;
    int pitch_direction;
    int thrust_direction;
};

struct Max {
    double v_xy;
    double roll;
    double pitch;
    double rate_yaw;
    double thrust;
};

class SwiftTrans : public rclcpp::Node
{

    private:
     rclcpp::Publisher<mav_msgs::msg::RollPitchYawrateThrust>::SharedPtr ctrl_pub;
     rclcpp::Subscription<swift_msgs::msg::SwiftMsgs>::ConstSharedPtr swift_sub;

     //check whether to add namespace or not

     Axes axes;
     mav_msgs::msg::RollPitchYawrateThrust control_msg;
     geometry_msgs::msg::PoseStamped pose;
     Max max;

     double current_yaw_vel;
     double div_factor;
     double v_yaw_step;
     double thrust_weight_offset_newtons;

     void StopMav();

     void TransCallback(const swift_msgs::msg::SwiftMsgs::ConstSharedPtr &msg);

    public:
     SwiftTrans();
     void Publish();

};

#endif
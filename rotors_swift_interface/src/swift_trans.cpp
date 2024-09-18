#include "rotors_swift_interface/swift_trans.hpp"
#include "mav_msgs/default_topics.hpp"

SwiftTrans::SwiftTrans() : Node("swift_interface") {

    ctrl_pub = this->create_publisher<mav_msgs::msg::RollPitchYawrateThrust> (mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 50);

    control_msg.roll = 0;
    control_msg.pitch = 0;
    control_msg.yaw_rate = 0;
    control_msg.thrust.x = 0;
    control_msg.thrust.y = 0;
    control_msg.thrust.z = 0;
    current_yaw_vel = 0;

    this->get_parameter_or("axis_direction_roll", axes.roll_direction, 1);
    this->get_parameter_or("axis_direction_pitch", axes.pitch_direction, 1);
    this->get_parameter_or("axis_direction_thrust", axes.thrust_direction, 1);

    this->get_parameter_or("max_v_xy", max.v_xy, 0.8);  // [m/s]
    this->get_parameter_or("max_roll", max.roll, 4.5 * M_PI / 180.0);   // [rad]
    this->get_parameter_or("max_pitch", max.pitch, 4.5 * M_PI / 180.0);   // [rad]
    this->get_parameter_or("max_yaw_rate", max.rate_yaw, 45.0 * M_PI / 180.0);   // [rad]
    this->get_parameter_or("max_thrust", max.thrust, 30.0);  //[N]
    this->get_parameter_or("division_factor", div_factor, 85.0);   // [rad]
    this->get_parameter_or("v_yaw_step", v_yaw_step, 0.05);
    this->get_parameter_or("thrust_weight_offset_newtons",thrust_weight_offset_newtons, 14.896);

    swift_sub = this->create_subscription<swift_msgs::msg::SwiftMsgs>("/drone_command", 50, std::bind(&SwiftTrans::TransCallback, this, std::placeholders::_1));

}

void SwiftTrans::StopMav() {

    control_msg.roll = 0;
    control_msg.pitch = 0;
    control_msg.yaw_rate = 0;
    control_msg.thrust.x = 0;
    control_msg.thrust.y = 0;
    control_msg.thrust.z = 0;

}

void SwiftTrans::TransCallback(const swift_msgs::msg::SwiftMsgs::ConstSharedPtr &msg) {

    control_msg.roll = ((msg->rc_roll-1500) * max.roll/div_factor)*axes.roll_direction;
    control_msg.pitch = ((msg->rc_pitch-1500) * max.pitch/div_factor)*axes.pitch_direction;
    
    if (msg->rc_yaw < 1500) {
        current_yaw_vel = max.rate_yaw;
    }
    else if (msg->rc_yaw > 1500) {
        current_yaw_vel = -max.rate_yaw;
    }
    else {
        current_yaw_vel = 0;
    }
    control_msg.yaw_rate = current_yaw_vel;

    //option1
    if(msg->rc_throttle > 1499)
        control_msg.thrust.z = ((msg->rc_throttle-1500) * (max.thrust-thrust_weight_offset_newtons)/500.0) + (thrust_weight_offset_newtons);
    else
        control_msg.thrust.z = (1+((msg->rc_throttle-1500)/500.0))*(thrust_weight_offset_newtons);

    rclcpp::Time update_time = this->get_clock()->now();    

    control_msg.header.stamp = update_time;
    control_msg.header.frame_id = "rotors_swift_frame";
    Publish();
}

void SwiftTrans::Publish() {
    ctrl_pub->publish(control_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwiftTrans>());
    rclcpp::shutdown();
    return 0;
    
}


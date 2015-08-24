/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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


#include "rotors_gazebo_plugins/gazebo_mavlink_interface.h"

namespace gazebo {

GazeboMavlinkInterface::~GazeboMavlinkInterface() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboMavlinkInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model.
  model_ = _model;

  world_ = model_->GetWorld();

  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_mavlink_interface] Please specify a robotNamespace.\n";

  node_handle_ = new ros::NodeHandle(namespace_);

  getSdfParam<std::string>(_sdf, "motorSpeedCommandPubTopic", motor_velocity_reference_pub_topic_,
                           motor_velocity_reference_pub_topic_);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboMavlinkInterface::OnUpdate, this, _1));

  mav_control_sub_ = node_handle_->subscribe(mavlink_control_sub_topic_, 10,
                                           &GazeboMavlinkInterface::MavlinkControlCallback,
                                           this);
  // Subscriber to IMU sensor_msgs::Imu Message.
  imu_sub_ = node_handle_->subscribe(imu_sub_topic_, 10, &GazeboMavlinkInterface::ImuCallback, this);
  
  motor_velocity_reference_pub_ = node_handle_->advertise<mav_msgs::CommandMotorSpeed>(motor_velocity_reference_pub_topic_, 10);
  hil_sensor_pub_ = node_handle_->advertise<mavros_msgs::Mavlink>(hil_sensor_mavlink_pub_topic_, 10);

  _rotor_count = 4;
  last_time_ = world_->GetSimTime();
  last_gps_time_ = world_->GetSimTime();
  double gps_update_interval_ = 200*1000000;  // nanoseconds for 5Hz

  gravity_W_ = world_->GetPhysicsEngine()->GetGravity();

  // Magnetic field data for Zurich from WMM2015 (10^5xnanoTesla (N, E, D))
  mag_W_ = {0.21523, 0.00771, 0.42741};

}

// This gets called by the world update start event.
void GazeboMavlinkInterface::OnUpdate(const common::UpdateInfo& /*_info*/) {
  gzerr << "[gazebo_mavlink_interface] Please specify a robotNamespace.\n";
  if(!received_first_referenc_)
    return;

  common::Time now = world_->GetSimTime();

  mav_msgs::CommandMotorSpeedPtr turning_velocities_msg(new mav_msgs::CommandMotorSpeed);

  for (int i = 0; i < input_reference_.size(); i++)
  turning_velocities_msg->motor_speed.push_back(input_reference_[i]);
  turning_velocities_msg->header.stamp.sec = now.sec;
  turning_velocities_msg->header.stamp.nsec = now.nsec;

  motor_velocity_reference_pub_.publish(turning_velocities_msg);
  turning_velocities_msg.reset();

  //send gps
  common::Time current_time  = now;
  double dt = (current_time - last_time_).Double();
  last_time_ = current_time;
  double t = current_time.Double();

  math::Pose T_W_I = model_->GetWorldPose(); //TODO(burrimi): Check tf.
  math::Vector3 pos_W_I = T_W_I.pos;  // Use the models' world position for GPS and pressure alt.

  math::Vector3 velocity_current_W = model_->GetWorldLinearVel();  // Use the models' world position for GPS velocity.

  math::Vector3 velocity_current_W_xy = velocity_current_W;
  velocity_current_W_xy.z = 0.0;

  // TODO: Remove GPS message from IMU plugin. Added gazebo GPS plugin. This is temp here.
  float lat_zurich = 47.3667;  // deg
  float long_zurich = 8.5500;  // deg
  float earth_radius = 6353000;  // m
  
  common::Time gps_update(gps_update_interval_);

  if(current_time - last_gps_time_ > gps_update){  // 5Hz
    mavlink_message_t gps_mmsg;

    hil_gps_msg_.time_usec = current_time.nsec*1000;
    hil_gps_msg_.fix_type = 3;
    hil_gps_msg_.lat = (lat_zurich + (pos_W_I.x/earth_radius)*180/3.1416) * 10000000;
    hil_gps_msg_.lon = (long_zurich + (-pos_W_I.y/earth_radius)*180/3.1416) * 10000000;
    hil_gps_msg_.alt = pos_W_I.z * 1000;
    hil_gps_msg_.eph = 100;
    hil_gps_msg_.epv = 100;
    hil_gps_msg_.vel = velocity_current_W_xy.GetLength() * 100;
    hil_gps_msg_.vn = velocity_current_W.x * 100;
    hil_gps_msg_.ve = -velocity_current_W.y * 100;
    hil_gps_msg_.vd = -velocity_current_W.z * 100;
    hil_gps_msg_.cog = atan2(hil_gps_msg_.ve, hil_gps_msg_.vn) * 180.0/3.1416 * 100.0;
    hil_gps_msg_.satellites_visible = 10;
           
    mavlink_hil_gps_t* hil_gps_msg = &hil_gps_msg_;
    mavlink_msg_hil_gps_encode(1, 240, &gps_mmsg, hil_gps_msg);
    mavlink_message_t* gps_msg = &gps_mmsg;
    mavros_msgs::MavlinkPtr gps_rmsg = boost::make_shared<mavros_msgs::Mavlink>();
    gps_rmsg->header.stamp = ros::Time::now();
    mavros_msgs::mavlink::convert(*gps_msg, *gps_rmsg);
    
    hil_sensor_pub_.publish(gps_rmsg);

    last_gps_time_ = current_time;
  }
}

void GazeboMavlinkInterface::CommandMotorMavros(const mav_msgs::CommandMotorSpeedPtr& input_reference_msg) {
  input_reference_.resize(input_reference_msg->motor_speed.size());
  for (int i = 0; i < input_reference_msg->motor_speed.size(); ++i) {
    input_reference_[i] = input_reference_msg->motor_speed[i];
  }
  received_first_referenc_ = true;
}


void GazeboMavlinkInterface::MavlinkControlCallback(const mavros_msgs::Mavlink::ConstPtr &rmsg) {
  mavlink_message_t mmsg;

  if(mavros_msgs::mavlink::convert(*rmsg, mmsg)){
    mavlink_hil_controls_t act_msg;

    mavlink_message_t* msg = &mmsg;

    mavlink_msg_hil_controls_decode(msg, &act_msg);

    inputs.control[0] =(double)act_msg.roll_ailerons;
    inputs.control[1] =(double)act_msg.pitch_elevator;
    inputs.control[2] =(double)act_msg.yaw_rudder;
    inputs.control[3] =(double)act_msg.throttle;
    inputs.control[4] =(double)act_msg.aux1;
    inputs.control[5] =(double)act_msg.aux2;
    inputs.control[6] =(double)act_msg.aux3;
    inputs.control[7] =(double)act_msg.aux4;

    // publish message
    double scaling = 150;
    double offset = 600;

    mav_msgs::CommandMotorSpeedPtr turning_velocities_msg(new mav_msgs::CommandMotorSpeed);

    for (int i = 0; i < _rotor_count; i++) {
      turning_velocities_msg->motor_speed.push_back(inputs.control[i] * scaling + offset);
    }

    CommandMotorMavros(turning_velocities_msg);
    turning_velocities_msg.reset();

  } else{
    std::cout << "incorrect mavlink data" <<"\n";
  }
}

void GazeboMavlinkInterface::ImuCallback(const sensor_msgs::ImuConstPtr& imu_message) {
  mavlink_message_t mmsg;

  math::Pose T_W_I = model_->GetWorldPose();
  math::Vector3 pos_W_I = T_W_I.pos;  // Use the models'world position for GPS and pressure alt.

  math::Quaternion C_W_I;
  C_W_I.w = imu_message->orientation.w;
  C_W_I.x = imu_message->orientation.x;
  C_W_I.y = imu_message->orientation.y;
  C_W_I.z = imu_message->orientation.z;

  math::Vector3 mag_I = C_W_I.RotateVectorReverse(mag_W_); // TODO: Add noise based on bais and variance like for imu and gyro

  hil_sensor_msg_.time_usec = imu_message->header.stamp.nsec*1000;
  hil_sensor_msg_.xacc = imu_message->linear_acceleration.x;
  hil_sensor_msg_.yacc = imu_message->linear_acceleration.y;
  hil_sensor_msg_.zacc = imu_message->linear_acceleration.z;
  hil_sensor_msg_.xgyro = imu_message->angular_velocity.x;
  hil_sensor_msg_.ygyro = imu_message->angular_velocity.y;
  hil_sensor_msg_.zgyro = imu_message->angular_velocity.z;
  hil_sensor_msg_.xmag = mag_I.x;
  hil_sensor_msg_.ymag = mag_I.y;
  hil_sensor_msg_.zmag = mag_I.z;
  hil_sensor_msg_.abs_pressure = 0.0;
  hil_sensor_msg_.diff_pressure = 0.0;
  hil_sensor_msg_.pressure_alt = pos_W_I.z;
  hil_sensor_msg_.temperature = 0.0;
  hil_sensor_msg_.fields_updated = 4095;  // 0b1111111111111 (All updated since new data with new noise added always)

  mavlink_hil_sensor_t* hil_msg = &hil_sensor_msg_;
  mavlink_msg_hil_sensor_encode(1, 240, &mmsg, hil_msg);
  mavlink_message_t* msg = &mmsg;
  
  mavros_msgs::MavlinkPtr rmsg = boost::make_shared<mavros_msgs::Mavlink>();
  rmsg->header.stamp = ros::Time::now();
  mavros_msgs::mavlink::convert(*msg, *rmsg);

  hil_sensor_pub_.publish(rmsg);

}

GZ_REGISTER_MODEL_PLUGIN(GazeboMavlinkInterface);
}

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


#include "gazebo_mavlink_interface.h"
#include "geo_mag_declination.h"

namespace gazebo {

// Set global reference point
// Zurich Irchel Park: 47.397742, 8.545594, 488m
// Seattle downtown (15 deg declination): 47.592182, -122.316031, 86m
// Moscow downtown: 55.753395, 37.625427, 155m

// Zurich Irchel Park
static const double lat_zurich = 47.397742 * M_PI / 180;  // rad
static const double lon_zurich = 8.545594 * M_PI / 180;  // rad
static const double alt_zurich = 488.0; // meters
// Seattle downtown (15 deg declination): 47.592182, -122.316031
// static const double lat_zurich = 47.592182 * M_PI / 180;  // rad
// static const double lon_zurich = -122.316031 * M_PI / 180;  // rad
// static const double alt_zurich = 86.0; // meters
static const float earth_radius = 6353000;  // m


GZ_REGISTER_MODEL_PLUGIN(GazeboMavlinkInterface);

GazeboMavlinkInterface::~GazeboMavlinkInterface() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
}

void GazeboMavlinkInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model.
  model_ = _model;

  world_ = model_->GetWorld();

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_mavlink_interface] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  getSdfParam<std::string>(_sdf, "motorSpeedCommandPubTopic", motor_velocity_reference_pub_topic_,
                           motor_velocity_reference_pub_topic_);
  getSdfParam<std::string>(_sdf, "imuSubTopic", imu_sub_topic_, imu_sub_topic_);
  getSdfParam<std::string>(_sdf, "lidarSubTopic", lidar_sub_topic_, lidar_sub_topic_);
  getSdfParam<std::string>(_sdf, "opticalFlowSubTopic",
      opticalFlow_sub_topic_, opticalFlow_sub_topic_);

  // set input_reference_ from inputs.control
  input_reference_.resize(n_out_max);
  joints_.resize(n_out_max);
  pids_.resize(n_out_max);
  for(int i = 0; i < n_out_max; ++i)
  {
    pids_[i].Init(0, 0, 0, 0, 0, 0, 0);
    input_reference_[i] = 0;
  }

  if (_sdf->HasElement("control_channels")) {
    sdf::ElementPtr control_channels = _sdf->GetElement("control_channels");
    sdf::ElementPtr channel = control_channels->GetElement("channel");
    while (channel)
    {
      if (channel->HasElement("input_index"))
      {
        int index = channel->Get<int>("input_index");
        if (index < n_out_max)
        {
          input_offset_[index] = channel->Get<double>("input_offset");
          input_scaling_[index] = channel->Get<double>("input_scaling");
          zero_position_disarmed_[index] = channel->Get<double>("zero_position_disarmed");
          zero_position_armed_[index] = channel->Get<double>("zero_position_armed");
          if (channel->HasElement("joint_control_type"))
          {
            joint_control_type_[index] = channel->Get<std::string>("joint_control_type");
          }
          else
          {
            gzwarn << "joint_control_type[" << index << "] not specified, using velocity.\n";
            joint_control_type_[index] = "velocity";
          }

          // start gz transport node handle
          if (joint_control_type_[index] == "position_gztopic")
          {
            // setup publisher handle to topic
            if (channel->HasElement("gztopic"))
              gztopic_[index] = "~/"+ model_->GetName() + channel->Get<std::string>("gztopic");
            else
              gztopic_[index] = "control_position_gztopic_" + std::to_string(index);
            #if GAZEBO_MAJOR_VERSION >= 7 && GAZEBO_MINOR_VERSION >= 4
              /// only gazebo 7.4 and above support Any
              joint_control_pub_[index] = node_handle_->Advertise<gazebo::msgs::Any>(
                gztopic_[index]);
            #else
              joint_control_pub_[index] = node_handle_->Advertise<gazebo::msgs::GzString>(
                gztopic_[index]);
            #endif
          }

          if (channel->HasElement("joint_name"))
          {
            std::string joint_name = channel->Get<std::string>("joint_name");
            joints_[index] = model_->GetJoint(joint_name);
            if (joints_[index] == nullptr)
            {
              gzwarn << "joint [" << joint_name << "] not found for channel["
                     << index << "] no joint control for this channel.\n";
            }
            else
            {
              gzdbg << "joint [" << joint_name << "] found for channel["
                    << index << "] joint control active for this channel.\n";
            }
          }
          else
          {
            gzdbg << "<joint_name> not found for channel[" << index
                  << "] no joint control will be performed for this channel.\n";
          }

          // setup joint control pid to control joint
          if (channel->HasElement("joint_control_pid"))
          {
            sdf::ElementPtr pid = channel->GetElement("joint_control_pid");
            double p = 0;
            if (pid->HasElement("p"))
              p = pid->Get<double>("p");
            double i = 0;
            if (pid->HasElement("i"))
              i = pid->Get<double>("i");
            double d = 0;
            if (pid->HasElement("d"))
              d = pid->Get<double>("d");
            double iMax = 0;
            if (pid->HasElement("iMax"))
              iMax = pid->Get<double>("iMax");
            double iMin = 0;
            if (pid->HasElement("iMin"))
              iMin = pid->Get<double>("iMin");
            double cmdMax = 0;
            if (pid->HasElement("cmdMax"))
              cmdMax = pid->Get<double>("cmdMax");
            double cmdMin = 0;
            if (pid->HasElement("cmdMin"))
              cmdMin = pid->Get<double>("cmdMin");
            pids_[index].Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
          }
        }
        else
        {
          gzerr << "input_index[" << index << "] out of range, not parsing.\n";
        }
      }
      else
      {
        gzerr << "no input_index, not parsing.\n";
        break;
      }
      channel = channel->GetNextElement("channel");
    }
  }

  if (_sdf->HasElement("left_elevon_joint")) {
    sdf::ElementPtr left_elevon =  _sdf->GetElement("left_elevon_joint");
    std::string left_elevon_joint_name = left_elevon->Get<std::string>();
    left_elevon_joint_ = model_->GetJoint(left_elevon_joint_name);
    int control_index;
    getSdfParam<int>(left_elevon, "input_index", control_index, -1);
    if (control_index >= 0) {
      joints_.at(control_index) = left_elevon_joint_;
    }
    // setup pid to control joint
    use_left_elevon_pid_ = false;
    if (left_elevon->HasElement("joint_control_pid")) {
      // setup joint control pid to control joint
      sdf::ElementPtr pid = _sdf->GetElement("joint_control_pid");
      double p = 0.1;
      if (pid->HasElement("p"))
        p = pid->Get<double>("p");
      double i = 0;
      if (pid->HasElement("i"))
        i = pid->Get<double>("i");
      double d = 0;
      if (pid->HasElement("d"))
        d = pid->Get<double>("d");
      double iMax = 0;
      if (pid->HasElement("iMax"))
        iMax = pid->Get<double>("iMax");
      double iMin = 0;
      if (pid->HasElement("iMin"))
        iMin = pid->Get<double>("iMin");
      double cmdMax = 3;
      if (pid->HasElement("cmdMax"))
        cmdMax = pid->Get<double>("cmdMax");
      double cmdMin = -3;
      if (pid->HasElement("cmdMin"))
        cmdMin = pid->Get<double>("cmdMin");
      left_elevon_pid_.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
      use_left_elevon_pid_ = true;
    }
  }

  if (_sdf->HasElement("left_aileron_joint")) {
    std::string left_elevon_joint_name = _sdf->GetElement("left_aileron_joint")->Get<std::string>();
    left_elevon_joint_ = model_->GetJoint(left_elevon_joint_name);
    int control_index;
    getSdfParam<int>(_sdf->GetElement("left_aileron_joint"), "input_index", control_index, -1);
    if (control_index >= 0) {
      joints_.at(control_index) = left_elevon_joint_;
    }
  }

  if (_sdf->HasElement("right_elevon_joint")) {
    sdf::ElementPtr right_elevon =  _sdf->GetElement("right_elevon_joint");
    std::string right_elevon_joint_name = right_elevon->Get<std::string>();
    right_elevon_joint_ = model_->GetJoint(right_elevon_joint_name);
    int control_index;
    getSdfParam<int>(right_elevon, "input_index", control_index, -1);
    if (control_index >= 0) {
      joints_.at(control_index) = right_elevon_joint_;
    }
    // setup pid to control joint
    use_right_elevon_pid_ = false;
    if (right_elevon->HasElement("joint_control_pid")) {
      // setup joint control pid to control joint
      sdf::ElementPtr pid = _sdf->GetElement("joint_control_pid");
      double p = 0.1;
      if (pid->HasElement("p"))
        p = pid->Get<double>("p");
      double i = 0;
      if (pid->HasElement("i"))
        i = pid->Get<double>("i");
      double d = 0;
      if (pid->HasElement("d"))
        d = pid->Get<double>("d");
      double iMax = 0;
      if (pid->HasElement("iMax"))
        iMax = pid->Get<double>("iMax");
      double iMin = 0;
      if (pid->HasElement("iMin"))
        iMin = pid->Get<double>("iMin");
      double cmdMax = 3;
      if (pid->HasElement("cmdMax"))
        cmdMax = pid->Get<double>("cmdMax");
      double cmdMin = -3;
      if (pid->HasElement("cmdMin"))
        cmdMin = pid->Get<double>("cmdMin");
      right_elevon_pid_.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
      use_right_elevon_pid_ = true;
    }
  }

  if (_sdf->HasElement("right_aileron_joint")) {
    std::string right_elevon_joint_name = _sdf->GetElement("right_aileron_joint")->Get<std::string>();
    right_elevon_joint_ = model_->GetJoint(right_elevon_joint_name);
    int control_index;
    getSdfParam<int>(_sdf->GetElement("right_aileron_joint"), "input_index", control_index, -1);
    if (control_index >= 0) {
      joints_.at(control_index) = right_elevon_joint_;
    }
  }

  if (_sdf->HasElement("elevator_joint")) {
    sdf::ElementPtr elevator =  _sdf->GetElement("elevator_joint");
    std::string elevator_joint_name = elevator->Get<std::string>();
    elevator_joint_ = model_->GetJoint(elevator_joint_name);
    int control_index;
    getSdfParam<int>(elevator, "input_index", control_index, -1);
    if (control_index >= 0) {
      joints_.at(control_index) = elevator_joint_;
    }
    // setup pid to control joint
    use_elevator_pid_ = false;
    if (elevator->HasElement("joint_control_pid")) {
      // setup joint control pid to control joint
      sdf::ElementPtr pid = _sdf->GetElement("joint_control_pid");
      double p = 0.1;
      if (pid->HasElement("p"))
        p = pid->Get<double>("p");
      double i = 0;
      if (pid->HasElement("i"))
        i = pid->Get<double>("i");
      double d = 0;
      if (pid->HasElement("d"))
        d = pid->Get<double>("d");
      double iMax = 0;
      if (pid->HasElement("iMax"))
        iMax = pid->Get<double>("iMax");
      double iMin = 0;
      if (pid->HasElement("iMin"))
        iMin = pid->Get<double>("iMin");
      double cmdMax = 3;
      if (pid->HasElement("cmdMax"))
        cmdMax = pid->Get<double>("cmdMax");
      double cmdMin = -3;
      if (pid->HasElement("cmdMin"))
        cmdMin = pid->Get<double>("cmdMin");
      elevator_pid_.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
      use_elevator_pid_ = true;
    }
  }

  if (_sdf->HasElement("propeller_joint")) {
    sdf::ElementPtr propeller =  _sdf->GetElement("propeller_joint");
    std::string propeller_joint_name = propeller->Get<std::string>();
    propeller_joint_ = model_->GetJoint(propeller_joint_name);
    int control_index;
    getSdfParam<int>(propeller, "input_index", control_index, -1);
    if (control_index >= 0) {
      joints_.at(control_index) = propeller_joint_;
    }
    use_propeller_pid_ = false;
    // setup joint control pid to control joint
    if (propeller->HasElement("joint_control_pid"))
    {
      sdf::ElementPtr pid = _sdf->GetElement("joint_control_pid");
      double p = 0.1;
      if (pid->HasElement("p"))
        p = pid->Get<double>("p");
      double i = 0;
      if (pid->HasElement("i"))
        i = pid->Get<double>("i");
      double d = 0;
      if (pid->HasElement("d"))
        d = pid->Get<double>("d");
      double iMax = 0;
      if (pid->HasElement("iMax"))
        iMax = pid->Get<double>("iMax");
      double iMin = 0;
      if (pid->HasElement("iMin"))
        iMin = pid->Get<double>("iMin");
      double cmdMax = 3;
      if (pid->HasElement("cmdMax"))
        cmdMax = pid->Get<double>("cmdMax");
      double cmdMin = -3;
      if (pid->HasElement("cmdMin"))
        cmdMin = pid->Get<double>("cmdMin");
      propeller_pid_.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
      use_propeller_pid_ = true;
    }
  }

  if (_sdf->HasElement("cgo3_mount_joint")) {
    std::string gimbal_yaw_joint_name = _sdf->GetElement("cgo3_mount_joint")->Get<std::string>();
    gimbal_yaw_joint_ = model_->GetJoint(gimbal_yaw_joint_name);
    int control_index;
    getSdfParam<int>(_sdf->GetElement("cgo3_mount_joint"), "input_index", control_index, -1);
    if (control_index >= 0) {
      joints_.at(control_index) = gimbal_yaw_joint_;
    }
  }

  if (_sdf->HasElement("cgo3_vertical_arm_joint")) {
    std::string gimbal_roll_joint_name = _sdf->GetElement("cgo3_vertical_arm_joint")->Get<std::string>();
    gimbal_roll_joint_ = model_->GetJoint(gimbal_roll_joint_name);
    int control_index;
    getSdfParam<int>(_sdf->GetElement("cgo3_vertical_arm_joint"), "input_index", control_index, -1);
    if (control_index >= 0) {
      joints_.at(control_index) = gimbal_roll_joint_;
    }
  }

  if (_sdf->HasElement("cgo3_horizontal_arm_joint")) {
    std::string gimbal_pitch_joint_name = _sdf->GetElement("cgo3_horizontal_arm_joint")->Get<std::string>();
    gimbal_pitch_joint_ = model_->GetJoint(gimbal_pitch_joint_name);
    int control_index;
    getSdfParam<int>(_sdf->GetElement("cgo3_horizontal_arm_joint"), "input_index", control_index, -1);
    if (control_index >= 0) {
      joints_.at(control_index) = gimbal_pitch_joint_;
    }
  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboMavlinkInterface::OnUpdate, this, _1));

  // Subscriber to IMU sensor_msgs::Imu Message and SITL message
  imu_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + imu_sub_topic_, &GazeboMavlinkInterface::ImuCallback, this);
  lidar_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + lidar_sub_topic_, &GazeboMavlinkInterface::LidarCallback, this);
  opticalFlow_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + opticalFlow_sub_topic_, &GazeboMavlinkInterface::OpticalFlowCallback, this);
  
  // Publish gazebo's motor_speed message
  motor_velocity_reference_pub_ = node_handle_->Advertise<mav_msgs::msgs::CommandMotorSpeed>("~/" + model_->GetName() + motor_velocity_reference_pub_topic_, 1);

  _rotor_count = 5;
  last_time_ = world_->GetSimTime();
  last_gps_time_ = world_->GetSimTime();
  gps_update_interval_ = 0.2;  // in seconds for 5Hz

  gravity_W_ = world_->GetPhysicsEngine()->GetGravity();

  // Magnetic field data for Zurich from WMM2015 (10^5xnanoTesla (N, E D) n-frame )
  // mag_n_ = {0.21523, 0.00771, -0.42741};
  // We set the world Y component to zero because we apply
  // the declination based on the global position,
  // and so we need to start without any offsets.
  // The real value for Zurich would be 0.00771
  // frame d is the magnetic north frame
  mag_d_.x = 0.21523;
  mag_d_.y = 0;
  mag_d_.z = -0.42741;

  //Create socket
  // udp socket data
  mavlink_addr_ = htonl(INADDR_ANY);
  if (_sdf->HasElement("mavlink_addr")) {
    std::string mavlink_addr = _sdf->GetElement("mavlink_addr")->Get<std::string>();
    if (mavlink_addr != "INADDR_ANY") {
      mavlink_addr_ = inet_addr(mavlink_addr.c_str());
      if (mavlink_addr_ == INADDR_NONE) {
        fprintf(stderr, "invalid mavlink_addr \"%s\"\n", mavlink_addr.c_str());
        return;
      }
    }
  }
  if (_sdf->HasElement("mavlink_udp_port")) {
    mavlink_udp_port_ = _sdf->GetElement("mavlink_udp_port")->Get<int>();
  }

  // try to setup udp socket for communcation with simulator
  if ((_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    printf("create socket failed\n");
    return;
  }

  memset((char *)&_myaddr, 0, sizeof(_myaddr));
  _myaddr.sin_family = AF_INET;
  _myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  // Let the OS pick the port
  _myaddr.sin_port = htons(0);

  if (bind(_fd, (struct sockaddr *)&_myaddr, sizeof(_myaddr)) < 0) {
    printf("bind failed\n");
    return;
  }

  _srcaddr.sin_family = AF_INET;
  _srcaddr.sin_addr.s_addr = mavlink_addr_;
  _srcaddr.sin_port = htons(mavlink_udp_port_);
  _addrlen = sizeof(_srcaddr);

  fds[0].fd = _fd;
  fds[0].events = POLLIN;

  gps_pub_ = node_handle_->Advertise<msgs::Vector3d>("~/gps_position");
}

// This gets called by the world update start event.
void GazeboMavlinkInterface::OnUpdate(const common::UpdateInfo& /*_info*/) {

  common::Time current_time = world_->GetSimTime();
  double dt = (current_time - last_time_).Double();

  pollForMAVLinkMessages(dt, 1000);

  handle_control(dt);

  if(received_first_referenc_) {

    mav_msgs::msgs::CommandMotorSpeed turning_velocities_msg;

    for (int i = 0; i < input_reference_.size(); i++){
      if (last_actuator_time_ == 0 || (current_time - last_actuator_time_).Double() > 0.2) {
        turning_velocities_msg.add_motor_speed(0);
      } else {
        turning_velocities_msg.add_motor_speed(input_reference_[i]);
      }
    }
    // TODO Add timestamp and Header
    // turning_velocities_msg->header.stamp.sec = current_time.sec;
    // turning_velocities_msg->header.stamp.nsec = current_time.nsec;

    // gzerr << turning_velocities_msg.motor_speed(0) << "\n";
    motor_velocity_reference_pub_->Publish(turning_velocities_msg);
  }

  last_time_ = current_time;

  //send gps
  math::Pose T_W_I = model_->GetWorldPose(); //TODO(burrimi): Check tf.
  math::Vector3 pos_W_I = T_W_I.pos;  // Use the models' world position for GPS and pressure alt.

  math::Vector3 velocity_current_W = model_->GetWorldLinearVel();  // Use the models' world position for GPS velocity.

  math::Vector3 velocity_current_W_xy = velocity_current_W;
  velocity_current_W_xy.z = 0;

  // TODO: Remove GPS message from IMU plugin. Added gazebo GPS plugin. This is temp here.
  // reproject local position to gps coordinates
  double x_rad = pos_W_I.y / earth_radius; // north
  double y_rad = pos_W_I.x / earth_radius; // east
  double c = sqrt(x_rad * x_rad + y_rad * y_rad);
  double sin_c = sin(c);
  double cos_c = cos(c);
  if (c != 0.0) {
    lat_rad = asin(cos_c * sin(lat_zurich) + (x_rad * sin_c * cos(lat_zurich)) / c);
    lon_rad = (lon_zurich + atan2(y_rad * sin_c, c * cos(lat_zurich) * cos_c - x_rad * sin(lat_zurich) * sin_c));
  } else {
   lat_rad = lat_zurich;
    lon_rad = lon_zurich;
  }

  if (current_time.Double() - last_gps_time_.Double() > gps_update_interval_) {  // 5Hz
    // Raw UDP mavlink
    mavlink_hil_gps_t hil_gps_msg;
    hil_gps_msg.time_usec = current_time.nsec*1000;
    hil_gps_msg.fix_type = 3;
    hil_gps_msg.lat = lat_rad * 180 / M_PI * 1e7;
    hil_gps_msg.lon = lon_rad * 180 / M_PI * 1e7;
    hil_gps_msg.alt = (pos_W_I.z + alt_zurich) * 1000;
    hil_gps_msg.eph = 100;
    hil_gps_msg.epv = 100;
    hil_gps_msg.vel = velocity_current_W_xy.GetLength() * 100;
    hil_gps_msg.vn = velocity_current_W.y * 100;
    hil_gps_msg.ve = velocity_current_W.x * 100;
    hil_gps_msg.vd = -velocity_current_W.z * 100;
    hil_gps_msg.cog = atan2(hil_gps_msg.ve, hil_gps_msg.vn) * 180.0/3.1416 * 100.0;
    hil_gps_msg.satellites_visible = 10;

    send_mavlink_message(MAVLINK_MSG_ID_HIL_GPS, &hil_gps_msg, 200);

    msgs::Vector3d gps_msg;
    gps_msg.set_x(lat_rad * 180. / M_PI);
    gps_msg.set_y(lon_rad * 180. / M_PI);
    gps_msg.set_z(hil_gps_msg.alt / 1000.f);
    gps_pub_->Publish(gps_msg);

    last_gps_time_ = current_time;
  }
}

void GazeboMavlinkInterface::send_mavlink_message(const uint8_t msgid, const void *msg, uint8_t component_ID) {
  component_ID = 0;
  uint8_t payload_len = mavlink_message_lengths[msgid];
  unsigned packet_len = payload_len + MAVLINK_NUM_NON_PAYLOAD_BYTES;

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  /* header */
  buf[0] = MAVLINK_STX;
  buf[1] = payload_len;
  /* no idea which numbers should be here*/
  buf[2] = 100;
  buf[3] = 0;
  buf[4] = component_ID;
  buf[5] = msgid;

  /* payload */
  memcpy(&buf[MAVLINK_NUM_HEADER_BYTES],msg, payload_len);

  /* checksum */
  uint16_t checksum;
  crc_init(&checksum);
  crc_accumulate_buffer(&checksum, (const char *) &buf[1], MAVLINK_CORE_HEADER_LEN + payload_len);
  crc_accumulate(mavlink_message_crcs[msgid], &checksum);

  buf[MAVLINK_NUM_HEADER_BYTES + payload_len] = (uint8_t)(checksum & 0xFF);
  buf[MAVLINK_NUM_HEADER_BYTES + payload_len + 1] = (uint8_t)(checksum >> 8);

  ssize_t len;

  len = sendto(_fd, buf, packet_len, 0, (struct sockaddr *)&_srcaddr, sizeof(_srcaddr));

  if (len <= 0) {
    printf("Failed sending mavlink message\n");
  }
}

void GazeboMavlinkInterface::ImuCallback(ImuPtr& imu_message) {

  // frames
  // g - gazebo (ENU), east, north, up
  // r - rotors imu frame (FLU), forward, left, up
  // b - px4 (FRD) forward, right down
  // n - px4 (NED) north, east, down
  math::Quaternion q_gr = math::Quaternion(
    imu_message->orientation().w(),
    imu_message->orientation().x(),
    imu_message->orientation().y(),
    imu_message->orientation().z());


  // q_br
  /*
  tf.euler2quat(*tf.mat2euler([
  #        F  L  U
          [1, 0, 0],  # F
          [0, -1, 0], # R
          [0, 0, -1]  # D
      ]
  )).round(5)
  */
  math::Quaternion q_br(0, 1, 0, 0);


  // q_ng
  /*
  tf.euler2quat(*tf.mat2euler([
  #        N  E  D
          [0, 1, 0],  # E
          [1, 0, 0],  # N
          [0, 0, -1]  # U
      ]
  )).round(5)
  */
  math::Quaternion q_ng(0, 0.70711, 0.70711, 0);

  math::Quaternion q_gb = q_gr*q_br.GetInverse();
  math::Quaternion q_nb = q_ng*q_gb;

  math::Vector3 pos_g = model_->GetWorldPose().pos;
  math::Vector3 pos_n = q_ng.RotateVector(pos_g);

  //gzerr << "got imu: " << C_W_I << "\n";
  //gzerr << "got pose: " << T_W_I.rot << "\n";
  float declination = get_mag_declination(lat_rad, lon_rad);

  math::Quaternion q_dn(0.0, 0.0, declination);
  math::Vector3 mag_n = q_dn.RotateVectorReverse(mag_d_);

  math::Vector3 vel_b = q_br.RotateVector(model_->GetRelativeLinearVel());
  math::Vector3 vel_n = q_ng.RotateVector(model_->GetWorldLinearVel());
  math::Vector3 omega_nb_b = q_br.RotateVector(model_->GetRelativeAngularVel());

  standard_normal_distribution_ = std::normal_distribution<float>(0, 0.01f);
  math::Vector3 mag_noise_b(
    standard_normal_distribution_(random_generator_),
    standard_normal_distribution_(random_generator_),
    standard_normal_distribution_(random_generator_));

  math::Vector3 accel_b = q_br.RotateVector(math::Vector3(
    imu_message->linear_acceleration().x(),
    imu_message->linear_acceleration().y(),
    imu_message->linear_acceleration().z()));
  math::Vector3 gyro_b = q_br.RotateVector(math::Vector3(
    imu_message->angular_velocity().x(),
    imu_message->angular_velocity().y(),
    imu_message->angular_velocity().z()));
  math::Vector3 mag_b = q_nb.RotateVectorReverse(mag_n) + mag_noise_b;

  mavlink_hil_sensor_t sensor_msg;
  sensor_msg.time_usec = world_->GetSimTime().nsec*1000;
  sensor_msg.xacc = accel_b.x;
  sensor_msg.yacc = accel_b.y;
  sensor_msg.zacc = accel_b.z;
  sensor_msg.xgyro = gyro_b.x;
  sensor_msg.ygyro = gyro_b.y;
  sensor_msg.zgyro = gyro_b.z;
  sensor_msg.xmag = mag_b.x;
  sensor_msg.ymag = mag_b.y;
  sensor_msg.zmag = mag_b.z;
  sensor_msg.abs_pressure = 0.0;
  float rho = 1.2754f; // density of air, TODO why is this not 1.225 as given by std. atmos.
  sensor_msg.diff_pressure = 0.5f*rho*vel_b.x*vel_b.x / 100;

  float p1, p2;

  // need to add noise to pressure alt
  do {
      p1 = rand() * (1.0 / RAND_MAX);
      p2 = rand() * (1.0 / RAND_MAX);
  } while (p1 <= FLT_EPSILON);

  float n = sqrtf(-2.0 * logf(p1)) * cosf(2.0f * M_PI * p2);
  float alt_n = -pos_n.z + n * sqrtf(0.006f);

  sensor_msg.pressure_alt = (std::isfinite(alt_n)) ? alt_n : -pos_n.z;
  sensor_msg.temperature = 0.0;
  sensor_msg.fields_updated = 4095;

  //gyro needed for optical flow message
  optflow_xgyro = gyro_b.x;
  optflow_ygyro = gyro_b.y;
  optflow_zgyro = gyro_b.z;

  send_mavlink_message(MAVLINK_MSG_ID_HIL_SENSOR, &sensor_msg, 200);

  // ground truth
  math::Vector3 accel_true_b = q_br.RotateVector(model_->GetRelativeLinearAccel());

  // send ground truth
  mavlink_hil_state_quaternion_t hil_state_quat;
  hil_state_quat.time_usec = world_->GetSimTime().nsec*1000;
  hil_state_quat.attitude_quaternion[0] = q_nb.w;
  hil_state_quat.attitude_quaternion[1] = q_nb.x;
  hil_state_quat.attitude_quaternion[2] = q_nb.y;
  hil_state_quat.attitude_quaternion[3] = q_nb.z;

  hil_state_quat.rollspeed = omega_nb_b.x;
  hil_state_quat.pitchspeed = omega_nb_b.y;
  hil_state_quat.yawspeed = omega_nb_b.z;

  hil_state_quat.lat = lat_rad * 180 / M_PI * 1e7;
  hil_state_quat.lon = lon_rad * 180 / M_PI * 1e7;
  hil_state_quat.alt = (-pos_n.z + alt_zurich) * 1000;

  hil_state_quat.vx = vel_n.x * 100;
  hil_state_quat.vy = vel_n.y * 100;
  hil_state_quat.vz = vel_n.z * 100;

  // assumed indicated airspeed due to flow aligned with pitot (body x)
  hil_state_quat.ind_airspeed = vel_b.x;
  hil_state_quat.true_airspeed = model_->GetWorldLinearVel().GetLength() * 100; //no wind simulated

  hil_state_quat.xacc = accel_true_b.x * 1000;
  hil_state_quat.yacc = accel_true_b.y * 1000;
  hil_state_quat.zacc = accel_true_b.z * 1000;

  send_mavlink_message(MAVLINK_MSG_ID_HIL_STATE_QUATERNION, &hil_state_quat, 200);
}

void GazeboMavlinkInterface::LidarCallback(LidarPtr& lidar_message) {
  mavlink_distance_sensor_t sensor_msg;
  sensor_msg.time_boot_ms = lidar_message->time_msec();
  sensor_msg.min_distance = lidar_message->min_distance() * 100.0;
  sensor_msg.max_distance = lidar_message->max_distance() * 100.0;
  sensor_msg.current_distance = lidar_message->current_distance() * 100.0;
  sensor_msg.type = 0;
  sensor_msg.id = 0;
  // to to roll 180 (downward facing for agl measurement)
  sensor_msg.orientation = 8;
  sensor_msg.covariance = 0;

  //distance needed for optical flow message
  optflow_distance = lidar_message->current_distance(); //[m]

  send_mavlink_message(MAVLINK_MSG_ID_DISTANCE_SENSOR, &sensor_msg, 200);

}

void GazeboMavlinkInterface::OpticalFlowCallback(OpticalFlowPtr& opticalFlow_message) {
  mavlink_hil_optical_flow_t sensor_msg;
  sensor_msg.time_usec = opticalFlow_message->time_usec();
  sensor_msg.sensor_id = opticalFlow_message->sensor_id();
  sensor_msg.integration_time_us = opticalFlow_message->integration_time_us();
  sensor_msg.integrated_x = opticalFlow_message->integrated_x();
  sensor_msg.integrated_y = opticalFlow_message->integrated_y();
  sensor_msg.integrated_xgyro = -optflow_ygyro * opticalFlow_message->integration_time_us() / 1000000.0; //xy switched
  sensor_msg.integrated_ygyro = optflow_xgyro * opticalFlow_message->integration_time_us() / 1000000.0; //xy switched
  sensor_msg.integrated_zgyro = -optflow_zgyro * opticalFlow_message->integration_time_us() / 1000000.0; //change direction
  sensor_msg.temperature = opticalFlow_message->temperature();
  sensor_msg.quality = opticalFlow_message->quality();
  sensor_msg.time_delta_distance_us = opticalFlow_message->time_delta_distance_us();
  sensor_msg.distance = optflow_distance;

  send_mavlink_message(MAVLINK_MSG_ID_HIL_OPTICAL_FLOW, &sensor_msg, 200);
}

/*ssize_t GazeboMavlinkInterface::receive(void *_buf, const size_t _size, uint32_t _timeoutMs)
{
  fd_set fds;
  struct timeval tv;

  FD_ZERO(&fds);
  FD_SET(this->handle, &fds);

  tv.tv_sec = _timeoutMs / 1000;
  tv.tv_usec = (_timeoutMs % 1000) * 1000UL;

  if (select(this->handle+1, &fds, NULL, NULL, &tv) != 1)
  {
      return -1;
  }

  return recv(this->handle, _buf, _size, 0);
}*/

void GazeboMavlinkInterface::pollForMAVLinkMessages(double _dt, uint32_t _timeoutMs)
{
  // convert timeout in ms to timeval
  struct timeval tv;
  tv.tv_sec = _timeoutMs / 1000;
  tv.tv_usec = (_timeoutMs % 1000) * 1000UL;

  // poll
  ::poll(&fds[0], (sizeof(fds[0])/sizeof(fds[0])), 0);

  if (fds[0].revents & POLLIN) {
    int len = recvfrom(_fd, _buf, sizeof(_buf), 0, (struct sockaddr *)&_srcaddr, &_addrlen);
    if (len > 0) {
      mavlink_message_t msg;
      mavlink_status_t status;
      for (unsigned i = 0; i < len; ++i)
      {
        if (mavlink_parse_char(MAVLINK_COMM_0, _buf[i], &msg, &status))
        {
          // have a message, handle it
          handle_message(&msg);
        }
      }
    }
  }
}

void GazeboMavlinkInterface::handle_message(mavlink_message_t *msg)
{
  switch(msg->msgid) {
  case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
    mavlink_hil_actuator_controls_t controls;
    mavlink_msg_hil_actuator_controls_decode(msg, &controls);
    bool armed = false;

    if ((controls.mode & MAV_MODE_FLAG_SAFETY_ARMED) > 0) {
      armed = true;
    }

    last_actuator_time_ = world_->GetSimTime();

    for (unsigned i = 0; i < n_out_max; i++) {
      input_index_[i] = i;
    }

    // set rotor speeds, controller targets
    input_reference_.resize(n_out_max);
    for (int i = 0; i < input_reference_.size(); i++) {
      if (armed) {
        input_reference_[i] = (controls.controls[input_index_[i]] + input_offset_[i])
          * input_scaling_[i] + zero_position_armed_[i];
        // if (joints_[i])
        //   gzerr << i << " : " << input_index_[i] << " : " << controls.controls[input_index_[i]] << " : " << input_reference_[i] << "\n";
      } else {
        input_reference_[i] = zero_position_disarmed_[i];
      }
    }

    received_first_referenc_ = true;
    break;
  }
}

void GazeboMavlinkInterface::handle_control(double _dt)
{
    // set joint positions
    for (int i = 0; i < input_reference_.size(); i++) {
      if (joints_[i]) {
        double target = input_reference_[i];
        if (joint_control_type_[i] == "velocity")
        {
          double current = joints_[i]->GetVelocity(0);
          double err = current - target;
          double force = pids_[i].Update(err, _dt);
          joints_[i]->SetForce(0, force);
          // gzerr << "chan[" << i << "] curr[" << current
          //       << "] cmd[" << target << "] f[" << force
          //       << "] scale[" << input_scaling_[i] << "]\n";
        }
        else if (joint_control_type_[i] == "position")
        {
          double current = joints_[i]->GetAngle(0).Radian();
          double err = current - target;
          double force = pids_[i].Update(err, _dt);
          joints_[i]->SetForce(0, force);
          // gzerr << "chan[" << i << "] curr[" << current
          //       << "] cmd[" << target << "] f[" << force
          //       << "] scale[" << input_scaling_[i] << "]\n";
        }
        else if (joint_control_type_[i] == "position_gztopic")
        {
          #if GAZEBO_MAJOR_VERSION >= 7 && GAZEBO_MINOR_VERSION >= 4
            /// only gazebo 7.4 and above support Any
            gazebo::msgs::Any m;
            m.set_type(gazebo::msgs::Any_ValueType_DOUBLE);
            m.set_double_value(target);
          #else
            std::stringstream ss;
            gazebo::msgs::GzString m;
            ss << target;
            m.set_data(ss.str());
          #endif
          joint_control_pub_[i]->Publish(m);
        }
        else if (joint_control_type_[i] == "position_kinematic")
        {
          /// really not ideal if your drone is moving at all,
          /// mixing kinematic updates with dynamics calculation is
          /// non-physical.
          #if GAZEBO_MAJOR_VERSION >= 6
            joints_[i]->SetPosition(0, input_reference_[i]);
          #else
            joints_[i]->SetAngle(0, input_reference_[i]);
          #endif
        }
        else
        {
          gzerr << "joint_control_type[" << joint_control_type_[i] << "] undefined.\n";
        }
      }
    }
}

}

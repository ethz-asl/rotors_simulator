/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015-2018 PX4 Pro Development Team
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

#include <rotors_gazebo_plugins/gazebo_mavlink_interface.h>

namespace gazebo {

// Set global reference point
// Zurich Irchel Park: 47.397742, 8.545594, 488m
// Seattle downtown (15 deg declination): 47.592182, -122.316031, 86m
// Moscow downtown: 55.753395, 37.625427, 155m

// Zurich Irchel Park
static const double kLatZurich_rad = 47.397742 * M_PI / 180;  // rad
static const double kLonZurich_rad = 8.545594 * M_PI / 180;   // rad
static const double kAltZurich_m = 488.0;                     // meters
static const float kEarthRadius_m = 6353000;                  // m

GZ_REGISTER_MODEL_PLUGIN(GazeboMavlinkInterface);

GazeboMavlinkInterface::~GazeboMavlinkInterface() {
  updateConnection_->~Connection();
}

void GazeboMavlinkInterface::Load(
    physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model.
  model_ = _model;

  world_ = model_->GetWorld();

  const char* env_alt = std::getenv("PX4_HOME_ALT");
  if (env_alt) {
    gzmsg << "Home altitude is set to " << env_alt << ".\n";
    alt_home = std::stod(env_alt);
  }

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_mavlink_interface] Please specify a robotNamespace.\n";
  }

  if (_sdf->HasElement("protocol_version")) {
    protocol_version_ = _sdf->GetElement("protocol_version")->Get<float>();
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init();

  getSdfParam<std::string>(
      _sdf, "motorSpeedCommandPubTopic", motor_velocity_reference_pub_topic_,
      motor_velocity_reference_pub_topic_);
  gzdbg << "motorSpeedCommandPubTopic = \""
        << motor_velocity_reference_pub_topic_ << "\"." << std::endl;
  getSdfParam<std::string>(_sdf, "imuSubTopic", imu_sub_topic_, imu_sub_topic_);
  getSdfParam<std::string>(
      _sdf, "lidarSubTopic", lidar_sub_topic_, lidar_sub_topic_);
  getSdfParam<std::string>(
      _sdf, "opticalFlowSubTopic", opticalFlow_sub_topic_,
      opticalFlow_sub_topic_);

  // set input_reference_ from inputs.control
  input_reference_.resize(kNOutMax);
  joints_.resize(kNOutMax);
  pids_.resize(kNOutMax);
  for (int i = 0; i < kNOutMax; ++i) {
    pids_[i].Init(0, 0, 0, 0, 0, 0, 0);
    input_reference_[i] = 0;
  }

  if (_sdf->HasElement("control_channels")) {
    sdf::ElementPtr control_channels = _sdf->GetElement("control_channels");
    sdf::ElementPtr channel = control_channels->GetElement("channel");
    while (channel) {
      if (channel->HasElement("input_index")) {
        int index = channel->Get<int>("input_index");
        if (index < kNOutMax) {
          input_offset_[index] = channel->Get<double>("input_offset");
          input_scaling_[index] = channel->Get<double>("input_scaling");
          zero_position_disarmed_[index] =
              channel->Get<double>("zero_position_disarmed");
          zero_position_armed_[index] =
              channel->Get<double>("zero_position_armed");
          if (channel->HasElement("joint_control_type")) {
            joint_control_type_[index] =
                channel->Get<std::string>("joint_control_type");
          } else {
            gzwarn << "joint_control_type[" << index
                   << "] not specified, using velocity.\n";
            joint_control_type_[index] = "velocity";
          }

          // start gz transport node handle
          if (joint_control_type_[index] == "position_gztopic") {
            // setup publisher handle to topic
            if (channel->HasElement("gztopic"))
              gztopic_[index] = "~/" + model_->GetName() +
                                channel->Get<std::string>("gztopic");
            else
              gztopic_[index] =
                  "control_position_gztopic_" + std::to_string(index);
                  
            joint_control_pub_[index] =
                node_handle_->Advertise<gazebo::msgs::Any>(gztopic_[index]);
          }

          if (channel->HasElement("joint_name")) {
            std::string joint_name = channel->Get<std::string>("joint_name");
            joints_[index] = model_->GetJoint(joint_name);
            if (joints_[index] == nullptr) {
              gzwarn << "joint [" << joint_name << "] not found for channel["
                     << index << "] no joint control for this channel.\n";
            } else {
              gzdbg << "joint [" << joint_name << "] found for channel["
                    << index << "] joint control active for this channel.\n";
            }
          } else {
            gzdbg << "<joint_name> not found for channel[" << index
                  << "] no joint control will be performed for this channel.\n";
          }

          // setup joint control pid to control joint
          if (channel->HasElement("joint_control_pid")) {
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
        } else {
          gzerr << "input_index[" << index << "] out of range, not parsing.\n";
        }
      } else {
        gzerr << "no input_index, not parsing.\n";
        break;
      }
      channel = channel->GetNextElement("channel");
    }
  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboMavlinkInterface::OnUpdate, this, _1));

  // Subscriber to IMU sensor_msgs::Imu Message and SITL message
  imu_sub_ = node_handle_->Subscribe(
      "~/" + model_->GetName() + imu_sub_topic_,
      &GazeboMavlinkInterface::ImuCallback, this);
  lidar_sub_ = node_handle_->Subscribe(
      "~/" + model_->GetName() + lidar_sub_topic_,
      &GazeboMavlinkInterface::LidarCallback, this);
  opticalFlow_sub_ = node_handle_->Subscribe(
      "~/" + model_->GetName() + opticalFlow_sub_topic_,
      &GazeboMavlinkInterface::OpticalFlowCallback, this);

  // Publish gazebo's motor_speed message
  motor_velocity_reference_pub_ =
      node_handle_->Advertise<gz_mav_msgs::CommandMotorSpeed>(
          "~/" + model_->GetName() + motor_velocity_reference_pub_topic_, 1);

  _rotor_count = 5;

  last_time_ = world_->SimTime();
  last_imu_time_ = world_->SimTime();
  gravity_W_ = world_->Gravity();

  if (_sdf->HasElement("imu_rate")) {
    imu_update_interval_ = 1.0 / _sdf->GetElement("imu_rate")->Get<int>();
  }

  // Magnetic field data for Zurich from WMM2015 (10^5xnanoTesla (N, E D)
  // n-frame ) mag_n_ = {0.21523, 0.00771, -0.42741}; We set the world Y
  // component to zero because we apply the declination based on the global
  // position, and so we need to start without any offsets. The real value for
  // Zurich would be 0.00771 frame d is the magnetic north frame
  mag_d_.X() = 0.21523;
  mag_d_.Y() = 0;
  mag_d_.Z() = -0.42741;

  if (_sdf->HasElement("hil_state_level")) {
    hil_mode_ = _sdf->GetElement("hil_mode")->Get<bool>();
  }

  if (_sdf->HasElement("hil_state_level")) {
    hil_state_level_ = _sdf->GetElement("hil_state_level")->Get<bool>();
  }

  // Get serial params
  if (_sdf->HasElement("serialEnabled")) {
    serial_enabled_ = _sdf->GetElement("serialEnabled")->Get<bool>();
  }

  if (serial_enabled_) {
    // Set up serial interface
    if (_sdf->HasElement("serialDevice")) {
      device_ = _sdf->GetElement("serialDevice")->Get<std::string>();
    }

    if (_sdf->HasElement("baudRate")) {
      baudrate_ = _sdf->GetElement("baudRate")->Get<int>();
    }
    io_service_.post(std::bind(&GazeboMavlinkInterface::do_read, this));

    // run io_service for async io
    io_thread_ = std::thread([this]() { io_service_.run(); });
    open();
  }

  // Create socket
  // udp socket data
  mavlink_addr_ = htonl(INADDR_ANY);
  if (_sdf->HasElement("mavlink_addr")) {
    std::string mavlink_addr =
        _sdf->GetElement("mavlink_addr")->Get<std::string>();
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

  auto worldName = world_->Name();
  model_param(worldName, model_->GetName(), "mavlink_udp_port",
  mavlink_udp_port_);

  qgc_addr_ = htonl(INADDR_ANY);
  if (_sdf->HasElement("qgc_addr")) {
    std::string qgc_addr = _sdf->GetElement("qgc_addr")->Get<std::string>();
    if (qgc_addr != "INADDR_ANY") {
      qgc_addr_ = inet_addr(qgc_addr.c_str());
      if (qgc_addr_ == INADDR_NONE) {
        fprintf(stderr, "invalid qgc_addr \"%s\"\n", qgc_addr.c_str());
        return;
      }
    }
  }
  if (_sdf->HasElement("qgc_udp_port")) {
    qgc_udp_port_ = _sdf->GetElement("qgc_udp_port")->Get<int>();
  }

  // try to setup udp socket for communcation
  if ((_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    printf("create socket failed\n");
    return;
  }

  memset((char*)&myaddr_, 0, sizeof(myaddr_));
  myaddr_.sin_family = AF_INET;
  srcaddr_.sin_family = AF_INET;

  if (serial_enabled_) {
    // gcs link
    myaddr_.sin_addr.s_addr = mavlink_addr_;
    myaddr_.sin_port = htons(mavlink_udp_port_);
    srcaddr_.sin_addr.s_addr = qgc_addr_;
    srcaddr_.sin_port = htons(qgc_udp_port_);
  }

  else {
    myaddr_.sin_addr.s_addr = htonl(INADDR_ANY);
    // Let the OS pick the port
    myaddr_.sin_port = htons(0);
    srcaddr_.sin_addr.s_addr = mavlink_addr_;
    srcaddr_.sin_port = htons(mavlink_udp_port_);
  }

  addrlen_ = sizeof(srcaddr_);

  if (bind(_fd, (struct sockaddr*)&myaddr_, sizeof(myaddr_)) < 0) {
    printf("bind failed\n");
    return;
  }

  fds[0].fd = _fd;
  fds[0].events = POLLIN;

  mavlink_status_t* chan_state = mavlink_get_channel_status(MAVLINK_COMM_0);

  // set the Mavlink protocol version to use on the link
  if (protocol_version_ == 2.0) {
    chan_state->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
    gzmsg << "Using MAVLink protocol v2.0\n";
  } else if (protocol_version_ == 1.0) {
    chan_state->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    gzmsg << "Using MAVLink protocol v1.0\n";
  } else {
    gzerr << "Unkown protocol version! Using v" << protocol_version_
          << "by default \n";
  }
}

// This gets called by the world update start event.
void GazeboMavlinkInterface::OnUpdate(const common::UpdateInfo& /*_info*/) {
  common::Time current_time = world_->SimTime();
  double dt = (current_time - last_time_).Double();

  pollForMAVLinkMessages(dt, 1000);

  handle_control(dt);

  if (received_first_reference_) {
    gz_mav_msgs::CommandMotorSpeed turning_velocities_msg;

    for (int i = 0; i < input_reference_.size(); i++) {
      if (last_actuator_time_ == 0 ||
          (current_time - last_actuator_time_).Double() > 0.2) {
        turning_velocities_msg.add_motor_speed(0);
      } else {
        turning_velocities_msg.add_motor_speed(input_reference_[i]);
      }
    }
    // TODO Add timestamp and Header
    // turning_velocities_msg->header.stamp.sec = current_time.sec;
    // turning_velocities_msg->header.stamp.nsec = current_time.nsec;

    motor_velocity_reference_pub_->Publish(turning_velocities_msg);
  }

  last_time_ = current_time;
}

void GazeboMavlinkInterface::send_mavlink_message(
    const mavlink_message_t* message, const int destination_port) {
  if (serial_enabled_ && destination_port == 0) {
    assert(message != nullptr);
    if (!is_open()) {
      gzerr << "Serial port closed! \n";
      return;
    }

    {
      lock_guard lock(mutex_);

      if (tx_q_.size() >= MAX_TXQ_SIZE) {
        //         gzwarn << "TX queue overflow. \n";
      }
      tx_q_.emplace_back(message);
    }
    io_service_.post(std::bind(&GazeboMavlinkInterface::do_write, this, true));
  }

  else {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int packetlen = mavlink_msg_to_send_buffer(buffer, message);

    struct sockaddr_in dest_addr;
    memcpy(&dest_addr, &srcaddr_, sizeof(srcaddr_));

    if (destination_port != 0) {
      dest_addr.sin_port = htons(destination_port);
    }

    ssize_t len = sendto(
        _fd, buffer, packetlen, 0, (struct sockaddr*)&srcaddr_,
        sizeof(srcaddr_));

    if (len <= 0) {
      printf("Failed sending mavlink message\n");
    }
  }
}

void GazeboMavlinkInterface::ImuCallback(ImuPtr& imu_message) {
  common::Time current_time = world_->SimTime();
  double dt = (current_time - last_imu_time_).Double();

  ignition::math::Quaterniond q_br(0, 1, 0, 0);
  ignition::math::Quaterniond q_ng(0, 0.70711, 0.70711, 0);

  ignition::math::Quaterniond q_gr = ignition::math::Quaterniond(
      imu_message->orientation().w(), imu_message->orientation().x(),
      imu_message->orientation().y(), imu_message->orientation().z());

  ignition::math::Quaterniond q_gb = q_gr * q_br.Inverse();
  ignition::math::Quaterniond q_nb = q_ng * q_gb;

  ignition::math::Vector3d pos_g = model_->WorldPose().Pos();
  ignition::math::Vector3d pos_n = q_ng.RotateVector(pos_g);

  float declination = get_mag_declination(lat_rad_, lon_rad_);

  ignition::math::Quaterniond q_dn(0.0, 0.0, declination);
  ignition::math::Vector3d mag_n = q_dn.RotateVector(mag_d_);

  ignition::math::Vector3d vel_b =
      q_br.RotateVector(model_->RelativeLinearVel());
  ignition::math::Vector3d vel_n = q_ng.RotateVector(model_->WorldLinearVel());
  ignition::math::Vector3d omega_nb_b =
      q_br.RotateVector(model_->RelativeAngularVel());

  ignition::math::Vector3d mag_noise_b(
      0.01 * randn_(rand_), 0.01 * randn_(rand_), 0.01 * randn_(rand_));

  ignition::math::Vector3d accel_b = q_br.RotateVector(ignition::math::Vector3d(
      imu_message->linear_acceleration().x(),
      imu_message->linear_acceleration().y(),
      imu_message->linear_acceleration().z()));
  ignition::math::Vector3d gyro_b = q_br.RotateVector(ignition::math::Vector3d(
      imu_message->angular_velocity().x(), imu_message->angular_velocity().y(),
      imu_message->angular_velocity().z()));
  ignition::math::Vector3d mag_b =
      q_nb.RotateVectorReverse(mag_n) + mag_noise_b;

  if (imu_update_interval_ != 0 && dt >= imu_update_interval_) {
    mavlink_hil_sensor_t sensor_msg;
    sensor_msg.time_usec = world_->SimTime().Double() * 1e6;
    sensor_msg.xacc = accel_b.X();
    sensor_msg.yacc = accel_b.Y();
    sensor_msg.zacc = accel_b.Z();
    sensor_msg.xgyro = gyro_b.X();
    sensor_msg.ygyro = gyro_b.Y();
    sensor_msg.zgyro = gyro_b.Z();
    sensor_msg.xmag = mag_b.X();
    sensor_msg.ymag = mag_b.Y();
    sensor_msg.zmag = mag_b.Z();

    // calculate abs_pressure using an ISA model for the tropsphere (valid up to
    // 11km above MSL)
    const float lapse_rate =
        0.0065f;  // reduction in temperature with altitude (Kelvin/m)
    const float temperature_msl = 288.0f;  // temperature at MSL (Kelvin)
    float alt_msl = (float)alt_home - pos_n.Z();
    float temperature_local = temperature_msl - lapse_rate * alt_msl;
    float pressure_ratio = powf((temperature_msl / temperature_local), 5.256f);
    const float pressure_msl = 101325.0f;  // pressure at MSL
    sensor_msg.abs_pressure = pressure_msl / pressure_ratio;

    // generate Gaussian noise sequence using polar form of Box-Muller
    // transformation http://www.design.caltech.edu/erik/Misc/Gaussian.html
    double x1, x2, w, y1, y2;
    do {
      x1 = 2.0 * (rand() * (1.0 / (double)RAND_MAX)) - 1.0;
      x2 = 2.0 * (rand() * (1.0 / (double)RAND_MAX)) - 1.0;
      w = x1 * x1 + x2 * x2;
    } while (w >= 1.0);
    w = sqrt((-2.0 * log(w)) / w);
    y1 = x1 * w;
    y2 = x2 * w;

    // Apply 1 Pa RMS noise
    float abs_pressure_noise = 1.0f * (float)w;
    sensor_msg.abs_pressure += abs_pressure_noise;

    // convert to hPa
    sensor_msg.abs_pressure *= 0.01f;

    // calculate density using an ISA model for the tropsphere (valid up to 11km
    // above MSL)
    const float density_ratio =
        powf((temperature_msl / temperature_local), 4.256f);
    float rho = 1.225f / density_ratio;

    // calculate pressure altitude including effect of pressure noise
    sensor_msg.pressure_alt =
        alt_msl - abs_pressure_noise / (gravity_W_.Length() * rho);

    // calculate differential pressure in hPa
    sensor_msg.diff_pressure = 0.005f * rho * vel_b.X() * vel_b.X();

    // calculate temperature in Celsius
    sensor_msg.temperature = temperature_local - 273.0f;

    sensor_msg.fields_updated = 4095;

    // accumulate gyro measurements that are needed for the optical flow message
    static uint32_t last_dt_us = sensor_msg.time_usec;
    uint32_t dt_us = sensor_msg.time_usec - last_dt_us;
    if (dt_us > 1000) {
      optflow_gyro_ += gyro_b * (dt_us / 1000000.0f);
      last_dt_us = sensor_msg.time_usec;
    }

    mavlink_message_t msg;
    mavlink_msg_hil_sensor_encode_chan(
        1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
    if (hil_mode_) {
      if (!hil_state_level_) {
        send_mavlink_message(&msg);
      }
    }

    else {
      send_mavlink_message(&msg);
    }
    last_imu_time_ = current_time;
  }

  // ground truth
  ignition::math::Vector3d accel_true_b =
      q_br.RotateVector(model_->RelativeLinearAccel());

  // send ground truth
  mavlink_hil_state_quaternion_t hil_state_quat;
  hil_state_quat.time_usec = world_->SimTime().Double() * 1e6;
  hil_state_quat.attitude_quaternion[0] = q_nb.W();
  hil_state_quat.attitude_quaternion[1] = q_nb.X();
  hil_state_quat.attitude_quaternion[2] = q_nb.Y();
  hil_state_quat.attitude_quaternion[3] = q_nb.Z();

  hil_state_quat.rollspeed = omega_nb_b.X();
  hil_state_quat.pitchspeed = omega_nb_b.Y();
  hil_state_quat.yawspeed = omega_nb_b.Z();

  hil_state_quat.lat = lat_rad_ * 180 / M_PI * 1e7;
  hil_state_quat.lon = lon_rad_ * 180 / M_PI * 1e7;
  hil_state_quat.alt = (-pos_n.Z() + kAltZurich_m) * 1000;

  hil_state_quat.vx = vel_n.X() * 100;
  hil_state_quat.vy = vel_n.Y() * 100;
  hil_state_quat.vz = vel_n.Z() * 100;

  // assumed indicated airspeed due to flow aligned with pitot (body x)
  hil_state_quat.ind_airspeed = vel_b.X();
  hil_state_quat.true_airspeed =
      model_->WorldLinearVel().Length() * 100;  // no wind simulated

  hil_state_quat.xacc = accel_true_b.X() * 1000;
  hil_state_quat.yacc = accel_true_b.Y() * 1000;
  hil_state_quat.zacc = accel_true_b.Z() * 1000;

  mavlink_message_t msg;
  mavlink_msg_hil_state_quaternion_encode_chan(
      1, 200, MAVLINK_COMM_0, &msg, &hil_state_quat);
  if (hil_mode_) {
    if (hil_state_level_) {
      send_mavlink_message(&msg);
    }
  }

  else {
    send_mavlink_message(&msg);
  }
}

void GazeboMavlinkInterface::LidarCallback(LidarPtr& lidar_message) {
  mavlink_distance_sensor_t sensor_msg;
  sensor_msg.time_boot_ms = lidar_message->time_msec();
  sensor_msg.min_distance = lidar_message->min_distance() * 100.0;
  sensor_msg.max_distance = lidar_message->max_distance() * 100.0;
  sensor_msg.current_distance = lidar_message->current_distance() * 100.0;
  sensor_msg.type = 0;
  sensor_msg.id = 0;
  sensor_msg.orientation = 25;  // downward facing
  sensor_msg.covariance = 0;

  // distance needed for optical flow message
  optflow_distance_ = lidar_message->current_distance();  //[m]

  mavlink_message_t msg;
  mavlink_msg_distance_sensor_encode_chan(
      1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
  send_mavlink_message(&msg);
}

void GazeboMavlinkInterface::OpticalFlowCallback(
    OpticalFlowPtr& opticalFlow_message) {
  mavlink_hil_optical_flow_t sensor_msg;
  sensor_msg.time_usec = world_->SimTime().Double() * 1e6;
  sensor_msg.sensor_id = opticalFlow_message->sensor_id();
  sensor_msg.integration_time_us = opticalFlow_message->integration_time_us();
  sensor_msg.integrated_x = opticalFlow_message->integrated_x();
  sensor_msg.integrated_y = opticalFlow_message->integrated_y();
  sensor_msg.integrated_xgyro =
      opticalFlow_message->quality() ? -optflow_gyro_.Y() : 0.0f;  // xy switched
  sensor_msg.integrated_ygyro =
      opticalFlow_message->quality() ? optflow_gyro_.X() : 0.0f;  // xy switched
  sensor_msg.integrated_zgyro = opticalFlow_message->quality()
                                    ? -optflow_gyro_.Z()
                                    : 0.0f;  // change direction
  sensor_msg.temperature = opticalFlow_message->temperature();
  sensor_msg.quality = opticalFlow_message->quality();
  sensor_msg.time_delta_distance_us =
      opticalFlow_message->time_delta_distance_us();
  sensor_msg.distance = optflow_distance_;

  // reset gyro integral
  optflow_gyro_.Set();

  mavlink_message_t msg;
  mavlink_msg_hil_optical_flow_encode_chan(
      1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
  send_mavlink_message(&msg);
}

void GazeboMavlinkInterface::pollForMAVLinkMessages(
    double _dt, uint32_t _timeoutMs) {
  // convert timeout in ms to timeval
  struct timeval tv;
  tv.tv_sec = _timeoutMs / 1000;
  tv.tv_usec = (_timeoutMs % 1000) * 1000UL;

  // poll
  ::poll(&fds[0], (sizeof(fds[0]) / sizeof(fds[0])), 0);

  if (fds[0].revents & POLLIN) {
    int len = recvfrom(
        _fd, buf_, sizeof(buf_), 0, (struct sockaddr*)&srcaddr_, &addrlen_);
    if (len > 0) {
      mavlink_message_t msg;
      mavlink_status_t status;
      for (unsigned i = 0; i < len; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buf_[i], &msg, &status)) {
          if (serial_enabled_) {
            // forward message from qgc to serial
            send_mavlink_message(&msg);
          }
          // have a message, handle it
          handle_message(&msg);
        }
      }
    }
  }
}

void GazeboMavlinkInterface::handle_message(mavlink_message_t* msg) {
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
      mavlink_hil_actuator_controls_t controls;
      mavlink_msg_hil_actuator_controls_decode(msg, &controls);
      bool armed = false;

      if ((controls.mode & MAV_MODE_FLAG_SAFETY_ARMED) > 0) {
        armed = true;
      }

      last_actuator_time_ = world_->SimTime();

      for (unsigned i = 0; i < kNOutMax; i++) {
        input_index_[i] = i;
      }

      // Set rotor speeds and controller targets for flagged messages.
      if (controls.flags == kMotorSpeedFlag) {
        input_reference_.resize(kNOutMax);
        for (unsigned i = 0; i < kNumMotors; ++i) {
          if (armed) {
            input_reference_[i] =
                (controls.controls[input_index_[i]] + input_offset_[i]) *
                    input_scaling_[i] +
                zero_position_armed_[i];
          } else {
            input_reference_[i] = zero_position_disarmed_[i];
          }
        }
        received_first_reference_ = true;
      } else if (controls.flags == kServoPositionFlag) {
        for (unsigned i = kNumMotors; i < (kNumMotors + kNumServos); ++i) {
          if (armed) {
            input_reference_[i] =
                (controls.controls[input_index_[i - kNumMotors]] +
                 input_offset_[i]) *
                    input_scaling_[i] +
                zero_position_armed_[i];
          } else {
            input_reference_[i] = zero_position_disarmed_[i];
          }
        }
      }
      // Set rotor speeds, controller targets for unflagged messages.
      else {
        input_reference_.resize(kNOutMax);
        for (unsigned i = 0; i < kNOutMax; ++i) {
          if (armed) {
            input_reference_[i] =
                (controls.controls[input_index_[i]] + input_offset_[i]) *
                    input_scaling_[i] +
                zero_position_armed_[i];
          } else {
            input_reference_[i] = zero_position_disarmed_[i];
          }
        }
        received_first_reference_ = true;
      }
      break;
  }
}

void GazeboMavlinkInterface::handle_control(double _dt) {
  // set joint positions
  for (int i = 0; i < input_reference_.size(); i++) {
    if (joints_[i]) {
      double target = input_reference_[i];
      if (joint_control_type_[i] == "velocity") {
        double current = joints_[i]->GetVelocity(0);
        double err = current - target;
        double force = pids_[i].Update(err, _dt);
        joints_[i]->SetForce(0, force);
      } else if (joint_control_type_[i] == "position") {
        double current = joints_[i]->Position(0);
        double err = current - target;
        double force = pids_[i].Update(err, _dt);
        joints_[i]->SetForce(0, force);
      } else if (joint_control_type_[i] == "position_gztopic") {
        gazebo::msgs::Any m;
        m.set_type(gazebo::msgs::Any_ValueType_DOUBLE);
        m.set_double_value(target);
        joint_control_pub_[i]->Publish(m);
      } else if (joint_control_type_[i] == "position_kinematic") {
        /// really not ideal if your drone is moving at all,
        /// mixing kinematic updates with dynamics calculation is
        /// non-physical.

        joints_[i]->SetPosition(0, input_reference_[i]);

      } else {
        gzerr << "joint_control_type[" << joint_control_type_[i]
              << "] undefined.\n";
      }
    }
  }
}

void GazeboMavlinkInterface::open() {
  try {
    serial_dev_.open(device_);
    serial_dev_.set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
    serial_dev_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_dev_.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
    serial_dev_.set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one));
    serial_dev_.set_option(boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none));
    gzdbg << "Opened serial device " << device_ << "\n";
  } catch (boost::system::system_error& err) {
    gzerr << "Error opening serial device: " << err.what() << "\n";
  }
}

void GazeboMavlinkInterface::close() {
  lock_guard lock(mutex_);
  if (!is_open())
    return;

  io_service_.stop();
  serial_dev_.close();

  if (io_thread_.joinable())
    io_thread_.join();
}

void GazeboMavlinkInterface::do_read(void) {
  serial_dev_.async_read_some(
      boost::asio::buffer(rx_buf_),
      boost::bind(
          &GazeboMavlinkInterface::parse_buffer, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
}

// Based on MAVConnInterface::parse_buffer in MAVROS
void GazeboMavlinkInterface::parse_buffer(
    const boost::system::error_code& err, std::size_t bytes_t) {
  mavlink_status_t status;
  mavlink_message_t message;
  uint8_t* buf = this->rx_buf_.data();

  assert(rx_buf_.size() >= bytes_t);

  for (; bytes_t > 0; bytes_t--) {
    auto c = *buf++;

    auto msg_received = static_cast<Framing>(
        mavlink_frame_char_buffer(&m_buffer_, &m_status_, c, &message, &status));
    if (msg_received == Framing::bad_crc ||
        msg_received == Framing::bad_signature) {
      _mav_parse_error(&m_status_);
      m_status_.msg_received = MAVLINK_FRAMING_INCOMPLETE;
      m_status_.parse_state = MAVLINK_PARSE_STATE_IDLE;
      if (c == MAVLINK_STX) {
        m_status_.parse_state = MAVLINK_PARSE_STATE_GOT_STX;
        m_buffer_.len = 0;
        mavlink_start_checksum(&m_buffer_);
      }
    }

    if (msg_received != Framing::incomplete) {
      // send to gcs
      send_mavlink_message(&message, qgc_udp_port_);
      handle_message(&message);
    }
  }
  do_read();
}

void GazeboMavlinkInterface::do_write(bool check_tx_state) {
  if (check_tx_state && tx_in_progress_)
    return;

  lock_guard lock(mutex_);
  if (tx_q_.empty())
    return;

  tx_in_progress_ = true;
  auto& buf_ref = tx_q_.front();

  serial_dev_.async_write_some(
      boost::asio::buffer(buf_ref.dpos(), buf_ref.nbytes()),
      [this, &buf_ref](
          boost::system::error_code error, size_t bytes_transferred) {
        assert(bytes_transferred <= buf_ref.len);
        if (error) {
          gzerr << "Serial error: " << error.message() << "\n";
          return;
        }

        lock_guard lock(mutex_);

        if (tx_q_.empty()) {
          tx_in_progress_ = false;
          return;
        }

        buf_ref.pos += bytes_transferred;
        if (buf_ref.nbytes() == 0) {
          tx_q_.pop_front();
        }

        if (!tx_q_.empty()) {
          do_write(false);
        } else {
          tx_in_progress_ = false;
        }
      });
}

}  // namespace gazebo

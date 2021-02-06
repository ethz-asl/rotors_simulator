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
  static const double kLonZurich_rad = 8.545594 * M_PI / 180;  // rad
  static const double kAltZurich_m = 488.0; // meters
  static const float kEarthRadius_m = 6353000;  // m

GZ_REGISTER_MODEL_PLUGIN(GazeboMavlinkInterface);

GazeboMavlinkInterface::~GazeboMavlinkInterface() {
    delete[] channels;
    updateConnection_->~Connection();
}

void GazeboMavlinkInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model.
  model_ = _model;

  world_ = model_->GetWorld();

  const char *env_alt = std::getenv("PX4_HOME_ALT");
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

  getSdfParam<std::string>(_sdf, "motorSpeedCommandPubTopic", motor_velocity_reference_pub_topic_,
      motor_velocity_reference_pub_topic_);
  getSdfParam<std::string>(_sdf, "actuatorsPubTopic", actuators_reference_pub_topic_,
      actuators_reference_pub_topic_);

  gzdbg << "motorSpeedCommandPubTopic = \"" << motor_velocity_reference_pub_topic_ << "\"." << std::endl;
  gzdbg << "actuatorsPubTopic = \"" << actuators_reference_pub_topic_ << "\"." << std::endl;

  getSdfParam<std::string>(_sdf, "imuSubTopic", imu_sub_topic_, imu_sub_topic_);
  getSdfParam<std::string>(_sdf, "lidarSubTopic", lidar_sub_topic_, lidar_sub_topic_);
  getSdfParam<std::string>(_sdf, "opticalFlowSubTopic", opticalFlow_sub_topic_, opticalFlow_sub_topic_);
  getSdfParam<std::string>(_sdf, "gpsSubTopic", gps_sub_topic_, gps_sub_topic_);
  getSdfParam<std::string>(_sdf, "gpsGtSubTopic", gps_gt_sub_topic_, gps_gt_sub_topic_);

  bool use_vane = false;
  if (_sdf->HasElement("vaneSubTopic")) {
    vane_sub_topic_ = _sdf->GetElement("vaneSubTopic")->Get<std::string>();
    use_vane = true;
  }

  // set input_reference_ from inputs.control
  /*
  input_reference_.resize(n_out_max);
  joints_.resize(n_out_max);
  pids_.resize(n_out_max);
  for (int i = 0; i < n_out_max; ++i)
  {
    pids_[i].Init(0, 0, 0, 0, 0, 0, 0);
    input_reference_[i] = 0;
  }
  */

  n_chan = 0;
  if (_sdf->HasElement("control_channels")) {
    sdf::ElementPtr control_channels = _sdf->GetElement("control_channels");
    sdf::ElementPtr channel = control_channels->GetElement("channel");

    while (channel)
    {
        ++n_chan;
        channel = channel->GetNextElement("channel");
    }

    channels = new ctrl_chan[n_chan];
    channel = control_channels->GetElement("channel");

    for(int i=0; i<n_chan; i++)
    {
      if (channel->HasElement("input_index"))
      {

        channels[i].input_index_ = channel->Get<int>("input_index");
        //int index = channel->Get<int>("input_index");
        if (channels[i].input_index_ < n_out_max)
        {
          channels[i].input_offset_ = channel->Get<double>("input_offset");
          channels[i].input_scaling_ = channel->Get<double>("input_scaling");
          channels[i].zero_position_disarmed_ = channel->Get<double>("zero_position_disarmed");
          channels[i].zero_position_armed_ = channel->Get<double>("zero_position_armed");
          if (channel->HasElement("joint_control_type")) {
            channels[i].joint_control_type_ = channel->Get<std::string>("joint_control_type");
            gzdbg<<"joint_control_type: "<<channels[i].joint_control_type_<<"\n";
          } else {
            gzwarn << "joint_control_type of" << channel->GetName() << " channel not specified, using velocity.\n";
            channels[i].joint_control_type_ = "velocity";
          }

          if (channel->HasElement("servo_params")) {
            sdf::ElementPtr servo_params = channel->GetElement("servo_params");
            if (servo_params->HasElement("slew"))
                channels[i].srv.slew = servo_params->Get<double>("slew");
            if (servo_params->HasElement("p"))
                channels[i].srv.P_pos = servo_params->Get<double>("p");
            if (servo_params->HasElement("d"))
                channels[i].srv.P_vel = servo_params->Get<double>("d");
          }

          // setup publisher handle to topic
          if (channel->HasElement("gztopic"))
              channels[i].gztopic_ = "~/" + namespace_ + "/" + channel->Get<std::string>("gztopic");
          else
              channels[i].gztopic_ = "control_gz_msg_" + std::to_string(i);

          channels[i].joint_control_pub_ = node_handle_->Advertise<gz_std_msgs::Float32>(channels[i].gztopic_);
          std::cout<<"\33[1m[gazebo_mavlink_interface]\33[0m Advertising \33[1;32m" << channels[i].joint_control_pub_->GetTopic() << "\33[0m gazebo message\n";

          if (channel->HasElement("joint_name"))
          {
              channels[i].joint_name = channel->Get<std::string>("joint_name");
              channels[i].joint_ = model_->GetJoint(channels[i].joint_name);

              if (channels[i].joint_ == nullptr) {
                  gzwarn << "joint [" << channels[i].joint_name << "] not found for channel["
                         << channels[i].input_index_ << "] no joint control for this channel.\n";
              } else {
                  gzdbg << "joint [" << channels[i].joint_name << "] found for channel["
                        << channels[i].input_index_ << "] joint control active for this channel.\n";
              }
          } else if(channels[i].joint_control_type_ != "gz_msg") {
              gzdbg << "<joint_name> not found for channel[" << channels[i].input_index_
                    << "] no joint control will be performed for this channel.\n";
          }

          // setup joint control pid to control joint
          if (channel->HasElement("joint_control_pid")) {
            sdf::ElementPtr pid = channel->GetElement("joint_control_pid");
            double p_gain = 0;
            if (pid->HasElement("p"))
              p_gain = pid->Get<double>("p");
            double i_gain = 0;
            if (pid->HasElement("i"))
              i_gain = pid->Get<double>("i");
            double d_gain = 0;
            if (pid->HasElement("d"))
              d_gain = pid->Get<double>("d");
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

            channels[i].pid_.Init(p_gain, i_gain, d_gain, iMax, iMin, cmdMax, cmdMin);
          }

        } else {
          gzerr << "input_index[" << channels[i].input_index_ << "] out of range, not parsing.\n";
        }
      } else {
        gzerr << "no input_index, not parsing.\n";
        break;
      }
      channel = channel->GetNextElement("channel");
    }
  }

  n_wind = 0;
  if (_sdf->HasElement("wind")) {
      sdf::ElementPtr _sdf_wind= _sdf->GetElement("wind");

      // get number of winds for this particular segment
      while (_sdf_wind) {
          _sdf_wind = _sdf_wind->GetNextElement("wind");
          ++n_wind;
      }

      gzdbg<<"found "<<n_wind<<" wind(s) affecting airspeed. \n";
      _sdf_wind = _sdf->GetElement("wind");
      wind = new Wind [n_wind];

      for(int j=0; j<n_wind; j++){

          if(_sdf_wind->HasElement("topic")){
              wind[j].wind_topic = _sdf_wind->Get<std::string>("topic");
          } else {
              gzwarn<<"wind ["<<j<<"] is missing 'topic' element \n";
          }

          _sdf_wind = _sdf_wind->GetNextElement("wind");
      }
  }

  if (_sdf->HasElement("linkName")) {
    std::string link_name = _sdf->GetElement("linkName")->Get<std::string>();
    link_ = model_->GetLink(link_name);

    if (!link_)
        gzerr << "Link with name[" << link_name << "] not found. ";

    getSdfParam<V3D>(_sdf, "pitotPos", airspeed_pos_, airspeed_pos_);
    getSdfParam<V3D>(_sdf, "baroPos", barometer_pos_, barometer_pos_);

  } else {
    gzwarn << "[gazebo_mavlink_interface] Please specify a linkName, sensing at model origin.\n";
  }

  last_imu_message_.set_seq(0); // initialize this before registring/start OnUpdate which uses this

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboMavlinkInterface::OnUpdate, this, _1));

  // Subscriber to IMU sensor_msgs::Imu Message and SITL message
  imu_sub_ = node_handle_->Subscribe("~/" + namespace_ + imu_sub_topic_, &GazeboMavlinkInterface::ImuCallback, this);
  //gzmsg<<"subscribing to " + imu_sub_->GetTopic()<<std::endl;
  std::cout<<"\33[1m[gazebo_mavlink_interface]\33[0m Subscribing to \33[1;34m"<<imu_sub_->GetTopic()<<"\33[0m gazebo message\n";

  lidar_sub_ = node_handle_->Subscribe(namespace_  + lidar_sub_topic_, &GazeboMavlinkInterface::LidarCallback, this);
  //gzmsg<<"subscribing to " + lidar_sub_->GetTopic()<<std::endl;
  std::cout<<"\33[1m[gazebo_mavlink_interface]\33[0m Subscribing to \33[1;34m" << lidar_sub_->GetTopic() << "\33[0m gazebo message\n";

  opticalFlow_sub_ = node_handle_->Subscribe("~/" + namespace_  + opticalFlow_sub_topic_, &GazeboMavlinkInterface::OpticalFlowCallback, this);
  //gzdbg<<"subscribing to ~/" + namespace_ + opticalFlow_sub_topic_<<std::endl;
  std::cout<<"\33[1m[gazebo_mavlink_interface]\33[0m Subscribing to \33[1;34m" << opticalFlow_sub_->GetTopic() << "\33[0m gazebo message\n";

  gps_sub_ = node_handle_->Subscribe("~/" + namespace_  + gps_sub_topic_ + "_hil", &GazeboMavlinkInterface::GpsCallback, this);
  //gzdbg<<"subscribing to ~/" + namespace_ + gps_sub_topic_ + "_hil"<<std::endl;
  std::cout<<"\33[1m[gazebo_mavlink_interface]\33[0m Subscribing to \33[1;34m" << gps_sub_->GetTopic() << "\33[0m gazebo message\n";

  gps_gt_sub_ = node_handle_->Subscribe("~/" + namespace_  + gps_gt_sub_topic_ + "_hil", &GazeboMavlinkInterface::GpsGtCallback, this);
  //gzdbg<<"subscribing to ~/" + namespace_ + gps_gt_sub_topic_ + "_hil"<<std::endl;
  std::cout<<"\33[1m[gazebo_mavlink_interface]\33[0m Subscribing to \33[1;34m" << gps_gt_sub_->GetTopic() << "\33[0m gazebo message\n";

  if (use_vane) {
    vane_sub_ = node_handle_->Subscribe("~/" + namespace_  + vane_sub_topic_, &GazeboMavlinkInterface::VaneCallback, this);
    //gzdbg<<"subscribing to ~/" + namespace_ + vane_sub_topic_<<std::endl;
    std::cout<<"\33[1m[gazebo_mavlink_interface]\33[0m Subscribing to \33[1;34m" << vane_sub_->GetTopic() << "\33[0m gazebo message\n";
  }

  for (int j=0; j<n_wind; j++) {
      wind[j].wind_sub_ = node_handle_->Subscribe("~/" + namespace_ + "/" + wind[j].wind_topic, &GazeboMavlinkInterface::Wind::Callback, &wind[j]);
      //gzdbg<<"subscribing to ~/" + namespace_ + "/" + wind[j].wind_topic + "\n";
      std::cout<<"\33[1m[gazebo_mavlink_interface]\33[0m Subscribing to \33[1;34m" << wind[j].wind_sub_->GetTopic() << "\33[0m gazebo message\n";
  }

  // Publish gazebo's motor_speed message
  motor_velocity_reference_pub_ = node_handle_->Advertise<gz_mav_msgs::CommandMotorSpeed>("~/" + namespace_ + motor_velocity_reference_pub_topic_, 1);
  //gzdbg<<"advertising ~/" + namespace_ + motor_velocity_reference_pub_topic_<<std::endl;
  std::cout<<"\33[1m[gazebo_mavlink_interface]\33[0m Advertising \33[1;32m" << motor_velocity_reference_pub_->GetTopic() << "\33[0m gazebo message\n";

  // Publish gazebo's actuators message
  actuators_reference_pub_ = node_handle_->Advertise<gz_sensor_msgs::Actuators>("~/" + namespace_ + actuators_reference_pub_topic_, 1);
  //gzdbg<<"advertising ~/" + namespace_ + actuators_reference_pub_topic_<<std::endl;
  std::cout<<"\33[1m[gazebo_mavlink_interface]\33[0m Advertising \33[1;32m" << actuators_reference_pub_->GetTopic() << "\33[0m gazebo message\n";

  //Publish robot world position message to be used by tracking camera
  tracking_pos_pub_ = node_handle_->Advertise<gazebo::msgs::Vector3d>(namespace_ + tracking_pos_pub_topic_, 1);
  //gzdbg<<"advertising ~/" + namespace_ + tracking_pos_pub_topic_<<std::endl;
  std::cout<<"\33[1m[gazebo_mavlink_interface]\33[0m Advertising \33[1;32m" << tracking_pos_pub_->GetTopic() << "\33[0m gazebo message\n";

  _rotor_count = 5;
#if GAZEBO_MAJOR_VERSION >= 9
  last_time_ = world_->SimTime();
  last_imu_time_ = world_->SimTime();
  last_wall_time_ = world_->RealTime();
  gravity_W_ = world_->Gravity();
#else
  last_time_ = world_->GetSimTime();
  last_imu_time_ = world_->GetSimTime();
  gravity_W_ = ignitionFromGazeboMath(world_->GetPhysicsEngine()->GetGravity());
#endif

  if (_sdf->HasElement("imu_rate")) {
    imu_update_interval_ = 1.0 / _sdf->GetElement("imu_rate")->Get<int>();
  }

  // Magnetic field data for Zurich from WMM2015 (10^5xnanoTesla (N E D) n-frame )
  // mag_n_ = {0.21523, 0.00771, -0.42741};
  // We set the world Y component to zero because we apply
  // the declination based on the global position,
  // and so we need to start without any offsets.
  // The real value for Zurich would be 0.00771
  // frame d is the magnetic north frame
  mag_d_.X() = 0.21523;
  mag_d_.Y() = 0;
  mag_d_.Z() = -0.42741;

  if(_sdf->HasElement("hil_mode"))
  {
    hil_mode_ = _sdf->GetElement("hil_mode")->Get<bool>();
  }

  if(_sdf->HasElement("hil_state_level"))
  {
    hil_state_level_ = _sdf->GetElement("hil_state_level")->Get<bool>();
  }

  // Get serial params
  if(_sdf->HasElement("serialEnabled"))
  {
    serial_enabled_ = _sdf->GetElement("serialEnabled")->Get<bool>();
  }

  if(serial_enabled_) {
    // Set up serial interface
    if(_sdf->HasElement("serialDevice"))
    {
      device_ = _sdf->GetElement("serialDevice")->Get<std::string>();
    }

    if (_sdf->HasElement("baudRate")) {
      baudrate_ = _sdf->GetElement("baudRate")->Get<int>();
    }
    io_service.post(std::bind(&GazeboMavlinkInterface::do_read, this));

    // run io_service for async io
    io_thread = std::thread([this] () {
    io_service.run();
  });
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
#if GAZEBO_MAJOR_VERSION >= 9
  auto worldName = world_->Name();
#else
  auto worldName = world_->GetName();
#endif
  model_param(worldName, model_->GetName(), "mavlink_udp_port", mavlink_udp_port_);

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

  memset((char *)&_myaddr, 0, sizeof(_myaddr));
  _myaddr.sin_family = AF_INET;
  _srcaddr.sin_family = AF_INET;

  if (serial_enabled_) {
    // gcs link
    _myaddr.sin_addr.s_addr = mavlink_addr_;
    _myaddr.sin_port = htons(mavlink_udp_port_);
    _srcaddr.sin_addr.s_addr = qgc_addr_;
    _srcaddr.sin_port = htons(qgc_udp_port_);
  }

  else {
    _myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    // Let the OS pick the port
    _myaddr.sin_port = htons(0);
    _srcaddr.sin_addr.s_addr = mavlink_addr_;
    _srcaddr.sin_port = htons(mavlink_udp_port_);
  }

  _addrlen = sizeof(_srcaddr);

  if (bind(_fd, (struct sockaddr *)&_myaddr, sizeof(_myaddr)) < 0) {
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
  }
  else if (protocol_version_ == 1.0) {
    chan_state->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    gzmsg << "Using MAVLink protocol v1.0\n";
  }
  else {
    gzerr << "Unkown protocol version! Using v" << protocol_version_ << "by default \n";
  }
}

// This gets called by the world update start event.
void GazeboMavlinkInterface::OnUpdate(const common::UpdateInfo&  /*_info*/) {

    std::unique_lock<std::mutex> lock(last_imu_message_mutex_); // blocks in imu callback
    //gzdbg<<"previous_imu_seq_: "<<previous_imu_seq_<<std::endl;
    if (previous_imu_seq_ > 0) {
        while (previous_imu_seq_ == last_imu_message_.seq() && IsRunning()) {
            // free lock if we want new imu data -> imu callback un-blocked for 10ms or
            // until new message arrived. During this time go to sleep
            last_imu_message_cond_.wait_for(lock, std::chrono::milliseconds(10));
            // does lock.unlock() upon call and lock.lock() upon notification or if timer expires
            // (might stay blocked if mutex is held by other lock at this time...
            // loop again if no new message arrived in this interval, otherwise proceed (locked)
        }
    }

    previous_imu_seq_ = last_imu_message_.seq();

#if GAZEBO_MAJOR_VERSION >= 9
   common::Time current_time = world_->SimTime();
   common::Time current_wall_time = world_->RealTime();
#else
   common::Time current_time = world_->GetSimTime();
#endif
  double dt = (current_time - last_time_).Double();
  dt_wall_ += (current_wall_time - last_wall_time_).Double();
  last_time_ = current_time;
  last_wall_time_ = current_wall_time;

  SendSensorMessages();

  pollForMAVLinkMessages(dt, 1000);

  handle_control(dt);

  if (received_first_referenc_) {
    gz_mav_msgs::CommandMotorSpeed turning_velocities_msg;
    gz_sensor_msgs::Actuators actuators_msg;

    for (int i = 0; i < 16; i++) {
      if (last_actuator_time_ == 0 || (current_time - last_actuator_time_).Double() > 0.2) {
        turning_velocities_msg.add_motor_speed(0);
        actuators_msg.add_normalized(0);
      } else {
        turning_velocities_msg.add_motor_speed(channels[i].input_reference_);
        actuators_msg.add_normalized(channels[i].input_reference_);
      }
    }

    actuators_msg.mutable_header()->mutable_stamp()->set_sec(current_time.sec);
    actuators_msg.mutable_header()->mutable_stamp()->set_nsec(current_time.nsec);
    actuators_msg.mutable_header()->set_frame_id("blabla");

    motor_velocity_reference_pub_->Publish(turning_velocities_msg);
    actuators_reference_pub_->Publish(actuators_msg);
  }

  gazebo::msgs::Vector3d tracking_pos;
  tracking_pos.set_x(model_->WorldPose().Pos().X());
  tracking_pos.set_y(model_->WorldPose().Pos().Y());
  tracking_pos.set_z(model_->WorldPose().Pos().Z());
  tracking_pos_pub_->Publish(tracking_pos);

  // Display timing statistics

  if(dbg_counter_%100==0){
      gzdbg<<"Itv: <.005  |.005-.01| .01-.02| .02-.04| .04-.1 | .1-.2  | .2-.3  | .3-.4  | .4-.5  | .5-1.0 |  >1.0  |   tot  | f=: "<<100/dt_wall_<<"\n";

      gzdbg<<"GPS:"
           <<std::setw(8)<<std::right<<timing_stats_gps_[0]<<"|"
           <<std::setw(8)<<std::right<<timing_stats_gps_[1]<<"|"
           <<std::setw(8)<<std::right<<timing_stats_gps_[2]<<"|"
           <<std::setw(8)<<std::right<<timing_stats_gps_[3]<<"|"
           <<std::setw(8)<<std::right<<timing_stats_gps_[4]<<"|"
           <<std::setw(8)<<std::right<<timing_stats_gps_[5]<<"|"
           <<std::setw(8)<<std::right<<timing_stats_gps_[6]<<"|"
           <<std::setw(8)<<std::right<<timing_stats_gps_[7]<<"|"
           <<std::setw(8)<<std::right<<timing_stats_gps_[8]<<"|"
           <<std::setw(8)<<std::right<<timing_stats_gps_[9]<<"|"
           <<std::setw(8)<<std::right<<timing_stats_gps_[10]<<"|"
           <<std::setw(8)<<std::right<<send_counter_gps_<<"\n";

      gzdbg<<"IMU:"
           <<std::setw(8)<<std::right<<timing_stats_imu_[0]<<"|"
           <<std::setw(8)<<std::right<<timing_stats_imu_[1]<<"|"
           <<std::setw(8)<<std::right<<timing_stats_imu_[2]<<"|"
           <<std::setw(8)<<std::right<<timing_stats_imu_[3]<<"|"
           <<std::setw(8)<<std::right<<timing_stats_imu_[4]<<"|"
           <<std::setw(8)<<std::right<<timing_stats_imu_[5]<<"|"
           <<std::setw(8)<<std::right<<timing_stats_imu_[6]<<"|"
           <<std::setw(8)<<std::right<<timing_stats_imu_[7]<<"|"
           <<std::setw(8)<<std::right<<timing_stats_imu_[8]<<"|"
           <<std::setw(8)<<std::right<<timing_stats_imu_[9]<<"|"
           <<std::setw(8)<<std::right<<timing_stats_imu_[10]<<"|"
           <<std::setw(8)<<std::right<<send_counter_imu_<<"\n\n";
      dt_wall_ = 0;
  }

  ++dbg_counter_;
}

void GazeboMavlinkInterface::send_mavlink_message(const mavlink_message_t *message, const int destination_port)
{
    if(serial_enabled_ && destination_port == 0) {
        assert(message != nullptr);
        if (!is_open()) {
            gzerr << "Serial port closed! \n";
            return;
        }

        // lock makes sure that emplace_back is executed sequentially
        // send_mavlink_messages probably called from different threads via callback: confirmed
        {
            lock_guard lock(mutex);

            if (tx_q.size() >= MAX_TXQ_SIZE) {
               gzwarn << "TX queue overflow, size:"<< tx_q.size() <<" \n";
            }
            tx_q.emplace_back(message);
        }

        io_service.post(std::bind(&GazeboMavlinkInterface::do_write, this, true)); // returns immediately
    }

    else {
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        int packetlen = mavlink_msg_to_send_buffer(buffer, message);

        struct sockaddr_in dest_addr;
        memcpy(&dest_addr, &_srcaddr, sizeof(_srcaddr));

        if (destination_port != 0) {
            dest_addr.sin_port = htons(destination_port);
        }

        ssize_t len = sendto(_fd, buffer, packetlen, 0, (struct sockaddr *)&_srcaddr, sizeof(_srcaddr));

        if (len <= 0)
        {
            gzerr << "Failed sending mavlink message: " << strerror(errno) << "\n";
        }
    }

}

void GazeboMavlinkInterface::ImuCallback(ImuPtr& imu_message)
{
  std::unique_lock<std::mutex> lock(last_imu_message_mutex_);

  const int64_t diff = imu_message->seq() - last_imu_message_.seq();
  if (diff != 1 && imu_message->seq() != 0)
  {
    gzerr << "Skipped " << (diff - 1) << " IMU samples (presumably CPU usage is too high)\n";
  }

  last_imu_message_ = *imu_message;

  // Manual unlocking is done before notifying, to avoid waking up
  // the waiting thread only to block again (see notify_one for details)
  lock.unlock();
  last_imu_message_cond_.notify_one();
}

void GazeboMavlinkInterface::SendSensorMessages() {
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = world_->SimTime();
#else
  common::Time current_time = world_->GetSimTime();
#endif
  double dt = (current_time - last_imu_time_).Double();

  V3D wind_sens_B = V3D(0,0,0);
  if (n_wind>0 && link_) {
#if GAZEBO_MAJOR_VERSION >= 9
        ignition::math::Pose3d pose = link_->WorldPose();
#else
        ignition::math::Pose3d pose = ignitionFromGazeboMath(link_->GetWorldPose());
#endif
      V3D cp = pose.Pos() + pose.Rot().RotateVector(airspeed_pos_); // airspeed sensor position in world frame
      UpdateWind(cp);   // get wind at airspeed sensor position (updates wind_sens)
      wind_sens_B = pose.Rot().RotateVectorReverse(wind_sens);
  }


  //thread safe copying
  std::unique_lock<std::mutex> lock(gps_gt_message_mutex_);
  double gt_lat_loc = gt_lat;
  double gt_lon_loc = gt_lon;
  double gt_alt_loc = gt_alt;
  lock.unlock();

  ignition::math::Quaterniond q_br(0, 1, 0, 0);             //imu-frame to body-frame
  ignition::math::Quaterniond q_ng(0, 0.70711, 0.70711, 0); //gazebo-frame to NED-frame

    ignition::math::Quaterniond q_gr = ignition::math::Quaterniond(
      last_imu_message_.orientation().w(),
      last_imu_message_.orientation().x(),
      last_imu_message_.orientation().y(),
      last_imu_message_.orientation().z());

    ignition::math::Quaterniond q_gb = q_gr*q_br.Inverse(); //body to gazebo
    ignition::math::Quaterniond q_nb = q_ng*q_gb;           //body to ned

    //float declination = get_mag_declination(lat_rad_, lon_rad_);

    // Magnetic declination and inclination (radians)
    float declination_rad = get_mag_declination(gt_lat_loc/1e7, gt_lon_loc/1e7) * M_PI / 180;
    float inclination_rad = get_mag_inclination(gt_lat_loc/1e7, gt_lon_loc/1e7) * M_PI / 180;

    // Magnetic strength (10^5xnanoTesla)
    float strength_ga = 0.01f * get_mag_strength(gt_lat_loc/1e7, gt_lon_loc/1e7);

    // Magnetic field components are calculated by http://geomag.nrcan.gc.ca/mag_fld/comp-en.php
    float H = strength_ga * cosf(inclination_rad);
    float Z = tanf(inclination_rad) * H;
    float X = H * cosf(declination_rad);
    float Y = H * sinf(declination_rad);

    // Magnetic field data from WMM2018 (10^5xnanoTesla (N E D) n-frame )
    mag_d_.X() = X;
    mag_d_.Y() = Y;
    mag_d_.Z() = Z;

    V3D pos_g;
    V3D vel_b;
    V3D vel_n;
    V3D omega_nb_b;

    if (link_){
#if GAZEBO_MAJOR_VERSION >= 9
        ignition::math::Pose3d pose = link_->WorldPose();
        pos_g = pose.Pos() + pose.Rot().RotateVector(barometer_pos_);
        vel_b = q_br.RotateVector(pose.Rot().RotateVectorReverse(link_->WorldLinearVel(airspeed_pos_)));
        vel_n = q_ng.RotateVector(link_->WorldLinearVel());
        omega_nb_b = q_br.RotateVector(link_->RelativeAngularVel());
#else
        ignition::math::Pose3d pose = ignitionFromGazeboMath(link_->GetWorldPose());
        pos_g = pose.Pos() + pose.Rot().RotateVector(barometer_pos_);
        vel_b = q_br.RotateVector(pose.Rot().RotateVectorReverse(link_->GetWorldLinearVel(airspeed_pos_)));
        vel_n = q_ng.RotateVector(ignitionFromGazeboMath(link_->GetWorldLinearVel()));
        omega_nb_b = q_br.RotateVector(ignitionFromGazeboMath(link_->GetRelativeAngularVel()));
#endif

    } else {
#if GAZEBO_MAJOR_VERSION >= 9
        pos_g = model_->WorldPose().Pos();
        vel_b = q_br.RotateVector(model_->RelativeLinearVel());
        vel_n = q_ng.RotateVector(model_->WorldLinearVel());
        omega_nb_b = q_br.RotateVector(model_->RelativeAngularVel());
#else
        pos_g = ignitionFromGazeboMath(model_->GetWorldPose().pos);
        vel_b = q_br.RotateVector(ignitionFromGazeboMath(model_->GetRelativeLinearVel()));
        vel_n = q_ng.RotateVector(ignitionFromGazeboMath(model_->GetWorldLinearVel()));
        omega_nb_b = q_br.RotateVector(ignitionFromGazeboMath(model_->GetRelativeAngularVel()));
#endif
    }

    ignition::math::Vector3d pos_n = q_ng.RotateVector(pos_g);

    ignition::math::Vector3d mag_noise_b(
      0.01 * randn_(rand_),
      0.01 * randn_(rand_),
      0.01 * randn_(rand_));

    ignition::math::Vector3d accel_b = q_br.RotateVector(ignition::math::Vector3d(
      last_imu_message_.linear_acceleration().x(),
      last_imu_message_.linear_acceleration().y(),
      last_imu_message_.linear_acceleration().z()));
    ignition::math::Vector3d gyro_b = q_br.RotateVector(ignition::math::Vector3d(
      last_imu_message_.angular_velocity().x(),
      last_imu_message_.angular_velocity().y(),
      last_imu_message_.angular_velocity().z()));
    ignition::math::Vector3d mag_b = q_nb.RotateVectorReverse(mag_d_) + mag_noise_b; //ned to body

  if (imu_update_interval_!=0 && dt >= imu_update_interval_)
  {
    mavlink_hil_sensor_t sensor_msg;
#if GAZEBO_MAJOR_VERSION >= 9
    sensor_msg.time_usec = world_->SimTime().Double() * 1e6;
#else
    sensor_msg.time_usec = world_->GetSimTime().Double() * 1e6;
#endif
    sensor_msg.xacc = accel_b.X();
    sensor_msg.yacc = accel_b.Y();
    sensor_msg.zacc = accel_b.Z();
    sensor_msg.xgyro = gyro_b.X();
    sensor_msg.ygyro = gyro_b.Y();
    sensor_msg.zgyro = gyro_b.Z();
    sensor_msg.xmag = mag_b.X();
    sensor_msg.ymag = mag_b.Y();
    sensor_msg.zmag = mag_b.Z();

    // calculate abs_pressure using an ISA model for the troposphere (valid up to 11km above MSL)
    const float lapse_rate = 0.0065f; // reduction in temperature with altitude (Kelvin/m)
    const float temperature_msl = 288.0f; // temperature at MSL (Kelvin)
    float alt_msl = (float)alt_home - pos_n.Z();
    float temperature_local = temperature_msl - lapse_rate * alt_msl;
    float pressure_ratio = powf((temperature_msl/temperature_local) , 5.256f);
    const float pressure_msl = 101325.0f; // pressure at MSL
    sensor_msg.abs_pressure = pressure_msl / pressure_ratio;

    // generate Gaussian noise sequence using polar form of Box-Muller transformation
    // http://www.design.caltech.edu/erik/Misc/Gaussian.html
    double x1, x2, w, y1, y2;
    do {
     x1 = 2.0 * (rand() * (1.0 / (double)RAND_MAX)) - 1.0;
     x2 = 2.0 * (rand() * (1.0 / (double)RAND_MAX)) - 1.0;
     w = x1 * x1 + x2 * x2;
    } while ( w >= 1.0 );
    w = sqrt( (-2.0 * log( w ) ) / w );
    y1 = x1 * w;
    y2 = x2 * w;

    // Apply 1 Pa RMS noise
    float abs_pressure_noise = 1.0f * (float)w;
    sensor_msg.abs_pressure += abs_pressure_noise;

    // convert to hPa
    sensor_msg.abs_pressure *= 0.01f;

    // calculate density using an ISA model for the tropsphere (valid up to 11km above MSL)
    const float density_ratio = powf((temperature_msl/temperature_local) , 4.256f);
    float rho = 1.225f / density_ratio;

    // calculate pressure altitude including effect of pressure noise (unused on PX4)
    sensor_msg.pressure_alt = alt_msl - abs_pressure_noise / (gravity_W_.Length() * rho);

    // calculate differential pressure in hPa -> for airspeed
    if (vel_b.X()-wind_sens_B.X()>0)
        sensor_msg.diff_pressure = 0.005f*rho*(vel_b.X()-wind_sens_B.X())*(vel_b.X()-wind_sens_B.X());
    else
        sensor_msg.diff_pressure = 0;

    // calculate temperature in Celsius
    sensor_msg.temperature = temperature_local - 273.0f;

    sensor_msg.fields_updated = 4095;

    //accumulate gyro measurements that are needed for the optical flow message
    static uint32_t last_dt_us = sensor_msg.time_usec;
    uint32_t dt_us = sensor_msg.time_usec - last_dt_us;
    if (dt_us > 1000) {
      optflow_gyro += gyro_b * (dt_us / 1000000.0f);
      last_dt_us = sensor_msg.time_usec;
    }

    mavlink_message_t msg;
    mavlink_msg_hil_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
    if (hil_mode_) {
      if (!hil_state_level_){

          // Get timing statistics
          {
              common::Time now = world_->RealTime();
              double_t dt_wall = (now-last_wall_time_imu_).Double();
              last_wall_time_imu_ = now;
              ++send_counter_imu_;

              if (dt_wall<0.005f) {
                  timing_stats_imu_[0]++;

              } else if (dt_wall>=0.005f&&dt_wall<0.01f){
                  timing_stats_imu_[1]++;

              } else if (dt_wall>=0.01f&&dt_wall<0.02f){
                  timing_stats_imu_[2]++;

              } else if (dt_wall>=0.02f&&dt_wall<0.04f){
                  timing_stats_imu_[3]++;

              } else if (dt_wall>=0.04f&&dt_wall<0.10f){
                  timing_stats_imu_[4]++;

              } else if (dt_wall>=0.10f&&dt_wall<0.20f){
                  timing_stats_imu_[5]++;

              } else if (dt_wall>=0.20f&&dt_wall<0.30f){
                  timing_stats_imu_[6]++;

              } else if (dt_wall>=0.30f&&dt_wall<0.40f){
                  timing_stats_imu_[7]++;

              } else if (dt_wall>=0.40f&&dt_wall<0.50f){
                  timing_stats_imu_[8]++;

              } else if (dt_wall>=0.50f&&dt_wall<1.00f){
                  timing_stats_imu_[9]++;

              } else if (dt_wall>=1.0f){
                  timing_stats_imu_[10]++;
              }
          }

        send_mavlink_message(&msg);
      }
    } else {
      send_mavlink_message(&msg);
    }
    last_imu_time_ = current_time;
  }

    // ground truth
#if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Vector3d accel_true_b = q_br.RotateVector(model_->RelativeLinearAccel());
#else
    ignition::math::Vector3d accel_true_b = q_br.RotateVector(ignitionFromGazeboMath(model_->GetRelativeLinearAccel()));
#endif

    // send ground truth
    mavlink_hil_state_quaternion_t hil_state_quat;
#if GAZEBO_MAJOR_VERSION >= 9
    hil_state_quat.time_usec = world_->SimTime().Double() * 1e6;
#else
    hil_state_quat.time_usec = world_->GetSimTime().Double() * 1e6;
#endif
    hil_state_quat.attitude_quaternion[0] = q_nb.W();
    hil_state_quat.attitude_quaternion[1] = q_nb.X();
    hil_state_quat.attitude_quaternion[2] = q_nb.Y();
    hil_state_quat.attitude_quaternion[3] = q_nb.Z();

    hil_state_quat.rollspeed = omega_nb_b.X();
    hil_state_quat.pitchspeed = omega_nb_b.Y();
    hil_state_quat.yawspeed = omega_nb_b.Z();

    hil_state_quat.lat = gt_lat_loc;
    hil_state_quat.lon = gt_lon_loc;
    hil_state_quat.alt = gt_alt_loc;

    hil_state_quat.vx = vel_n.X() * 100;
    hil_state_quat.vy = vel_n.Y() * 100;
    hil_state_quat.vz = vel_n.Z() * 100;

    // assumed indicated airspeed due to flow aligned with pitot (body x)
    if (vel_b.X()-wind_sens_B.X()>0)
        hil_state_quat.ind_airspeed = vel_b.X()-wind_sens_B.X();
    else
        hil_state_quat.ind_airspeed = 0;

    V3D tgs = V3D(0,0,0);
    if (link_) {
#if GAZEBO_MAJOR_VERSION >= 9
        tgs = link_->WorldLinearVel(airspeed_pos_);
#else
        tgs = link_->GetWorldLinearVel(airspeed_pos_);
#endif
    } else {
#if GAZEBO_MAJOR_VERSION >= 9
        tgs = model_->WorldLinearVel();
#else
        tgs = model_->GetWorldLinearVel();
#endif
    }

    hil_state_quat.true_airspeed = (tgs - wind_sens_B).Length() * 100;

    hil_state_quat.xacc = accel_true_b.X() * 1000;
    hil_state_quat.yacc = accel_true_b.Y() * 1000;
    hil_state_quat.zacc = accel_true_b.Z() * 1000;

    mavlink_message_t msg;
    mavlink_msg_hil_state_quaternion_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &hil_state_quat);
    if (hil_mode_) {
      if (hil_state_level_){
        send_mavlink_message(&msg);
      }
    } else {
      send_mavlink_message(&msg);
    }

    if (dbg_counter_%100==0&&false) {
        gzdbg<<"lat: "  <<gt_lat_loc <<" | lon: "<<gt_lon_loc   <<" | alt: "<<gt_alt_loc<<"\n";
        gzdbg<<"pos_x: "<<pos_g.X()  <<" | pos_y: "<<pos_g.Y()  <<" | pos_z: "<<pos_g.Z()<<"\n";
        gzdbg<<"b_acc_x: "<<accel_b.X()<<" | b_acc_y: "<<accel_b.Y()<<" | b_acc_z: "<<accel_b.Z()<<"\n";
        gzdbg<<"acc_x: "<<last_imu_message_.linear_acceleration().x()<<" | acc_y: "<<last_imu_message_.linear_acceleration().y()<<" | acc_z: "<<last_imu_message_.linear_acceleration().z()<<"\n";
        gzdbg<<"gyr_x: "<<gyro_b.X() <<" | gyr_y: "<<gyro_b.Y() <<" | gyr_z: "<<gyro_b.Z()<<"\n";
        gzdbg<<"mag_x: "<<mag_b.X() <<" | mag_y: "<<mag_b.Y() <<" | mag_z: "<<mag_b.Z()<<"\n";
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
  sensor_msg.orientation = 25;//downward facing
  sensor_msg.covariance = 0;

  //distance needed for optical flow message
  optflow_distance = lidar_message->current_distance();  //[m]

  mavlink_message_t msg;
  mavlink_msg_distance_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
  send_mavlink_message(&msg);
}

void GazeboMavlinkInterface::OpticalFlowCallback(OpticalFlowPtr& opticalFlow_message) {

  mavlink_hil_optical_flow_t sensor_msg;
#if GAZEBO_MAJOR_VERSION >= 9
  sensor_msg.time_usec = world_->SimTime().Double() * 1e6;
#else
  sensor_msg.time_usec = world_->GetSimTime().Double() * 1e6;
#endif
  sensor_msg.sensor_id = opticalFlow_message->sensor_id();
  sensor_msg.integration_time_us = opticalFlow_message->integration_time_us();
  sensor_msg.integrated_x = opticalFlow_message->integrated_x();
  sensor_msg.integrated_y = opticalFlow_message->integrated_y();
  sensor_msg.integrated_xgyro = opticalFlow_message->quality() ? -optflow_gyro.Y() : 0.0f;//xy switched
  sensor_msg.integrated_ygyro = opticalFlow_message->quality() ? optflow_gyro.X() : 0.0f;  //xy switched
  sensor_msg.integrated_zgyro = opticalFlow_message->quality() ? -optflow_gyro.Z() : 0.0f;//change direction
  sensor_msg.temperature = opticalFlow_message->temperature();
  sensor_msg.quality = opticalFlow_message->quality();
  sensor_msg.time_delta_distance_us = opticalFlow_message->time_delta_distance_us();
  sensor_msg.distance = optflow_distance;

  //reset gyro integral
  optflow_gyro.Set();

  mavlink_message_t msg;
  mavlink_msg_hil_optical_flow_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
  send_mavlink_message(&msg);
}

void GazeboMavlinkInterface::VaneCallback(VanePtr& vane_message) {

  mavlink_hil_extended_t hil_extended_msg;

#if GAZEBO_MAJOR_VERSION >= 9
  hil_extended_msg.timestamp = world_->SimTime().Double() * 1e6;
#else
  hil_extended_msg.timestamp = world_->GetSimTime().Double() * 1e6;
#endif

  hil_extended_msg.var1 = vane_message->x();    // angle of attack [rad]
  hil_extended_msg.var2 = vane_message->y();    // sideslip angle [rad]
  hil_extended_msg.var3 = 0;
  hil_extended_msg.var4 = 0;
  hil_extended_msg.var5 = 0;
  hil_extended_msg.var6 = 0;
  hil_extended_msg.var7 = 0;
  hil_extended_msg.var8 = 0;

  mavlink_message_t msg;
  mavlink_msg_hil_extended_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &hil_extended_msg);
  send_mavlink_message(&msg);

  /*
  gzdbg<<"alpha: "<<vane_message->x()<<"\n";
  gzdbg<<"beta: "<<vane_message->y()<<"\n";
  */
}

void GazeboMavlinkInterface::GpsCallback(GpsPtr& gps_msg) {
  // fill HIL GPS Mavlink msg
  mavlink_hil_gps_t hil_gps_msg;
  hil_gps_msg.time_usec = gps_msg->time_usec();
  hil_gps_msg.fix_type = 3;
  hil_gps_msg.lat = gps_msg->latitude_deg() * 1e7;
  hil_gps_msg.lon = gps_msg->longitude_deg() * 1e7;
  hil_gps_msg.alt = gps_msg->altitude() * 1000.0;
  hil_gps_msg.eph = gps_msg->eph() * 100.0;
  hil_gps_msg.epv = gps_msg->epv() * 100.0;
  hil_gps_msg.vel = gps_msg->velocity() * 100.0;
  hil_gps_msg.vn = gps_msg->velocity_north() * 100.0;
  hil_gps_msg.ve = gps_msg->velocity_east() * 100.0;
  hil_gps_msg.vd = -gps_msg->velocity_up() * 100.0;
  // MAVLINK_HIL_GPS_T CoG is [0, 360]. math::Angle::Normalize() is [-pi, pi].
  //ignition::math::Angle cog(atan2(gps_msg->velocity_east(), gps_msg->velocity_north()));
  //cog.Normalize();
  double cog = atan2(gps_msg->velocity_east(), gps_msg->velocity_north());
  hil_gps_msg.cog = static_cast<uint16_t>(180*(cog+3.14159)/3.14159);
  hil_gps_msg.satellites_visible = 10;

  //gzdbg<<"dlat: "<<lat_last-hil_gps_msg.lat<<" dlon: "<<lon_last-hil_gps_msg.lon<<std::endl;
  lon_last = hil_gps_msg.lon;
  lat_last = hil_gps_msg.lat;

  // send HIL_GPS Mavlink msg

  if (hil_mode_ || (hil_mode_ && !hil_state_level_)) {

      // Get timing statistics
      {
          common::Time now = world_->RealTime();
          double_t dt_wall = (now-last_wall_time_gps_).Double();
          last_wall_time_gps_ = now;
          ++send_counter_gps_;

          if (dt_wall<0.005f) {
              timing_stats_gps_[0]++;

          } else if (dt_wall>=0.005f&&dt_wall<0.01f){
              timing_stats_gps_[1]++;

          } else if (dt_wall>=0.01f&&dt_wall<0.02f){
              timing_stats_gps_[2]++;

          } else if (dt_wall>=0.02f&&dt_wall<0.04f){
              timing_stats_gps_[3]++;

          } else if (dt_wall>=0.04f&&dt_wall<0.10f){
              timing_stats_gps_[4]++;

          } else if (dt_wall>=0.10f&&dt_wall<0.20f){
              timing_stats_gps_[5]++;

          } else if (dt_wall>=0.20f&&dt_wall<0.30f){
              timing_stats_gps_[6]++;

          } else if (dt_wall>=0.30f&&dt_wall<0.40f){
              timing_stats_gps_[7]++;

          } else if (dt_wall>=0.40f&&dt_wall<0.50f){
              timing_stats_gps_[8]++;

          } else if (dt_wall>=0.50f&&dt_wall<1.00f){
              timing_stats_gps_[9]++;

          } else if (dt_wall>=1.0f){
              timing_stats_gps_[10]++;
          }
      }

    mavlink_message_t msg;
    mavlink_msg_hil_gps_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &hil_gps_msg);
    send_mavlink_message(&msg);
  }
}

void GazeboMavlinkInterface::GpsGtCallback(GpsPtr& gps_gt_msg){

    std::unique_lock<std::mutex> lock(gps_gt_message_mutex_);
    gt_lat = gps_gt_msg->latitude_deg() * 1e7;
    gt_lon = gps_gt_msg->longitude_deg() * 1e7;
    gt_alt = gps_gt_msg->altitude() * 1000.0;
    //lock.unlock();
}

void GazeboMavlinkInterface::pollForMAVLinkMessages(double _dt, uint32_t _timeoutMs)
{
  // convert timeout in ms to timeval
  struct timeval tv;
  tv.tv_sec = _timeoutMs / 1000;
  tv.tv_usec = (_timeoutMs % 1000) * 1000UL;

  // poll
  ::poll(&fds[0], (sizeof(fds[0]) / sizeof(fds[0])), 0);

  if (fds[0].revents & POLLIN) {
    int len = recvfrom(_fd, _buf, sizeof(_buf), 0, (struct sockaddr *)&_srcaddr, &_addrlen);
    if (len > 0) {
      mavlink_message_t msg;
      mavlink_status_t status;
      for (unsigned i = 0; i < len; ++i)
      {
        if (mavlink_parse_char(MAVLINK_COMM_0, _buf[i], &msg, &status))
        {
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

void GazeboMavlinkInterface::handle_message(mavlink_message_t *msg)
{
  switch (msg->msgid) {
  case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
    //gzdbg<<"got a hil_actuator_controls message"<<std::endl;
    mavlink_hil_actuator_controls_t controls;
    mavlink_msg_hil_actuator_controls_decode(msg, &controls);
    bool armed = false;

    if ((controls.mode & MAV_MODE_FLAG_SAFETY_ARMED) > 0) {
      armed = true;
    }
#if GAZEBO_MAJOR_VERSION >= 9
    last_actuator_time_ = world_->SimTime();
#else
    last_actuator_time_ = world_->GetSimTime();
#endif
    /*
    for (unsigned i = 0; i < n_out_max; i++) {
      input_index_[i] = i;
    }
    */

    // Set rotor speeds and controller targets for flagged messages.
    if (controls.flags == kMotorSpeedFlag) {
      //input_reference_.resize(n_out_max);
      for (unsigned i = 0; i < n_motors; ++i) {
        if (i>=n_chan) {
            gzerr<<"index out of bounds, break \n";
            break;
        }

        if (armed) {
          channels[i].control = controls.controls[channels[i].input_index_];
          channels[i].input_reference_ =
              (controls.controls[channels[i].input_index_] + channels[i].input_offset_) *
                  channels[i].input_scaling_ +
              channels[i].zero_position_armed_;
        } else {
          channels[i].input_reference_ = channels[i].zero_position_disarmed_;
          channels[i].control = 0.0;
        }
      }
      received_first_referenc_ = true;
    }
    else if (controls.flags == kServoPositionFlag) {
      for (unsigned i = n_motors; i < (n_motors + n_servos); ++i) {
        if (i>=n_chan) {
              gzerr<<"index out of bounds, break \n";
              break;
        }

        if (armed) {
          channels[i].control = controls.controls[channels[i].input_index_];
          channels[i].input_reference_ =
              (controls.controls[channels[i].input_index_] + channels[i].input_offset_) *
                  channels[i].input_scaling_ +
              channels[i].zero_position_armed_;
        } else {
          channels[i].input_reference_ = channels[i].zero_position_disarmed_;
          channels[i].control = 0.0;
        }
      }
    }
    // Set rotor speeds, controller targets for unflagged messages.
    else {
      //input_reference_.resize(n_out_max);

      for (unsigned i = 0; i < n_chan; ++i) {
        if (armed) {
          channels[i].control = controls.controls[channels[i].input_index_];
          channels[i].input_reference_ =
              (controls.controls[channels[i].input_index_] + channels[i].input_offset_) *
                  channels[i].input_scaling_ +
              channels[i].zero_position_armed_;

        } else {
          channels[i].input_reference_ = channels[i].zero_position_disarmed_;
          channels[i].control = 0.0;
        }
      }

      /*
      gzdbg<<"a0: "<<input_reference_[0]<<" | a1: "<<input_reference_[1]<<" | a2: "<<input_reference_[2]
           <<" | a3: "<<input_reference_[3]<<" | a4: "<<input_reference_[4]<<" | a5: "<<input_reference_[5]
           <<" | a6: "<<input_reference_[6]<<" | a7: "<<input_reference_[7]<<std::endl;
*/
      received_first_referenc_ = true;
    }
    break;
  }

}

void GazeboMavlinkInterface::handle_control(double _dt)
{
  //gzdbg<<"dbg: 2a"<<std::endl;
  // set joint positions
  for (int i = 0; i < n_chan; i++) {
    if (channels[i].joint_||channels[i].joint_control_type_ == "gz_msg") {
      //gzdbg<<"dbg: 2b"<<i<<std::endl;
      double target = channels[i].input_reference_;

      if (channels[i].joint_control_type_ == "velocity") {
        double current = channels[i].joint_->GetVelocity(0);
        double err =  current - target;
        double force = channels[i].pid_.Update(err, _dt);

        force = ignition::math::clamp(-3*err,-1.0,1.0);

        if (!isnan(force)) {
            channels[i].joint_->SetVelocity(0,target);
            //gzdbg<<"prop speed: "<<current<<" prop moment: "<<force<<"err: "<<err<<"\n";
        } else {
            gzdbg<<"nan force\n";
        }

        //publish velocity
        gz_std_msgs::Float32 m;
        m.set_data((float)target);
        channels[i].joint_control_pub_->Publish(m);

      } else if (channels[i].joint_control_type_ == "position") {
#if GAZEBO_MAJOR_VERSION >= 9
        double current = channels[i].joint_->Position(0);
#else
        double current = channels[i].joint_->GetAngle(0).Radian();
#endif

        double err = current - target;
        double force = channels[i].pid_.Update(err, _dt);
        channels[i].joint_->SetForce(0, force);

        //publish velocity
        gz_std_msgs::Float32 m;
        m.set_data((float)target);
        channels[i].joint_control_pub_->Publish(m);

      } else if (channels[i].joint_control_type_ == "servo") {

        common::Time now;
        double pos;
        double rate;
        double rate_ref;

#if GAZEBO_MAJOR_VERSION >= 9
        now = world_->SimTime();
        pos = channels[i].joint_->Position(0);
        rate = channels[i].joint_->GetVelocity(0);
#else
        now = world_->GetSimTime();
        pos = channels[i].joint_->GetAngle(0).Radian();
        rate = channels[i].joint_->GetVelocity(0);
#endif

        if (!channels[i].srv.init) {
            channels[i].srv.last_srv_time = now;
            channels[i].srv.init = true;
        }

        double dt = (now - channels[i].srv.last_srv_time).Double();
        double d_ref = channels[i].srv.slew * dt;
        channels[i].srv.last_srv_time = now;

        // implementation of slew-rate constraint
        if (channels[i].input_reference_>channels[i].srv.ref+d_ref) {
            rate_ref = channels[i].srv.slew;
            channels[i].srv.ref+=d_ref;
        } else if (channels[i].input_reference_<channels[i].srv.ref-d_ref) {
            rate_ref = -channels[i].srv.slew;
            channels[i].srv.ref -= d_ref;
        } else {
            rate_ref = (channels[i].input_reference_-channels[i].srv.ref)/dt;
            channels[i].srv.ref = channels[i].input_reference_;
        }

        // sanity
        if (isnan(channels[i].srv.ref))
            channels[i].srv.ref = 0.0;

        if (isnan(rate_ref))
            rate_ref = 0.0;

        channels[i].srv.ref = ignition::math::clamp(channels[i].srv.ref,
                                                    -abs(channels[i].input_scaling_),
                                                    +abs(channels[i].input_scaling_));

        double err_pos = channels[i].srv.ref-pos;
        double err_vel = rate_ref-rate;
        double torque = channels[i].srv.P_pos*err_pos + channels[i].srv.P_vel*err_vel;
        channels[i].joint_->SetForce(0, torque);

        //if(dt>0 && std::abs(err_pos)>0.02)
        //  channels[i].joint_->SetVelocity(0,rate_ref+(err_pos/dt/2.0));

        //publish servo position (without link dynamics)
        gz_std_msgs::Float32 m;
        m.set_data((float)channels[i].srv.ref);
        channels[i].joint_control_pub_->Publish(m);

      } else if (channels[i].joint_control_type_ == "gz_msg") {
          gz_std_msgs::Float32 m;
          m.set_data((float)target);
          //gzdbg<<"target: "<<target<<"\n";
     /*
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
     */
        channels[i].joint_control_pub_->Publish(m);

      } else if (channels[i].joint_control_type_ == "position_kinematic") {
        /// really not ideal if your drone is moving at all,
        /// mixing kinematic updates with dynamics calculation is
        /// non-physical.
     #if GAZEBO_MAJOR_VERSION >= 6
        channels[i].joint_->SetPosition(0, channels[i].input_reference_);
     #else
        channels[i].joint_->SetAngle(0, channels[i].input_reference_);
     #endif
        //publish position
        gz_std_msgs::Float32 m;
        m.set_data((float)target);
        channels[i].joint_control_pub_->Publish(m);
      } else {
        gzerr << "joint_control_type[" << channels[i].joint_control_type_ << "] undefined.\n";
      }
    }
  }

  for(int i=0; i<n_chan; i++){
       if(isnan(channels[i].input_reference_))
            gzerr<<"ch_"<<i<<" input_reference_: NaN \n";
  }

  if (dbg_counter_%100==0&&true) {
      for(int i=0; i<n_chan; i++){
          gzdbg<<"ch_"<<i<<"("<<channels[i].input_index_<<") Ref: "<<channels[i].input_reference_<<" Raw:"<<channels[i].control<<"\n";
          /*if (channels[i].joint_)
              gzdbg<<channels[i].joint_name <<": "<<channels[i].joint_->Position(0)
                  <<" | ref: "<<channels[i].input_reference_
                 <<" | srv_ref: "<<channels[i].srv.ref<<"\n";*/
      }
      gzdbg<<"\n";
  }
}

bool GazeboMavlinkInterface::IsRunning()
{
#if GAZEBO_MAJOR_VERSION >= 8
    return world_->Running();
#else
    return world_->GetRunning();
#endif
}

void GazeboMavlinkInterface::open() {
  try{
    serial_dev.open(device_);
    serial_dev.set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
    serial_dev.set_option(boost::asio::serial_port_base::character_size(8));
    serial_dev.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_dev.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_dev.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    gzdbg << "Opened serial device " << device_ << "\n";
  }
  catch (boost::system::system_error &err) {
    gzerr <<"Error opening serial device: " << err.what() << "\n";
  }
}

void GazeboMavlinkInterface::close()
{
  lock_guard lock(mutex);
  if (!is_open())
    return;

  io_service.stop();
  serial_dev.close();

  if (io_thread.joinable())
    io_thread.join();
}

void GazeboMavlinkInterface::do_read(void)
{
  serial_dev.async_read_some(boost::asio::buffer(rx_buf), boost::bind(
      &GazeboMavlinkInterface::parse_buffer, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred
      )
  );
}

// Based on MAVConnInterface::parse_buffer in MAVROS
void GazeboMavlinkInterface::parse_buffer(const boost::system::error_code& err, std::size_t bytes_t){
  mavlink_status_t status;
  mavlink_message_t message;
  uint8_t *buf = this->rx_buf.data();

  assert(rx_buf.size() >= bytes_t);

  for(; bytes_t > 0; bytes_t--)
  {
    auto c = *buf++;

    auto msg_received = static_cast<Framing>(mavlink_frame_char_buffer(&m_buffer, &m_status, c, &message, &status));
    if (msg_received == Framing::bad_crc || msg_received == Framing::bad_signature) {
      _mav_parse_error(&m_status);
      m_status.msg_received = MAVLINK_FRAMING_INCOMPLETE;
      m_status.parse_state = MAVLINK_PARSE_STATE_IDLE;
      if (c == MAVLINK_STX) {
        m_status.parse_state = MAVLINK_PARSE_STATE_GOT_STX;
        m_buffer.len = 0;
        mavlink_start_checksum(&m_buffer);
      }
    }

    if(msg_received != Framing::incomplete){
      // send to gcs
      //std::thread::id this_id = std::this_thread::get_id();
      //gzdbg<<"parse_buffer thread ID: "<<this_id<<std::endl;
      send_mavlink_message(&message, qgc_udp_port_);
      //gzdbg<<"parse_buffer calls handle message"<<std::endl;
      handle_message(&message);
    }
  }
  do_read();
}

void GazeboMavlinkInterface::do_write(bool check_tx_state){

  //std::thread::id this_id = std::this_thread::get_id();
  //gzdbg<<"do_write thread ID: "<<this_id<<std::endl;

  //gzdbg<<"pre-check, tx_in_progress: "<<tx_in_progress<<" | check_tx_state: "<<check_tx_state<<std::endl;

  if (check_tx_state && tx_in_progress)
    return;

  lock_guard lock(mutex);

  //gzdbg<<"do_write locking"<<std::endl;

  if (tx_q.empty())
    return;

  tx_in_progress = true;
  auto &buf_ref = tx_q.front();

  serial_dev.async_write_some(
    boost::asio::buffer(buf_ref.dpos(), buf_ref.nbytes()), [this, &buf_ref] (boost::system::error_code error,   size_t bytes_transferred)
    {
      //std::thread::id this_id = std::this_thread::get_id();
      //gzdbg<<"async_write_some thread ID: "<<this_id<<std::endl;

      assert(bytes_transferred <= buf_ref.len);
      if(error) {
        gzerr << "Serial error: " << error.message() << "\n";
      return;
      }

    lock_guard lock(mutex);
    //gzdbg<<"write handler locking"<<std::endl;

    if (tx_q.empty()) {
      tx_in_progress = false;
      return;
    }
    //gzdbg << "transferred bytes: " << bytes_transferred <<std::endl;
    //gzdbg << "buf_ref.pos: " << buf_ref.pos <<std::endl;

    buf_ref.pos += bytes_transferred;
    if (buf_ref.nbytes() == 0) {
      tx_q.pop_front();
      //gzdbg << "pop" <<std::endl;
    }

    if (!tx_q.empty()) {
      do_write(false);
    }
    else {
      tx_in_progress = false;
    }
  });
  // async_write_some returns immediately
  //gzdbg<<"async_write_some called"<<std::endl;

  /*
  if (!tx_q.empty()) {
    tx_q.pop_front();
  }
  */
}

}

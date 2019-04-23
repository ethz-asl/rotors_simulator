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
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <atomic>
#include <chrono>
#include <memory>
#include <sstream>
#include <cassert>
#include <stdexcept>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/shared_array.hpp>
#include <boost/system/system_error.hpp>

#include <iostream>
#include <random>
#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>

#include <Eigen/Eigen>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <ignition/math.hh>
#include <sdf/sdf.hh>
#include "common.h"
#include <CommandMotorSpeed.pb.h>
//#include <MotorSpeed.pb.h>
#include <Imu.pb.h>
#include <OpticalFlow.pb.h>
//#include <Range.pb.h>
//#include <SITLGps.pb.h>
//#include <IRLock.pb.h>
//#include <Groundtruth.pb.h>
//#include <Odometry.pb.h>
#include <Lidar.pb.h>

#include <mavlink/v2.0/common/mavlink.h>
#include "msgbuffer.h"

#include <rotors_gazebo_plugins/geo_mag_declination_tmp.h>

static const uint32_t kDefaultMavlinkUdpPort = 14560;
static const uint32_t kDefaultMavlinkTcpPort = 4560;
static const uint32_t kDefaultQGCUdpPort = 14550;

using lock_guard = std::lock_guard<std::recursive_mutex>;
static constexpr auto kDefaultDevice = "/dev/ttyACM0";
static constexpr auto kDefaultBaudRate = 921600;

//! Maximum buffer size with padding for CRC bytes (280 + padding)
static constexpr ssize_t MAX_SIZE = MAVLINK_MAX_PACKET_LEN + 16;
static constexpr size_t MAX_TXQ_SIZE = 1000;

namespace gazebo {

typedef const boost::shared_ptr<const gz_mav_msgs::CommandMotorSpeed> CommandMotorSpeedPtr;
//typedef const boost::shared_ptr<const nav_msgs::msgs::Odometry> OdomPtr;
//typedef const boost::shared_ptr<const sensor_msgs::msgs::Groundtruth> GtPtr;
typedef const boost::shared_ptr<const gz_sensor_msgs::Imu> ImuPtr;
//typedef const boost::shared_ptr<const sensor_msgs::msgs::IRLock> IRLockPtr;
typedef const boost::shared_ptr<const opticalFlow_msgs::msgs::opticalFlow> OpticalFlowPtr;
//typedef const boost::shared_ptr<const sensor_msgs::msgs::Range> SonarPtr;
//typedef const boost::shared_ptr<const lidar_msgs::msgs::lidar> LidarPtr;
//typedef const boost::shared_ptr<const sensor_msgs::msgs::SITLGps> GpsPtr;

// Default values
// static const std::string kDefaultNamespace = "";

// This just proxies the motor commands from command/motor_speed to the single motors via internal
// ConsPtr passing, such that the original commands don't have to go n_motors-times over the wire.
static const std::string kDefaultMotorVelocityReferencePubTopic = "/gazebo/command/motor_speed";

static const std::string kDefaultImuTopic = "/imu";
//static const std::string kDefaultLidarTopic = "/link/lidar";
static const std::string kDefaultOpticalFlowTopic = "/px4flow/link/opticalFlow";
//static const std::string kDefaultSonarTopic = "/sonar_model/link/sonar";
//static const std::string kDefaultIRLockTopic = "/camera/link/irlock";
//static const std::string kDefaultGPSTopic = "/gps";
//static const std::string kDefaultVisionTopic = "/vision_odom";

//! Rx packer framing status. (same as @p mavlink::mavlink_framing_t)
enum class Framing : uint8_t {
	incomplete = MAVLINK_FRAMING_INCOMPLETE,
	ok = MAVLINK_FRAMING_OK,
	bad_crc = MAVLINK_FRAMING_BAD_CRC,
	bad_signature = MAVLINK_FRAMING_BAD_SIGNATURE,
};

class GazeboMavlinkInterface : public ModelPlugin {
public:
  GazeboMavlinkInterface() : ModelPlugin(),
    dbgCounter(1),
    received_first_actuator_(false),
    namespace_(kDefaultNamespace),
    protocol_version_(2.0),
    motor_velocity_reference_pub_topic_(kDefaultMotorVelocityReferencePubTopic),
    use_propeller_pid_(false),
    use_elevator_pid_(false),
    use_left_elevon_pid_(false),
    use_right_elevon_pid_(false),
    //vehicle_is_tailsitter_(false),
    //send_vision_estimation_(false),
    //send_odometry_(false),
    imu_sub_topic_(kDefaultImuTopic),
    opticalFlow_sub_topic_(kDefaultOpticalFlowTopic),
    //lidar_sub_topic_(kDefaultLidarTopic),
    //sonar_sub_topic_(kDefaultSonarTopic),
    //irlock_sub_topic_(kDefaultIRLockTopic),
    //gps_sub_topic_(kDefaultGPSTopic),
    //vision_sub_topic_(kDefaultVisionTopic),
    model_ {},
    world_(nullptr),
    left_elevon_joint_(nullptr),
    right_elevon_joint_(nullptr),
    elevator_joint_(nullptr),
    propeller_joint_(nullptr),
    gimbal_yaw_joint_(nullptr),
    gimbal_pitch_joint_(nullptr),
    gimbal_roll_joint_(nullptr),
    input_offset_ {},
    input_scaling_ {},
    zero_position_disarmed_ {},
    zero_position_armed_ {},
    input_index_ {},
    groundtruth_lat_rad(0.0),
    groundtruth_lon_rad(0.0),
    groundtruth_altitude(0.0),
    mavlink_udp_port_(kDefaultMavlinkUdpPort),
    mavlink_tcp_port_(kDefaultMavlinkTcpPort),
    tcp_client_fd_(0),
    use_tcp_(false),
    qgc_udp_port_(kDefaultQGCUdpPort),
    serial_enabled_(false),
    tx_q {},
    rx_buf {},
    m_status {},
    m_buffer {},
    io_service(),
    serial_dev(io_service),
    device_(kDefaultDevice),
    baudrate_(kDefaultBaudRate),
    tx_in_progress(false),
    hil_mode_(false),
    hil_state_level_(false),
    baro_rnd_y2_(0.0),
    baro_rnd_use_last_(false)
    {}

  ~GazeboMavlinkInterface();

  void Publish();

protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo&  /*_info*/);

private:
  unsigned int dbgCounter;
  bool received_first_actuator_;
  Eigen::VectorXd input_reference_;

  float protocol_version_;

  std::string namespace_;
  std::string motor_velocity_reference_pub_topic_;
  std::string mavlink_control_sub_topic_;
  std::string link_name_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr motor_velocity_reference_pub_;
  transport::SubscriberPtr mav_control_sub_;

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::JointPtr left_elevon_joint_;
  physics::JointPtr right_elevon_joint_;
  physics::JointPtr elevator_joint_;
  physics::JointPtr propeller_joint_;
  physics::JointPtr gimbal_yaw_joint_;
  physics::JointPtr gimbal_pitch_joint_;
  physics::JointPtr gimbal_roll_joint_;
  common::PID propeller_pid_;
  common::PID elevator_pid_;
  common::PID left_elevon_pid_;
  common::PID right_elevon_pid_;
  bool use_propeller_pid_;
  bool use_elevator_pid_;
  bool use_left_elevon_pid_;
  bool use_right_elevon_pid_;

  //bool vehicle_is_tailsitter_;

  //bool send_vision_estimation_;
  //bool send_odometry_;

  std::vector<physics::JointPtr> joints_;
  std::vector<common::PID> pids_;

  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread();
  void ImuCallback(ImuPtr& imu_msg);
  //void GpsCallback(GpsPtr& gps_msg);
  //void GroundtruthCallback(GtPtr& groundtruth_msg);
  //void LidarCallback(LidarPtr& lidar_msg);
  //void SonarCallback(SonarPtr& sonar_msg);
  void OpticalFlowCallback(OpticalFlowPtr& opticalFlow_msg);
  //void IRLockCallback(IRLockPtr& irlock_msg);
  //void VisionCallback(OdomPtr& odom_msg);
  void send_mavlink_message(const mavlink_message_t *message, const int destination_port = 0);
  void handle_message(mavlink_message_t *msg, bool &received_actuator);
  void pollForMAVLinkMessages();
  void SendSensorMessages();
  void handle_control(double _dt);
  bool IsRunning();


  // Serial interface
  void open();
  void close();
  void do_read();
  void parse_buffer(const boost::system::error_code& err, std::size_t bytes_t);
  void do_write(bool check_tx_state);
  inline bool is_open(){
    return serial_dev.is_open();
  }

  static const unsigned n_out_max = 16;
  double alt_home = 488.0;   // meters

  double input_offset_[n_out_max];
  double input_scaling_[n_out_max];
  std::string joint_control_type_[n_out_max];
  std::string gztopic_[n_out_max];
  double zero_position_disarmed_[n_out_max];
  double zero_position_armed_[n_out_max];
  int input_index_[n_out_max];
  transport::PublisherPtr joint_control_pub_[n_out_max];

  transport::SubscriberPtr imu_sub_;
  //transport::SubscriberPtr lidar_sub_;
  //transport::SubscriberPtr sonar_sub_;
  transport::SubscriberPtr opticalFlow_sub_;
  //transport::SubscriberPtr irlock_sub_;
  //transport::SubscriberPtr gps_sub_;
  //transport::SubscriberPtr groundtruth_sub_;
  //transport::SubscriberPtr vision_sub_;

  std::string imu_sub_topic_;
  //std::string lidar_sub_topic_;
  std::string opticalFlow_sub_topic_;
  //std::string sonar_sub_topic_;
  //std::string irlock_sub_topic_;
  //std::string gps_sub_topic_;
  //std::string groundtruth_sub_topic_;
  //std::string vision_sub_topic_;

  std::mutex last_imu_message_mutex_ {};
  std::condition_variable last_imu_message_cond_ {};
  gz_sensor_msgs::Imu last_imu_message_;
  common::Time last_time_;
  common::Time last_imu_time_;
  common::Time last_actuator_time_;

  double groundtruth_lat_rad;
  double groundtruth_lon_rad;
  double groundtruth_altitude;

  double imu_update_interval_ = 0.004; ///< Used for non-lockstep

  ignition::math::Vector3d gravity_W_;
  ignition::math::Vector3d velocity_prev_W_;
  ignition::math::Vector3d mag_d_;

  std::default_random_engine rand_;
  std::normal_distribution<float> randn_;

  int _fd;
  struct sockaddr_in _myaddr;     ///< The locally bound address
  socklen_t _myaddr_len;
  struct sockaddr_in _srcaddr;    ///< SITL instance
  socklen_t _srcaddr_len;
  unsigned char _buf[65535];
  struct pollfd fds_[1];

  double optflow_distance;
  double sonar_distance;

  in_addr_t mavlink_addr_;
  int mavlink_udp_port_;
  int mavlink_tcp_port_;
  int tcp_client_fd_;
  bool use_tcp_ = false;

  in_addr_t qgc_addr_;
  int qgc_udp_port_;

  bool enable_lockstep_ = false;
  double speed_factor_ = 1.0;
  int64_t previous_imu_seq_ = 0;

  // Serial interface
  mavlink_status_t m_status;
  mavlink_message_t m_buffer;
  bool serial_enabled_;
  std::thread io_thread;
  std::string device_;
  std::array<uint8_t, MAX_SIZE> rx_buf;
  std::recursive_mutex mutex;
  unsigned int baudrate_;
  std::atomic<bool> tx_in_progress;
  std::deque<MsgBuffer> tx_q;
  boost::asio::io_service io_service;
  boost::asio::serial_port serial_dev;

  bool hil_mode_;
  bool hil_state_level_;

  // state variables for baro pressure sensor random noise generator
  double baro_rnd_y2_;
  bool baro_rnd_use_last_;
  };
}

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
#include <atomic>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/shared_array.hpp>
#include <boost/system/system_error.hpp>
#include <cassert>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

#include <cstdlib>
#include <iostream>
#include <math.h>
#include <netinet/in.h>
#include <random>
#include <stdio.h>
#include <string>
#include <sys/socket.h>

#include <Eigen/Eigen>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <CommandMotorSpeed.pb.h>
#include <Imu.pb.h>
#include <Lidar.pb.h>
#include <OpticalFlow.pb.h>
#include <ignition/math.hh>
#include <sdf/sdf.hh>
#include "common.h"

#include <mavlink/v2.0/common/mavlink.h>
#include "msgbuffer.h"

#include <rotors_gazebo_plugins/geo_mag_declination.h>

static const uint32_t kDefaultMavlinkUdpPort = 14560;
static const uint32_t kDefaultQGCUdpPort = 14550;

using lock_guard = std::lock_guard<std::recursive_mutex>;
static constexpr auto kDefaultDevice = "/dev/ttyACM0";
static constexpr auto kDefaultBaudRate = 921600;
static constexpr double kDefaultImuInterval = 0.004;

//! Maximum buffer size with padding for CRC bytes (280 + padding)
static constexpr ssize_t MAX_SIZE = MAVLINK_MAX_PACKET_LEN + 16;
static constexpr size_t MAX_TXQ_SIZE = 1000;

namespace gazebo {

typedef const boost::shared_ptr<const gz_mav_msgs::CommandMotorSpeed>
    CommandMotorSpeedPtr;
typedef const boost::shared_ptr<const gz_sensor_msgs::Imu> ImuPtr;
typedef const boost::shared_ptr<const lidar_msgs::msgs::lidar> LidarPtr;
typedef const boost::shared_ptr<const opticalFlow_msgs::msgs::opticalFlow>
    OpticalFlowPtr;

// Default values
// static const std::string kDefaultNamespace = "";

// This just proxies the motor commands from command/motor_speed to the single
// motors via internal ConsPtr passing, such that the original commands don't
// have to go n_motors-times over the wire.
static const std::string kDefaultMotorVelocityReferencePubTopic =
    "/gazebo/command/motor_speed";

static const std::string kDefaultImuTopic = "/imu";
static const std::string kDefaultLidarTopic = "/link/lidar";
static const std::string kDefaultOpticalFlowTopic = "/px4flow/link/opticalFlow";

//! Rx packer framing status. (same as @p mavlink::mavlink_framing_t)
enum class Framing : uint8_t {
  incomplete = MAVLINK_FRAMING_INCOMPLETE,
  ok = MAVLINK_FRAMING_OK,
  bad_crc = MAVLINK_FRAMING_BAD_CRC,
  bad_signature = MAVLINK_FRAMING_BAD_SIGNATURE,
};

class GazeboMavlinkInterface : public ModelPlugin {
 public:
  GazeboMavlinkInterface()
      : ModelPlugin(),
        received_first_reference_(false),
        namespace_(kDefaultNamespace),
        protocol_version_(2.0),
        motor_velocity_reference_pub_topic_(
            kDefaultMotorVelocityReferencePubTopic),
        use_propeller_pid_(false),
        use_elevator_pid_(false),
        use_left_elevon_pid_(false),
        use_right_elevon_pid_(false),
        imu_sub_topic_(kDefaultImuTopic),
        opticalFlow_sub_topic_(kDefaultOpticalFlowTopic),
        lidar_sub_topic_(kDefaultLidarTopic),
        model_{},
        world_(nullptr),
        left_elevon_joint_(nullptr),
        right_elevon_joint_(nullptr),
        elevator_joint_(nullptr),
        propeller_joint_(nullptr),
        gimbal_yaw_joint_(nullptr),
        gimbal_pitch_joint_(nullptr),
        gimbal_roll_joint_(nullptr),
        input_offset_{},
        input_scaling_{},
        zero_position_disarmed_{},
        zero_position_armed_{},
        input_index_{},
        lat_rad_(0.0),
        lon_rad_(0.0),
        mavlink_udp_port_(kDefaultMavlinkUdpPort),
        qgc_udp_port_(kDefaultMavlinkUdpPort),
        serial_enabled_(false),
        tx_q_{},
        rx_buf_{},
        m_status_{},
        m_buffer_{},
        io_service_(),
        serial_dev_(io_service_),
        device_(kDefaultDevice),
        baudrate_(kDefaultBaudRate),
        hil_mode_(false),
        hil_state_level_(false) {}

  ~GazeboMavlinkInterface();

  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:
  bool received_first_reference_;
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

  std::vector<physics::JointPtr> joints_;
  std::vector<common::PID> pids_;

  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread();
  void ImuCallback(ImuPtr& imu_msg);
  void LidarCallback(LidarPtr& lidar_msg);
  void OpticalFlowCallback(OpticalFlowPtr& opticalFlow_msg);
  void send_mavlink_message(
      const mavlink_message_t* message, const int destination_port = 0);
  void handle_message(mavlink_message_t* msg);
  void pollForMAVLinkMessages(double _dt, uint32_t _timeoutMs);

  // Serial interface
  void open();
  void close();
  void do_read();
  void parse_buffer(const boost::system::error_code& err, std::size_t bytes_t);
  void do_write(bool check_tx_state);
  inline bool is_open() {
    return serial_dev_.is_open();
  }

  // Set the number of BLDC motors and tilting servos,
  // if using flags to differentiate.
  static const size_t kNOutMax = 18u;
  static const size_t kNumMotors = 12u;
  static const size_t kNumServos = 6u;

  static const u_int32_t kMotorSpeedFlag = 1;
  static const u_int32_t kServoPositionFlag = 2;

  double alt_home = 488.0;  // meters

  unsigned _rotor_count;

  double input_offset_[kNOutMax];
  double input_scaling_[kNOutMax];
  std::string joint_control_type_[kNOutMax];
  std::string gztopic_[kNOutMax];
  double zero_position_disarmed_[kNOutMax];
  double zero_position_armed_[kNOutMax];
  int input_index_[kNOutMax];
  transport::PublisherPtr joint_control_pub_[kNOutMax];

  transport::SubscriberPtr imu_sub_;
  transport::SubscriberPtr lidar_sub_;
  transport::SubscriberPtr sonar_sub_;
  transport::SubscriberPtr opticalFlow_sub_;

  std::string imu_sub_topic_;
  std::string lidar_sub_topic_;
  std::string opticalFlow_sub_topic_;

  common::Time last_time_;
  common::Time last_imu_time_;
  common::Time last_actuator_time_;
  double imu_update_interval_ = kDefaultImuInterval;
  double lat_rad_;
  double lon_rad_;

  void handle_control(double _dt);

  ignition::math::Vector3d gravity_W_;
  ignition::math::Vector3d velocity_prev_W_;
  ignition::math::Vector3d mag_d_;

  std::default_random_engine rand_;
  std::normal_distribution<float> randn_;

  int _fd;
  struct sockaddr_in myaddr_;   ///< The locally bound address
  struct sockaddr_in srcaddr_;  ///< SITL instance
  socklen_t addrlen_;
  unsigned char buf_[65535];
  struct pollfd fds[1];

  // so we dont have to do extra callbacks
  ignition::math::Vector3d optflow_gyro_{};
  double optflow_distance_;

  in_addr_t mavlink_addr_;
  int mavlink_udp_port_;

  in_addr_t qgc_addr_;
  int qgc_udp_port_;

  // Serial interface
  mavlink_status_t m_status_;
  mavlink_message_t m_buffer_;
  bool serial_enabled_;
  std::thread io_thread_;
  std::string device_;
  std::array<uint8_t, MAX_SIZE> rx_buf_;
  std::recursive_mutex mutex_;
  unsigned int baudrate_;
  std::atomic<bool> tx_in_progress_;
  std::deque<MsgBuffer> tx_q_;
  boost::asio::io_service io_service_;
  boost::asio::serial_port serial_dev_;

  bool hil_mode_;
  bool hil_state_level_;
};
}  // namespace gazebo

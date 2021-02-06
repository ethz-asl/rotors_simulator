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

#include <cstdlib>
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
#include <Actuators.pb.h>
#include <Imu.pb.h>
#include <OpticalFlow.pb.h>
#include <Lidar.pb.h>
#include <SITLGps.pb.h>
#include <Float32.pb.h>
#include "vector2d.pb.h"
#include "vector3d.pb.h"
#include "WindSpeedBeta.pb.h"

#include <mavlink/v2.0/ASLUAV/mavlink.h>

#include "msgbuffer.h"

#include <rotors_gazebo_plugins/geo_mag_declination_tmp.h>

static const uint32_t kDefaultMavlinkUdpPort = 14560;
static const uint32_t kDefaultQGCUdpPort = 14550;

using lock_guard = std::lock_guard<std::recursive_mutex>;
static constexpr auto kDefaultDevice = "/dev/ttyACM0";
static constexpr auto kDefaultBaudRate = 921600;

//! Maximum buffer size with padding for CRC bytes (280 + padding)
static constexpr ssize_t MAX_SIZE = MAVLINK_MAX_PACKET_LEN + 16;
static constexpr size_t MAX_TXQ_SIZE = 1000;

namespace gazebo {

typedef ignition::math::Vector3d V3D;
typedef ignition::math::Matrix3<double> M3D;

typedef const boost::shared_ptr<const gz_mav_msgs::CommandMotorSpeed> CommandMotorSpeedPtr;
typedef const boost::shared_ptr<const gz_sensor_msgs::Actuators> ActuatorsPtr;
typedef const boost::shared_ptr<const gz_sensor_msgs::Imu> ImuPtr;
typedef const boost::shared_ptr<const lidar_msgs::msgs::lidar> LidarPtr;
typedef const boost::shared_ptr<const opticalFlow_msgs::msgs::opticalFlow> OpticalFlowPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::SITLGps> GpsPtr;
//typedef const boost::shared_ptr<const sensor_msgs::msgs::Groundtruth> GpsGtPtr;
typedef const boost::shared_ptr<const gz_mav_msgs::WindSpeedBeta> WindPtr;
typedef const boost::shared_ptr<const gazebo::msgs::Vector2d> VanePtr;

// Default values
// static const std::string kDefaultNamespace = "";

// This just proxies the motor commands from command/motor_speed to the single motors via internal
// ConsPtr passing, such that the original commands don't have to go n_motors-times over the wire.
static const std::string kDefaultMotorVelocityReferencePubTopic = "/gazebo/command/motor_speed_old";
static const std::string kDefaultActuatorsReferencePubTopic = "/gazebo/command/motor_speed";
static const std::string kDefaultImuTopic = "/imu";
static const std::string kDefaultLidarTopic = "/link/lidar";
static const std::string kDefaultOpticalFlowTopic = "/px4flow/link/opticalFlow";
static const std::string kDefaultGpsTopic = "/gps_hil";
static const std::string kDefaultTrackingPos = "/tracking_pos";

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
    dbg_counter_(0),
    received_first_referenc_(false),
    namespace_(kDefaultNamespace),
    protocol_version_(2.0),
    motor_velocity_reference_pub_topic_(kDefaultMotorVelocityReferencePubTopic),
    actuators_reference_pub_topic_(kDefaultActuatorsReferencePubTopic),
    tracking_pos_pub_topic_(kDefaultTrackingPos),
    use_propeller_pid_(false),
    use_elevator_pid_(false),
    use_left_elevon_pid_(false),
    use_right_elevon_pid_(false),
    imu_sub_topic_(kDefaultImuTopic),
    opticalFlow_sub_topic_(kDefaultOpticalFlowTopic),
    gps_sub_topic_(kDefaultGpsTopic),
    lidar_sub_topic_(kDefaultLidarTopic),
    model_ {},
    world_(nullptr),
    left_elevon_joint_(nullptr),
    right_elevon_joint_(nullptr),
    elevator_joint_(nullptr),
    propeller_joint_(nullptr),
    gimbal_yaw_joint_(nullptr),
    gimbal_pitch_joint_(nullptr),
    gimbal_roll_joint_(nullptr),
    //input_offset_ {},
    //input_scaling_ {},
    //zero_position_disarmed_ {},
    //zero_position_armed_ {},
    //input_index_ {},
    lat_rad_(0.0),
    lon_rad_(0.0),
    gt_lat(0.0),
    gt_lon(0.0),
    gt_alt(0.0),
    mavlink_udp_port_(kDefaultMavlinkUdpPort),
    qgc_udp_port_(kDefaultMavlinkUdpPort),
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
    hil_state_level_(false)
    {}

  ~GazeboMavlinkInterface();

  void Publish();

protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo&  /*_info*/);

private:
  unsigned int dbg_counter_;
  bool received_first_referenc_;
  Eigen::VectorXd input_reference_;

  float protocol_version_;

  std::string namespace_;
  std::string motor_velocity_reference_pub_topic_;
  std::string actuators_reference_pub_topic_;
  std::string tracking_pos_pub_topic_;
  std::string mavlink_control_sub_topic_;
  std::string link_name_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr motor_velocity_reference_pub_;
  transport::PublisherPtr actuators_reference_pub_;
  transport::PublisherPtr tracking_pos_pub_;
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

  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread();
  void ImuCallback(ImuPtr& imu_msg);
  void LidarCallback(LidarPtr& lidar_msg);
  void OpticalFlowCallback(OpticalFlowPtr& opticalFlow_msg);
  void GpsCallback(GpsPtr& gps_msg);
  void GpsGtCallback(GpsPtr& gps_gt_msg);
  void VaneCallback(VanePtr& vane_msg);
  void SendSensorMessages();
  void send_mavlink_message(const mavlink_message_t *message, const int destination_port = 0);
  void handle_message(mavlink_message_t *msg);
  void pollForMAVLinkMessages(double _dt, uint32_t _timeoutMs);
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

  static const unsigned n_out_max = 18;
	static const unsigned n_motors = 12;
	static const unsigned n_servos = 6;
	static const u_int32_t kMotorSpeedFlag = 1;
	static const u_int32_t kServoPositionFlag = 2;

  double alt_home = 488.0;   // meters

  unsigned _rotor_count;

  struct servo {

      servo():
      ref(0),
      slew(1.0/0.3),
      torque(0.85),
      init(false),
      P_vel(0.002),
      P_pos(0.04){}  //slew/max_err

      double ref;       // rad
      double slew;      // rad/s
      double torque;    // Nm
      common::Time last_srv_time;
      bool init;

      double P_vel;
      double P_pos;
  };

  struct ctrl_chan {

      ctrl_chan():
          input_index_(0),
          input_offset_(0),
          input_scaling_(0),
          zero_position_disarmed_(0),
          zero_position_armed_(0),
          input_reference_(0),
          joint_(nullptr),
          joint_control_pub_(nullptr),
          gztopic_("default"){
          pid_.Init(0, 0, 0, 0, 0, 0, 0);
      }

      physics::JointPtr joint_;
      std::string joint_name;

      int input_index_;
      double input_offset_;
      double input_scaling_;
      double zero_position_disarmed_;
      double zero_position_armed_;
      double input_reference_;
      double control;

      std::string joint_control_type_;
      common::PID pid_;
      servo srv;
      std::string gztopic_;
      transport::PublisherPtr joint_control_pub_;
  };

  int n_chan;
  ctrl_chan* channels;

  /*
  std::vector<physics::JointPtr> joints_;
  std::vector<common::PID> pids_;
  double input_offset_[n_out_max];
  double input_scaling_[n_out_max];
  std::string joint_control_type_[n_out_max];
  std::string gztopic_[n_out_max];
  double zero_position_disarmed_[n_out_max];
  double zero_position_armed_[n_out_max];
  int input_index_[n_out_max];
  transport::PublisherPtr joint_control_pub_[n_out_max];
  */

  struct Wind {
      Wind(){}

      transport::SubscriberPtr wind_sub_ = nullptr;
      std::mutex wind_lock;
      std::string wind_topic;

      V3D pos_ned = V3D(0,0,0);
      V3D wind_ned = V3D(0,0,0);
      M3D wind_grad_ned = M3D(0,0,0,0,0,0,0,0,0);

      void Callback(WindPtr& wind){
          std::unique_lock<std::mutex> lock(wind_lock);

          pos_ned = V3D(wind->pos_ned().x(),
                        wind->pos_ned().y(),
                        wind->pos_ned().z());

          wind_ned = V3D(wind->wind_ned().x(),
                         wind->wind_ned().y(),
                         wind->wind_ned().z());

          wind_grad_ned = M3D(wind->wind_grad_ned().xx(),
                              wind->wind_grad_ned().xy(),
                              wind->wind_grad_ned().xz(),
                              wind->wind_grad_ned().yx(),
                              wind->wind_grad_ned().yy(),
                              wind->wind_grad_ned().yz(),
                              wind->wind_grad_ned().zx(),
                              wind->wind_grad_ned().zy(),
                              wind->wind_grad_ned().zz());
          lock.unlock();
      }

      V3D GetWind(V3D p_cp){
          std::unique_lock<std::mutex> lock(wind_lock);   //necessary? atomic V3D?
          V3D wind_local = wind_ned + wind_grad_ned*(p_cp-pos_ned);
          //V3D wind_local = wind_grad_ned*(p_cp-pos_ned);
          lock.unlock();
          return wind_local;
      }
  };

  Wind * wind;
  V3D wind_sens = V3D(0,0,0);
  int n_wind = 0;

  void UpdateWind(V3D w_cp){
      wind_sens = V3D(0,0,0);
      for(int j=0; j<n_wind; j++){
          wind_sens += wind[j].GetWind(w_cp);
      }
  }

  physics::LinkPtr link_ = NULL;
  V3D airspeed_pos_ = V3D(0,0,0);
  V3D barometer_pos_ = V3D(0,0,0);

  transport::SubscriberPtr imu_sub_;
  transport::SubscriberPtr lidar_sub_;
  transport::SubscriberPtr sonar_sub_;
  transport::SubscriberPtr opticalFlow_sub_;
  transport::SubscriberPtr gps_sub_;
  transport::SubscriberPtr gps_gt_sub_; // for ground truth and custom magnetometer
  transport::SubscriberPtr vane_sub_;

  std::string imu_sub_topic_;
  std::string lidar_sub_topic_;
  std::string opticalFlow_sub_topic_;
  std::string gps_sub_topic_;
  std::string gps_gt_sub_topic_;
  std::string vane_sub_topic_;

  common::Time last_time_;
  common::Time last_imu_time_;
  common::Time last_actuator_time_;
  double imu_update_interval_ = 0.004;
  double lat_rad_;
  double lon_rad_;

  double gt_lat;
  double gt_lon;
  double gt_alt;

  std::mutex gps_gt_message_mutex_ {};

  std::mutex last_imu_message_mutex_ {};
  std::condition_variable last_imu_message_cond_ {};
  gz_sensor_msgs::Imu last_imu_message_;
  int64_t previous_imu_seq_ = 0;

  ignition::math::Vector3d gravity_W_;
  ignition::math::Vector3d velocity_prev_W_;
  ignition::math::Vector3d mag_d_;

  std::default_random_engine rand_;
  std::normal_distribution<float> randn_;

  int _fd;
  struct sockaddr_in _myaddr;     ///< The locally bound address
  struct sockaddr_in _srcaddr;    ///< SITL instance
  socklen_t _addrlen;
  unsigned char _buf[65535];
  struct pollfd fds[1];

  struct sockaddr_in _srcaddr_2;  ///< MAVROS

  //so we dont have to do extra callbacks
  ignition::math::Vector3d optflow_gyro {};
  double optflow_distance;
  double sonar_distance;

  in_addr_t mavlink_addr_;
  int mavlink_udp_port_;

  in_addr_t qgc_addr_;
  int qgc_udp_port_;

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

  double lon_last = 0;
  double lat_last = 0;

  uint16_t timing_stats_imu_[11]{0,0,0,0,0,0,0,0,0,0,0};
  uint16_t timing_stats_gps_[11]{0,0,0,0,0,0,0,0,0,0,0};
  common::Time last_wall_time_;
  common::Time last_wall_time_imu_;
  common::Time last_wall_time_gps_;
  double dt_wall_ = 0.004;
  uint16_t send_counter_imu_ = 0;
  uint16_t send_counter_gps_ = 0;

  };
}

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

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
#include <regex>
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

#include <Airspeed.pb.h>
#include <CommandMotorSpeed.pb.h>
#include <MotorSpeed.pb.h>
#include <Imu.pb.h>
#include <OpticalFlow.pb.h>
#include <Range.pb.h>
#include <SITLGps.pb.h>
#include <IRLock.pb.h>
#include <Groundtruth.pb.h>
#include <Odometry.pb.h>
#include <MagneticField.pb.h>
#include <Pressure.pb.h>
#include <Wind.pb.h>
#include <Actuators.pb.h>
#include <Float32.pb.h>

#include "mavlink_interface.h"
#include "msgbuffer.h"

//! Default distance sensor model joint naming
static const std::regex kDefaultLidarModelNaming(".*(lidar|sf10a)(.*)");
static const std::regex kDefaultSonarModelNaming(".*(sonar|mb1240-xl-ez4)(.*)");
static const std::regex kDefaultGPSModelNaming(".*(gps|ublox-neo-7M)(.*)");

namespace gazebo {
/*
typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;
typedef const boost::shared_ptr<const nav_msgs::msgs::Odometry> OdomPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Airspeed> AirspeedPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Groundtruth> GtPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Imu> ImuPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::IRLock> IRLockPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::OpticalFlow> OpticalFlowPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Range> SonarPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Range> LidarPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::SITLGps> GpsPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::MagneticField> MagnetometerPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Pressure> BarometerPtr;
typedef const boost::shared_ptr<const physics_msgs::msgs::Wind> WindPtr;
*/

typedef const boost::shared_ptr<const gz_mav_msgs::CommandMotorSpeed> CommandMotorSpeedPtr;
typedef const boost::shared_ptr<const gz_geometry_msgs::Odometry> OdomPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Airspeed> AirspeedPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Groundtruth> GtPtr;
typedef const boost::shared_ptr<const gz_sensor_msgs::Imu> ImuPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::IRLock> IRLockPtr;
typedef const boost::shared_ptr<const opticalFlow_msgs::msgs::opticalFlow> OpticalFlowPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Range> SonarPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Range> LidarPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::SITLGps> GpsPtr;
typedef const boost::shared_ptr<const gz_sensor_msgs::MagneticField> MagnetometerPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Pressure> BarometerPtr;
typedef const boost::shared_ptr<const physics_msgs::msgs::Wind> WindPtr;

typedef std::pair<const int, const ignition::math::Quaterniond> SensorIdRot_P;
typedef std::map<transport::SubscriberPtr, SensorIdRot_P > Sensor_M;

// Default values
//static const std::string kDefaultNamespace = "";

// This just proxies the motor commands from command/motor_speed to the single motors via internal
// ConsPtr passing, such that the original commands don't have to go n_motors-times over the wire.
static const std::string kDefaultMotorVelocityReferencePubTopic = "/gazebo/command/motor_speed";

static const std::string kDefaultImuTopic = "/imu";
static const std::string kDefaultOpticalFlowTopic = "/px4flow/link/opticalFlow";
static const std::string kDefaultIRLockTopic = "/camera/link/irlock";
static const std::string kDefaultVisionTopic = "/vision_odom";
static const std::string kDefaultMagTopic = "/mag";
static const std::string kDefaultAirspeedTopic = "/airspeed";
static const std::string kDefaultBarometerTopic = "/baro";
static const std::string kDefaultWindTopic = "/world_wind";
static const std::string kDefaultGroundtruthTopic = "/groundtruth";
static const std::string kDefaultGpsTopic = "/gps";
static const std::string kDefaultGpsGtTopic = "/gps_gt";
static const std::string kDefaultVaneTopic = "/vanes";

//! OR operation for the enumeration and unsigned types that returns the bitmask
template<typename A, typename B>
static inline uint32_t operator |(A lhs, B rhs) {
  // make it type safe
  static_assert((std::is_same<A, uint32_t>::value || std::is_same<A, SensorSource>::value),
		"first argument is not uint32_t or SensorSource enum type");
  static_assert((std::is_same<B, uint32_t>::value || std::is_same<B, SensorSource>::value),
		"second argument is not uint32_t or SensorSource enum type");

  return static_cast<uint32_t> (
    static_cast<std::underlying_type<SensorSource>::type>(lhs) |
    static_cast<std::underlying_type<SensorSource>::type>(rhs)
  );
}

class GazeboMavlinkInterfacePx4 : public ModelPlugin {
public:
  GazeboMavlinkInterfacePx4();
  ~GazeboMavlinkInterfacePx4();

  void Publish();

protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo&  /*_info*/);

private:
  bool received_first_actuator_{false};
  Eigen::VectorXd input_reference_;

  float protocol_version_{2.0};

  std::unique_ptr<MavlinkInterface> mavlink_interface_;

  std::string namespace_{kDefaultNamespace};
  std::string motor_velocity_reference_pub_topic_{kDefaultMotorVelocityReferencePubTopic};
  std::string actuators_reference_pub_topic_;
  std::string tracking_pos_pub_topic_;
  std::string mavlink_control_sub_topic_;
  std::string link_name_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr motor_velocity_reference_pub_;
  transport::PublisherPtr actuators_reference_pub_;
  transport::PublisherPtr tracking_pos_pub_;
  transport::SubscriberPtr mav_control_sub_;

  physics::ModelPtr model_{};
  physics::WorldPtr world_{nullptr};

  bool send_vision_estimation_{false};
  bool send_odometry_{false};

  std::vector<physics::JointPtr> joints_;
  std::vector<common::PID> pids_;
  std::vector<double> joint_max_errors_;

  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;
  event::ConnectionPtr sigIntConnection_;

  void ImuCallback(ImuPtr& imu_msg);
  //void GpsCallback(GpsPtr& gps_msg, const int& id);
  void GpsCallback(GpsPtr& gps_msg);
  void GroundtruthCallback(GtPtr& groundtruth_msg);
  void LidarCallback(LidarPtr& lidar_msg, const int& id);
  void SonarCallback(SonarPtr& sonar_msg, const int& id);
  void OpticalFlowCallback(OpticalFlowPtr& opticalFlow_msg);
  void IRLockCallback(IRLockPtr& irlock_msg);
  void VisionCallback(OdomPtr& odom_msg);
  void MagnetometerCallback(MagnetometerPtr& mag_msg);
  void AirspeedCallback(AirspeedPtr& airspeed_msg);
  void BarometerCallback(BarometerPtr& baro_msg);
  void WindVelocityCallback(WindPtr& msg);
  void SendSensorMessages();
  void SendGroundTruth();
  void handle_actuator_controls();
  void handle_control(double _dt);
  bool IsRunning();
  void onSigInt();

  /**
   * @brief Set the MAV_SENSOR_ORIENTATION enum value based on the sensor orientation
   *
   * @param[in] rootModel		The root model where the sensor is attached
   * @param[in] u_Xs				Unit vector of X-axis sensor in `base_link` frame
   * @param[in] sensor_msg	The Mavlink DISTANCE_SENSOR message struct
   */
  template <class T>
  void setMavlinkSensorOrientation(const ignition::math::Vector3d& u_Xs, T& sensor_msg);

  /**
   * @brief A helper class that allows the creation of multiple subscriptions to sensors.
   *	    It gets the sensor link/joint and creates the subscriptions based on those.
   *	    It also allows to set the initial rotation of the sensor, to allow computing
   *	    the sensor orientation quaternion.
   * @details GazeboMsgT  The type of the message that will be subscribed to the Gazebo framework.
   */
  template <typename GazeboMsgT>
  void CreateSensorSubscription(
      void (GazeboMavlinkInterfacePx4::*fp)(const boost::shared_ptr<GazeboMsgT const>&, const int&),
      GazeboMavlinkInterfacePx4* ptr, const physics::Joint_V& joints, physics::ModelPtr& nested_model, const std::regex& model);

  static const unsigned n_out_max = 16;
  static const unsigned n_motors = 12;
  static const unsigned n_servos = 6;
  static const u_int32_t kMotorSpeedFlag = 1;
  static const u_int32_t kServoPositionFlag = 2;

  double input_offset_[n_out_max]{};
  double input_scaling_[n_out_max]{};
  std::string joint_control_type_[n_out_max];
  std::string gztopic_[n_out_max];
  double zero_position_disarmed_[n_out_max]{};
  double zero_position_armed_[n_out_max]{};
  int input_index_[n_out_max]{};
  transport::PublisherPtr joint_control_pub_[n_out_max];

  transport::SubscriberPtr imu_sub_{nullptr};
  transport::SubscriberPtr opticalFlow_sub_{nullptr};
  transport::SubscriberPtr irlock_sub_{nullptr};
  transport::SubscriberPtr groundtruth_sub_{nullptr};
  transport::SubscriberPtr vision_sub_{nullptr};
  transport::SubscriberPtr mag_sub_{nullptr};
  transport::SubscriberPtr airspeed_sub_{nullptr};
  transport::SubscriberPtr baro_sub_{nullptr};
  transport::SubscriberPtr wind_sub_{nullptr};
  transport::SubscriberPtr gps_sub_{nullptr};
  transport::SubscriberPtr gps_gt_sub_{nullptr};
  transport::SubscriberPtr vanes_sub_{nullptr};

  Sensor_M sensor_map_{}; // Map of sensor SubscriberPtr, IDs and orientations

  std::string imu_sub_topic_{kDefaultImuTopic};
  std::string opticalFlow_sub_topic_{kDefaultOpticalFlowTopic};
  std::string irlock_sub_topic_{kDefaultIRLockTopic};
  std::string groundtruth_sub_topic_{kDefaultGroundtruthTopic};
  std::string vision_sub_topic_{kDefaultVisionTopic};
  std::string mag_sub_topic_{kDefaultMagTopic};
  std::string airspeed_sub_topic_{kDefaultAirspeedTopic};
  std::string baro_sub_topic_{kDefaultBarometerTopic};
  std::string wind_sub_topic_{kDefaultWindTopic};
  std::string gps_sub_topic_{kDefaultGpsTopic};
  std::string gps_gt_sub_topic_{kDefaultGpsGtTopic};
  std::string vane_sub_topic_{kDefaultVaneTopic};

  std::mutex last_imu_message_mutex_ {};
  std::condition_variable last_imu_message_cond_ {};
  gz_sensor_msgs::Imu last_imu_message_;
  common::Time last_time_;
  common::Time last_imu_time_;
  common::Time last_actuator_time_;

  bool mag_updated_{false};
  bool baro_updated_{false};
  bool diff_press_updated_{false};

  double groundtruth_lat_rad_{0.0};
  double groundtruth_lon_rad_{0.0};
  double groundtruth_altitude_{0.0};

  double imu_update_interval_{0.004}; ///< Used for non-lockstep

  ignition::math::Vector3d velocity_prev_W_;
  ignition::math::Vector3d mag_n_;
  ignition::math::Vector3d wind_vel_;

  double temperature_{25.0};
  double pressure_alt_{0.0};
  double abs_pressure_{0.0};

  bool close_conn_{false};

  double optflow_distance_{0.0};
  double diff_pressure_{0.0};

  bool enable_lockstep_{false};
  double speed_factor_{1.0};
  int64_t previous_imu_seq_{0};
  unsigned update_skip_factor_{1};

  bool hil_mode_{false};
  bool hil_state_level_{false};

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

};
}

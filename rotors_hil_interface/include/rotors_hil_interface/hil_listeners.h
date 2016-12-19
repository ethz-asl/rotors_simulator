/*
 * Copyright 2016 Pavel Vechersky, ASL, ETH Zurich, Switzerland
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

#ifndef ROTORS_HIL_LISTENERS_H_
#define ROTORS_HIL_LISTENERS_H_

#include <boost/thread/mutex.hpp>
#include <Eigen/Dense>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>

namespace rotors_hil {
// Constants
static constexpr float kAirDensity_kg_per_m3 = 1.18;
static constexpr float kGravityMagnitude_m_per_s2 = 9.8068;
static constexpr float kStandardPressure_Mbar = 1013.25;
static constexpr float kTemperature_degC = 15.0;
static constexpr int kFixNone = 0;
static constexpr int kFix3D = 3;
static constexpr int kHDOP = 100;
static constexpr int kVDOP = 100;
static constexpr int kSatellitesVisible = 4;
static constexpr int kUnknown = 65535;

// Conversions
static constexpr float kDegreesToHil = 1e7;
static constexpr float kFeetToMeters_m_per_ft = 0.3048;
static constexpr float kMetersToCm_cm_per_m = 100.0;
static constexpr float kMetersToMm_mm_per_m = 1000.0;
static constexpr float kPascalToMillibar_mbar_per_Pa = 0.01;
static constexpr float kPressureToAltExp = 0.190284;
static constexpr float kPressureToAltMult = 145366.45;
static constexpr float kSecToNsec_ns_per_s = 1e9;
static constexpr float kTeslaToGauss_G_per_T = 10000.0;

struct HilData {
  HilData() :
      temperature_degC(kTemperature_degC),
      eph_cm(kHDOP),
      epv_cm(kVDOP),
      cog_cdeg(kUnknown),
      ind_airspeed_cm_per_s(0),
      satellites_visible(kSatellitesVisible) {}

  Eigen::Quaterniond att;           // Attitude quaternion
  Eigen::Vector3f acc_m_per_s2;     // Linear acceleration (m/s^2)
  Eigen::Vector3f gyro_rad_per_s;   // Angular rate in body frame (rad / sec)
  Eigen::Vector3f mag_G;            // Magnetic field (Gauss)
  Eigen::Vector3i gps_vel_cm_per_s; // GPS velocity in cm/s in earth-fixed NED frame
  float pressure_abs_mbar;          // Absolute pressure in millibar
  float pressure_diff_mbar;         // Differential pressure (airspeed) in millibar
  float pressure_alt_m;             // Altitude calculated from pressure
  float temperature_degC;           // Temperature in degrees celsius
  uint32_t lat_deg;                 // Latitude (WGS84), in degrees * 1E7
  uint32_t lon_deg;                 // Longitude (WGS84), in degrees * 1E7
  uint32_t alt_mm;                  // Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
  uint16_t eph_cm;                  // GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
  uint16_t epv_cm;                  // GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
  uint16_t vel_cm_per_s;            // GPS ground speed (m/s * 100). If unknown, set to: 65535
  uint16_t cog_cdeg;                // Course over ground (NOT heading, but direction of movement) in degrees * 100. If unknown, set to: 65535
  uint16_t ind_airspeed_cm_per_s;   // Indicated airspeed, expressed as m/s * 100
  uint16_t true_airspeed_cm_per_s;  // True airspeed, expressed as m/s * 100*/
  uint8_t fix_type;                 // < 0-1: no fix, 2: 2D fix, 3: 3D fix
  uint8_t satellites_visible;       // Number of satellites visible. If unknown, set to 255
};

class HilListeners {
 public:
  HilListeners() {}
  virtual ~HilListeners() {}

  /// \brief Callback for handling Air Speed messages.
  /// \param[in] air_speed_msg An Air Speed message.
  /// \param[out] hil_data Pointer to latest data collected for HIL publishing.
  void AirSpeedCallback(const geometry_msgs::TwistStampedConstPtr& air_speed_msg,
                        HilData* hil_data) {
    boost::mutex::scoped_lock lock(mtx_);

    ROS_ASSERT(hil_data);

    Eigen::Vector3d air_velocity(air_speed_msg->twist.linear.x,
                                 air_speed_msg->twist.linear.y,
                                 air_speed_msg->twist.linear.z);

    double air_speed_m_per_s = air_velocity.norm();

    // TODO(pvechersky): Simulate indicated air speed.

    // MAVLINK HIL_STATE_QUATERNION message measured airspeed in cm/s.
    hil_data->ind_airspeed_cm_per_s = air_speed_m_per_s * kMetersToCm_cm_per_m;
    hil_data->true_airspeed_cm_per_s = air_speed_m_per_s * kMetersToCm_cm_per_m;
  }

  /// \brief Callback for handling GPS messages.
  /// \param[in] gps_msg A GPS message.
  /// \param[out] hil_data Pointer to latest data collected for HIL publishing.
  void GpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg,
                   HilData* hil_data) {
    boost::mutex::scoped_lock lock(mtx_);

    ROS_ASSERT(hil_data);

    // MAVLINK HIL_GPS message measures latitude and longitude in degrees * 1e7
    // while altitude is reported in mm.
    hil_data->lat_deg = gps_msg->latitude * kDegreesToHil;
    hil_data->lon_deg = gps_msg->longitude * kDegreesToHil;
    hil_data->alt_mm = gps_msg->altitude * kMetersToMm_mm_per_m;

    hil_data->fix_type =
        (gps_msg->status.status > sensor_msgs::NavSatStatus::STATUS_NO_FIX) ?
            kFix3D : kFixNone;
  }

  /// \brief Callback for handling Ground Speed messages.
  /// \param[in] ground_speed_msg A ground speed message.
  /// \param[out] hil_data Pointer to latest data collected for HIL publishing.
  void GroundSpeedCallback(const geometry_msgs::TwistStampedConstPtr &ground_speed_msg,
                           HilData* hil_data) {
    boost::mutex::scoped_lock lock(mtx_);

    ROS_ASSERT(hil_data);

    // MAVLINK HIL_GPS message measures GPS velocity in cm/s
    hil_data->gps_vel_cm_per_s = Eigen::Vector3i(ground_speed_msg->twist.linear.x,
                                        ground_speed_msg->twist.linear.y,
                                        ground_speed_msg->twist.linear.z) * kMetersToCm_cm_per_m;

    hil_data->vel_cm_per_s = hil_data->gps_vel_cm_per_s.norm();
  }

  /// \brief Callback for handling IMU messages.
  /// \param[in] imu_msg An IMU message.
  /// \param[out] hil_data Pointer to latest data collected for HIL publishing.
  void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg,
                   HilData* hil_data) {
    boost::mutex::scoped_lock lock(mtx_);

    ROS_ASSERT(hil_data);

    hil_data->acc_m_per_s2 = Eigen::Vector3f(imu_msg->linear_acceleration.x,
                                    imu_msg->linear_acceleration.y,
                                    imu_msg->linear_acceleration.z);

    hil_data->att = Eigen::Quaterniond(imu_msg->orientation.w,
                                       imu_msg->orientation.x,
                                       imu_msg->orientation.y,
                                       imu_msg->orientation.z);

    hil_data->gyro_rad_per_s = Eigen::Vector3f(imu_msg->angular_velocity.x,
                                     imu_msg->angular_velocity.y,
                                     imu_msg->angular_velocity.z);
  }

  /// \brief Callback for handling Magnetometer messages.
  /// \param[in] mag_msg A Magnetometer message.
  /// \param[out] hil_data Pointer to latest data collected for HIL publishing.
  void MagCallback(const sensor_msgs::MagneticFieldConstPtr &mag_msg,
                   HilData* hil_data) {
    boost::mutex::scoped_lock lock(mtx_);

    ROS_ASSERT(hil_data);

    // ROS magnetic field sensor message is in Tesla, while
    // MAVLINK HIL_SENSOR message measures magnetic field in Gauss.
    // 1 Tesla = 10000 Gauss
    hil_data->mag_G = Eigen::Vector3f(mag_msg->magnetic_field.x,
                                    mag_msg->magnetic_field.y,
                                    mag_msg->magnetic_field.z) * kTeslaToGauss_G_per_T;
  }

  /// \brief Callback for handling Air Pressure messages.
  /// \param[in] pressure_msg An Air Pressure message.
  /// \param[out] hil_data Pointer to latest data collected for HIL publishing.
  void PressureCallback(const sensor_msgs::FluidPressureConstPtr &pressure_msg,
                        HilData* hil_data) {
    boost::mutex::scoped_lock lock(mtx_);

    ROS_ASSERT(hil_data);

    // ROS fluid pressure sensor message is in Pascals, while
    // MAVLINK HIL_SENSOR message measures fluid pressure in millibar.
    // 1 Pascal = 0.01 millibar
    float pressure_mbar = pressure_msg->fluid_pressure * kPascalToMillibar_mbar_per_Pa;
    hil_data->pressure_abs = pressure_mbar;

    // From the following formula: p_stag - p_static = 0.5 * rho * v^2
    // HIL air speed is in cm/s and is converted to m/s for the purpose of
    // computing pressure.
    hil_data->pressure_diff_mbar = 0.5 * kAirDensity_kg_per_m3 * hil_data->ind_airspeed_cm_per_s *
            hil_data->ind_airspeed_cm_per_s * kPascalToMillibar_mbar_per_Pa /
            (kMetersToCm_cm_per_m * kMetersToCm_cm_per_m);

    hil_data->pressure_alt_m =
        (1 - pow((pressure_mbar / kStandardPressure_Mbar), kPressureToAltExp)) *
            kPressureToAltMult * kFeetToMeters_m_per_ft;
  }

 private:
  /// Mutex lock for thread safety of writing hil data.
  boost::mutex mtx_;
};

}

#endif // ROTORS_HIL_LISTENERS_H_

/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
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

#ifndef ROTORS_GAZEBO_PLUGINS_COMMON_H_
#define ROTORS_GAZEBO_PLUGINS_COMMON_H_

#include <Eigen/Dense>
#include <gazebo/gazebo.hh>
#include <tinyxml.h>
#include <typeinfo>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {

//===============================================================================================//
//========================================= DEBUGGING ===========================================//
//===============================================================================================//

/// \addtogroup Debug Print Switches
/// @{

// The following boolean constants enable/disable debug printing when certain plugin methods are called.
// Suitable for debugging purposes. Left on permanently can swamp std::out and can crash Gazebo.

static const bool kPrintOnPluginLoad    = false;
static const bool kPrintOnUpdates       = false;
static const bool kPrintOnMsgCallback   = false;

/// @}

// Default values
static const std::string kDefaultNamespace = "";
static constexpr double kDefaultRotorVelocitySlowdownSim = 10.0;

//===============================================================================================//
//================================== TOPICS FOR ROS INTERFACE ===================================//
//===============================================================================================//

// These should perhaps be defined in an .sdf/.xacro file instead?
static const std::string kConnectGazeboToRosSubtopic = "connect_gazebo_to_ros_subtopic";
static const std::string kConnectRosToGazeboSubtopic = "connect_ros_to_gazebo_subtopic";

/// \brief    Special-case topic for ROS interface plugin to listen to (if present)
///           and broadcast transforms to the ROS system.
static const std::string kBroadcastTransformSubtopic = "broadcast_transform";


/// \brief      Obtains a parameter from sdf.
/// \param[in]  sdf           Pointer to the sdf object.
/// \param[in]  name          Name of the parameter.
/// \param[out] param         Param Variable to write the parameter to.
/// \param[in]  default_value Default value, if the parameter not available.
/// \param[in]  verbose       If true, gzerror if the parameter is not available.
template<class T>
bool getSdfParam(sdf::ElementPtr sdf, const std::string& name, T& param, const T& default_value, const bool& verbose =
                     false) {
  if (sdf->HasElement(name)) {
    param = sdf->GetElement(name)->Get<T>();
    return true;
  }
  else {
    param = default_value;
    if (verbose)
      gzerr << "[rotors_gazebo_plugins] Please specify a value for parameter \"" << name << "\".\n";
  }
  return false;
}

template <typename T>
void model_param(const std::string& world_name, const std::string& model_name, const std::string& param, T& param_value)
{
  TiXmlElement* e_param = nullptr;
  TiXmlElement* e_param_tmp = nullptr;
  std::string dbg_param;

  TiXmlDocument doc(world_name + ".xml");
  if (doc.LoadFile())
  {
    TiXmlHandle h_root(doc.RootElement());

    TiXmlElement* e_model = h_root.FirstChild("model").Element();

    for( e_model; e_model; e_model=e_model->NextSiblingElement("model") )
    {
      const char* attr_name = e_model->Attribute("name");
      if (attr_name)
      {
        //specific
        if (model_name.compare(attr_name) == 0)
        {
          e_param_tmp = e_model->FirstChildElement(param);
          if (e_param_tmp)
          {
            e_param = e_param_tmp;
            dbg_param = "";
          }
          break;
        }
      }
      else
      {
        //common
        e_param = e_model->FirstChildElement(param);
        dbg_param = "common ";
      }
    }

    if (e_param)
    {
      std::istringstream iss(e_param->GetText());
      iss >> param_value;

      gzdbg << model_name << " model: " << dbg_param << "parameter " << param << " = " << param_value << " from " << doc.Value() << "\n";
    }
  }

}

/**
 * \brief Get a math::Angle as an angle from [0, 360)
 */
inline double GetDegrees360(const ignition::math::Angle& angle) {
  double degrees = angle.Degree();
  while (degrees < 0.) degrees += 360.0;
  while (degrees >= 360.0) degrees -= 360.0;
  return degrees;
}

}  // namespace gazebo


/// \brief    This class can be used to apply a first order filter on a signal.
///           It allows different acceleration and deceleration time constants.
/// \details
///           Short reveiw of discrete time implementation of first order system:
///           Laplace:
///             X(s)/U(s) = 1/(tau*s + 1)
///           continous time system:
///             dx(t) = (-1/tau)*x(t) + (1/tau)*u(t)
///           discretized system (ZoH):
///             x(k+1) = exp(samplingTime*(-1/tau))*x(k) + (1 - exp(samplingTime*(-1/tau))) * u(k)
template <typename T>
class FirstOrderFilter {

 public:
  FirstOrderFilter(double timeConstantUp, double timeConstantDown, T initialState):
      timeConstantUp_(timeConstantUp),
      timeConstantDown_(timeConstantDown),
      previousState_(initialState) {}

  /// \brief    This method will apply a first order filter on the inputState.
  T updateFilter(T inputState, double samplingTime) {

    T outputState;
    if (inputState > previousState_) {
      // Calcuate the outputState if accelerating.
      double alphaUp = exp(-samplingTime / timeConstantUp_);
      // x(k+1) = Ad*x(k) + Bd*u(k)
      outputState = alphaUp * previousState_ + (1 - alphaUp) * inputState;

    }
    else {
      // Calculate the outputState if decelerating.
      double alphaDown = exp(-samplingTime / timeConstantDown_);
      outputState = alphaDown * previousState_ + (1 - alphaDown) * inputState;
    }
    previousState_ = outputState;
    return outputState;

  }

  ~FirstOrderFilter() {}

 protected:
  double timeConstantUp_;
  double timeConstantDown_;
  T previousState_;
};

/// Returns scalar value constrained by (min_val, max_val)
/// \brief    Computes a quaternion from the 3-element small angle approximation theta.
template<typename Scalar>
static inline constexpr const Scalar &constrain(const Scalar &val, const Scalar &min_val, const Scalar &max_val) {
  return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
}


/// \brief    Computes a quaternion from the 3-element small angle approximation theta.
template<class Derived>
Eigen::Quaternion<typename Derived::Scalar> QuaternionFromSmallAngle(const Eigen::MatrixBase<Derived> & theta) {
  typedef typename Derived::Scalar Scalar;
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  const Scalar q_squared = theta.squaredNorm() / 4.0;

  if (q_squared < 1) {
    return Eigen::Quaternion<Scalar>(sqrt(1 - q_squared), theta[0] * 0.5, theta[1] * 0.5, theta[2] * 0.5);
  }
  else {
    const Scalar w = 1.0 / sqrt(1 + q_squared);
    const Scalar f = w * 0.5;
    return Eigen::Quaternion<Scalar>(w, theta[0] * f, theta[1] * f, theta[2] * f);
  }
}

template<class In, class Out>
void copyPosition(const In& in, Out* out) {
  out->x = in.x;
  out->y = in.y;
  out->z = in.z;
}

#if GAZEBO_MAJOR_VERSION < 9
inline ignition::math::Vector3d ignitionFromGazeboMath(const gazebo::math::Vector3 &vec_gz) {
  return ignition::math::Vector3d(vec_gz.x, vec_gz.y, vec_gz.z);
}

inline ignition::math::Pose3d ignitionFromGazeboMath(const gazebo::math::Pose &pose_gz) {

  return ignition::math::Pose3d(pose_gz.pos.x, pose_gz.pos.y, pose_gz.pos.z,
                                pose_gz.rot.w, pose_gz.rot.x, pose_gz.rot.y, pose_gz.rot.z);
}
#endif

/**
 * @note Frames of reference:
 * g - gazebo (ENU), east, north, up
 * r - rotors imu frame (FLU), forward, left, up
 * b - px4 (FRD) forward, right down
 * n - px4 (NED) north, east, down
 */

/**
 * @brief Quaternion for rotation between ENU and NED frames
 *
 * NED to ENU: +PI/2 rotation about Z (Down) followed by a +PI rotation around X (old North/new East)
 * ENU to NED: +PI/2 rotation about Z (Up) followed by a +PI rotation about X (old East/new North)
 * This rotation is symmetric, so q_ENU_to_NED == q_NED_to_ENU.
 */
static const auto q_ENU_to_NED = ignition::math::Quaterniond(0, 0.70711, 0.70711, 0);

/**
 * @brief Quaternion for rotation between body FLU and body FRD frames
 *
 * +PI rotation around X (Forward) axis rotates from Forward, Right, Down (aircraft)
 * to Forward, Left, Up (base_link) frames and vice-versa.
 * This rotation is symmetric, so q_FLU_to_FRD == q_FRD_to_FLU.
 */
static const auto q_FLU_to_FRD = ignition::math::Quaterniond(0, 1, 0, 0);

// sensor X-axis unit vector in `base_link` frame
static const ignition::math::Vector3d kDownwardRotation = ignition::math::Vector3d(0, 0, -1);
static const ignition::math::Vector3d kUpwardRotation = ignition::math::Vector3d(0, 0, 1);
static const ignition::math::Vector3d kBackwardRotation = ignition::math::Vector3d(-1, 0, 0);
static const ignition::math::Vector3d kForwardRotation = ignition::math::Vector3d(1, 0, 0);
static const ignition::math::Vector3d kLeftRotation = ignition::math::Vector3d(0, 1, 0);
static const ignition::math::Vector3d kRightRotation = ignition::math::Vector3d(0, -1, 0);

// Zurich Irchel Park
static constexpr const double kDefaultHomeLatitude = 47.397742 * M_PI / 180.0;   // rad
static constexpr const double kDefaultHomeLongitude = 8.545594 * M_PI / 180.0;   // rad
static constexpr const double kDefaultHomeAltitude = 488.0;                      // meters

// Earth radius
static constexpr const double earth_radius = 6353000.0;      // meters

/**
 * @brief Get latitude and longitude coordinates from local position
 * @param[in] pos position in the local frame
 * @return std::pair of Latitude and Longitude
 */
inline std::pair<double, double> reproject(ignition::math::Vector3d& pos,
                                    double& lat_home,
                                    double& lon_home,
                                    double& alt_home)
{
  // reproject local position to gps coordinates
  const double x_rad = pos.Y() / earth_radius;    // north
  const double y_rad = pos.X() / earth_radius;    // east
  const double c = sqrt(x_rad * x_rad + y_rad * y_rad);
  const double sin_c = sin(c);
  const double cos_c = cos(c);

  double lat_rad, lon_rad;

  if (c != 0.0) {
    lat_rad = asin(cos_c * sin(lat_home) + (x_rad * sin_c * cos(lat_home)) / c);
    lon_rad = (lon_home + atan2(y_rad * sin_c, c * cos(lat_home) * cos_c - x_rad * sin(lat_home) * sin_c));
  } else {
    lat_rad = lat_home;
    lon_rad = lon_home;
  }

  return std::make_pair (lat_rad, lon_rad);
}

/**
 * @brief Check if the world spherical coordinates are set and set them
 * @param[in] world ptr to the world
 * @return true if they exist, false otherwise
 */
inline const bool checkWorldHomePosition(gazebo::physics::WorldPtr& world,
                                         double& world_latitude,
                                         double& world_longitude,
                                         double& world_altitude)
{
#if GAZEBO_MAJOR_VERSION >= 9
  gazebo::common::SphericalCoordinatesPtr spherical_coords = world->SphericalCoords();
#else
  gazebo::common::SphericalCoordinatesPtr spherical_coords = world->GetSphericalCoordinates();
#endif

  if (!spherical_coords) {
    return false;
  }
  world_latitude = spherical_coords->LatitudeReference().Radian();
  world_longitude = spherical_coords->LongitudeReference().Radian();
  world_altitude = spherical_coords->GetElevationReference();
  // This logic is required given that the spherical coordinates reference call
  // return 0 if the spherical coordnates are not defined in the world file
  return (world_latitude && world_latitude && world_latitude) ? true : false;
}

template <typename T>
inline T degrees(T radians)
{
    return radians * static_cast<T>(180.0) / static_cast<T>(M_PI);
}

template <typename T>
inline T radians(T degrees)
{
    return radians / static_cast<T>(180.0) * static_cast<T>(M_PI);
}


#endif /* ROTORS_GAZEBO_PLUGINS_COMMON_H_ */

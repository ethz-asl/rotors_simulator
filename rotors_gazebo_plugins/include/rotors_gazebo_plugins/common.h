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

}

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

#endif /* ROTORS_GAZEBO_PLUGINS_COMMON_H_ */

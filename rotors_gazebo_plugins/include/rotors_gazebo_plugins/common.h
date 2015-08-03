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

#ifndef ROTORS_GAZEBO_PLUGINS_COMMON_H_
#define ROTORS_GAZEBO_PLUGINS_COMMON_H_

#include <Eigen/Dense>
#include <gazebo/gazebo.hh>
#include <mav_msgs/default_topics.h>

namespace gazebo {

// Default values
static const std::string kDefaultNamespace = "";

static constexpr double kDefaultRotorVelocitySlowdownSim = 10.0;

static const std::string kDefaultGroundTruthPoseTopic =
    mav_msgs::default_topics::GROUND_TRUTH_POSE; // "ground_truth/pose";
static const std::string kDefaultGroundTruthTwistTopic =
    mav_msgs::default_topics::GROUND_TRUTH_TWIST;   // "ground_truth/twist";
static const std::string kDefaultImuTopic =
    mav_msgs::default_topics::IMU; // "imu"
static const std::string kDefaultCommandAttitudeThrustTopic =
    mav_msgs::default_topics::COMMAND_ATTITUDE_THRUST; // "command/attitude"
static const std::string kDefaultCommandMotorSpeedTopic =
    mav_msgs::default_topics::COMMAND_ACTUATORS;  // "command/motor_speed"
static const std::string kDefaultCommandRateThrustTopic =
    mav_msgs::default_topics::COMMAND_RATE_THRUST;  // "command/rate"
static const std::string kDefaultMotorTopic =
    mav_msgs::default_topics::MOTOR_MEASUREMENT;    // "motor_speed"
static const std::string kDefaultWrenchTopic =
    mav_msgs::default_topics::WRENCH;    // "wrench"
static const std::string kDefaultWindTopic =
    mav_msgs::default_topics::WIND;      // "wind";
static const std::string kDefaultTrajectoryTopic =
    mav_msgs::default_topics::COMMAND_TRAJECTORY;  // "command/trajectory"
static const std::string kDefaultPoseTopic =
    mav_msgs::default_topics::POSE;  // "pose"
static const std::string kDefaultPoseWithCovarianceTopic =
    mav_msgs::default_topics::POSE_WITH_COVARIANCE;  // "pose_with_covariance"
static const std::string kDefaultPositionTopic = "position";
static const std::string kDefaultTransformTopic =
    mav_msgs::default_topics::TRANSFORM;  // "transform"
static const std::string kDefaultOdometryTopic =
    mav_msgs::default_topics::ODOMETRY;  // "odometry"

/**
 * \brief Obtains a parameter from sdf.
 * \param[in] sdf Pointer to the sdf object.
 * \param[in] name Name of the parameter.
 * \param[out] param Param Variable to write the parameter to.
 * \param[in] default_value Default value, if the parameter not available.
 * \param[in] verbose If true, gzerror if the parameter is not available.
 */
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

}



template <typename T>
class FirstOrderFilter {
/*
This class can be used to apply a first order filter on a signal.
It allows different acceleration and deceleration time constants.

Short reveiw of discrete time implementation of firest order system:
Laplace:
    X(s)/U(s) = 1/(tau*s + 1)
continous time system:
    dx(t) = (-1/tau)*x(t) + (1/tau)*u(t)
discretized system (ZoH):
    x(k+1) = exp(samplingTime*(-1/tau))*x(k) + (1 - exp(samplingTime*(-1/tau))) * u(k)
*/

  public:
    FirstOrderFilter(double timeConstantUp, double timeConstantDown, T initialState):
      timeConstantUp_(timeConstantUp),
      timeConstantDown_(timeConstantDown),
      previousState_(initialState) {}

    T updateFilter(T inputState, double samplingTime) {
      /*
      This method will apply a first order filter on the inputState.
      */
      T outputState;
      if(inputState > previousState_){
        // Calcuate the outputState if accelerating.
        double alphaUp = exp(- samplingTime / timeConstantUp_);
        // x(k+1) = Ad*x(k) + Bd*u(k)
        outputState = alphaUp * previousState_ + (1 - alphaUp) * inputState;

      }else{
        // Calculate the outputState if decelerating.
        double alphaDown = exp(- samplingTime / timeConstantDown_);
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



/// Computes a quaternion from the 3-element small angle approximation theta.
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

template<typename T>
inline T limit(T value, T max, T min){
  if(value > max)
    return max;
  if(value < min)
    return min;

  return value;
}


#endif /* ROTORS_GAZEBO_PLUGINS_COMMON_H_ */

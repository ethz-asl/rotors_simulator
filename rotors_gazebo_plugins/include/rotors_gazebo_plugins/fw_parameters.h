/*
 * Copyright 2017 Pavel Vechersky, ASL, ETH Zurich, Switzerland
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

#ifndef ROTORS_GAZEBO_PLUGINS_FW_PARAMETERS_H_
#define ROTORS_GAZEBO_PLUGINS_FW_PARAMETERS_H_

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

namespace gazebo {

// Forward declaration.
struct ControlSurface;

// Default vehicle parameters (Techpod model)
static constexpr double kDefaultWingSpan = 2.59;
static constexpr double kDefaultWingSurface = 0.47;
static constexpr double kDefaultChordLength = 0.18;
static constexpr double kDefaultThrustInclination = 0.0;

// Default aerodynamic parameter values (Techpod model)
static constexpr double kDefaultAlphaMax = 0.27;
static constexpr double kDefaultAlphaMin = -0.27;

static const Eigen::Vector3d kDefaultCDragAlpha =
    Eigen::Vector3d(0.1360, -0.6737, 5.4546);
static const Eigen::Vector3d kDefaultCDragBeta =
    Eigen::Vector3d(0.0195, 0.0, -0.3842);
static const Eigen::Vector3d kDefaultCDragDeltaAil =
    Eigen::Vector3d(0.0195, 1.4205e-4, 7.5037e-6);
static const Eigen::Vector3d kDefaultCDragDeltaFlp =
    Eigen::Vector3d(0.0195, 2.7395e-4, 1.23e-5);

static const Eigen::Vector2d kDefaultCSideForceBeta =
    Eigen::Vector2d(0.0, -0.3073);

static const Eigen::Vector4d kDefaultCLiftAlpha =
    Eigen::Vector4d(0.2127, 10.8060, -46.8324, 60.6017);
static const Eigen::Vector2d kDefaultCLiftDeltaAil =
    Eigen::Vector2d(0.3304, 0.0048);
static const Eigen::Vector2d kDefaultCLiftDeltaFlp =
    Eigen::Vector2d(0.3304, 0.0073);

static const Eigen::Vector2d kDefaultCRollMomentBeta =
    Eigen::Vector2d(0.0, -0.0154);
static const Eigen::Vector2d kDefaultCRollMomentP =
    Eigen::Vector2d(0.0, -0.1647);
static const Eigen::Vector2d kDefaultCRollMomentR =
    Eigen::Vector2d(0.0, 0.0117);
static const Eigen::Vector2d kDefaultCRollMomentDeltaAil =
    Eigen::Vector2d(0.0, 0.0570);
static const Eigen::Vector2d kDefaultCRollMomentDeltaFlp =
    Eigen::Vector2d(0.0, 0.001);

static const Eigen::Vector2d kDefaultCPitchMomentAlpha =
    Eigen::Vector2d(0.0435, -2.9690);
static const Eigen::Vector2d kDefaultCPitchMomentQ =
    Eigen::Vector2d(-0.1173, -106.1541);
static const Eigen::Vector2d kDefaultCPitchMomentDeltaElv =
    Eigen::Vector2d(-0.1173, -6.1308);

static const Eigen::Vector2d kDefaultCYawMomentBeta =
    Eigen::Vector2d(0.0, 0.0430);
static const Eigen::Vector2d kDefaultCYawMomentR =
    Eigen::Vector2d(0.0, -0.0827);
static const Eigen::Vector2d kDefaultCYawMomentDeltaRud =
    Eigen::Vector2d(0.0, 0.06);

static const Eigen::Vector3d kDefaultCThrust =
    Eigen::Vector3d(0.0, 14.7217, 0.0);

// Default values for fixed-wing controls (Techpod model)
static constexpr double kDefaultControlSurfaceDeflectionMin =
    -20.0 * M_PI / 180.0;
static constexpr double kDefaultControlSurfaceDeflectionMax =
    20.0 * M_PI / 180.0;

static constexpr int kDefaultAileronLeftChannel = 4;
static constexpr int kDefaultAileronRightChannel = 0;
static constexpr int kDefaultElevatorChannel = 1;
static constexpr int kDefaultFlapChannel = 2;
static constexpr int kDefaultRudderChannel = 3;
static constexpr int kDefaultThrottleChannel = 5;

/// \brief  Wrapper function for extracting control surface parameters from a
///         YAML node.
inline void YAMLReadControlSurface(const YAML::Node& node,
                                   const std::string& name,
                                   ControlSurface& surface);

/// \brief  This function reads a vector from a YAML node and converts it into
///         a vector of type Eigen.
template <typename Derived>
inline void YAMLReadEigenVector(const YAML::Node& node,
                                const std::string& name,
                                Eigen::MatrixBase<Derived>& value);

/// \brief  This function reads a parameter from a YAML node.
template <typename T>
inline void YAMLReadParam(const YAML::Node& node,
                          const std::string& name,
                          T& value);

/// \brief  Macros to reduce copies of names.
#define READ_CONTROL_SURFACE(node, item) \
    YAMLReadControlSurface(node, #item, item);
#define READ_EIGEN_VECTOR(node, item) YAMLReadEigenVector(node, #item, item);
#define READ_PARAM(node, item) YAMLReadParam(node, #item, item);

struct FWAerodynamicParameters {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FWAerodynamicParameters()
      : alpha_max(kDefaultAlphaMax),
        alpha_min(kDefaultAlphaMin),
        c_drag_alpha(kDefaultCDragAlpha),
        c_drag_beta(kDefaultCDragBeta),
        c_drag_delta_ail(kDefaultCDragDeltaAil),
        c_drag_delta_flp(kDefaultCDragDeltaFlp),
        c_side_force_beta(kDefaultCSideForceBeta),
        c_lift_alpha(kDefaultCLiftAlpha),
        c_lift_delta_ail(kDefaultCLiftDeltaAil),
        c_lift_delta_flp(kDefaultCLiftDeltaFlp),
        c_roll_moment_beta(kDefaultCRollMomentBeta),
        c_roll_moment_p(kDefaultCRollMomentP),
        c_roll_moment_r(kDefaultCRollMomentR),
        c_roll_moment_delta_ail(kDefaultCRollMomentDeltaAil),
        c_roll_moment_delta_flp(kDefaultCRollMomentDeltaFlp),
        c_pitch_moment_alpha(kDefaultCPitchMomentAlpha),
        c_pitch_moment_q(kDefaultCPitchMomentQ),
        c_pitch_moment_delta_elv(kDefaultCPitchMomentDeltaElv),
        c_yaw_moment_beta(kDefaultCYawMomentBeta),
        c_yaw_moment_r(kDefaultCYawMomentR),
        c_yaw_moment_delta_rud(kDefaultCYawMomentDeltaRud),
        c_thrust(kDefaultCThrust) {}

  double alpha_max;
  double alpha_min;

  Eigen::Vector3d c_drag_alpha;
  Eigen::Vector3d c_drag_beta;
  Eigen::Vector3d c_drag_delta_ail;
  Eigen::Vector3d c_drag_delta_flp;

  Eigen::Vector2d c_side_force_beta;

  Eigen::Vector4d c_lift_alpha;
  Eigen::Vector2d c_lift_delta_ail;
  Eigen::Vector2d c_lift_delta_flp;

  Eigen::Vector2d c_roll_moment_beta;
  Eigen::Vector2d c_roll_moment_p;
  Eigen::Vector2d c_roll_moment_r;
  Eigen::Vector2d c_roll_moment_delta_ail;
  Eigen::Vector2d c_roll_moment_delta_flp;

  Eigen::Vector2d c_pitch_moment_alpha;
  Eigen::Vector2d c_pitch_moment_q;
  Eigen::Vector2d c_pitch_moment_delta_elv;

  Eigen::Vector2d c_yaw_moment_beta;
  Eigen::Vector2d c_yaw_moment_r;
  Eigen::Vector2d c_yaw_moment_delta_rud;

  Eigen::Vector3d c_thrust;

  void LoadAeroParamsYAML(const std::string& yaml_path) {
    const YAML::Node node = YAML::LoadFile(yaml_path);

    READ_PARAM(node, alpha_max);
    READ_PARAM(node, alpha_min);

    READ_EIGEN_VECTOR(node, c_drag_alpha);
    READ_EIGEN_VECTOR(node, c_drag_beta);
    READ_EIGEN_VECTOR(node, c_drag_delta_ail);
    READ_EIGEN_VECTOR(node, c_drag_delta_flp);

    READ_EIGEN_VECTOR(node, c_side_force_beta);

    READ_EIGEN_VECTOR(node, c_lift_alpha);
    READ_EIGEN_VECTOR(node, c_lift_delta_ail);
    READ_EIGEN_VECTOR(node, c_lift_delta_flp);

    READ_EIGEN_VECTOR(node, c_roll_moment_beta);
    READ_EIGEN_VECTOR(node, c_roll_moment_p);
    READ_EIGEN_VECTOR(node, c_roll_moment_r);
    READ_EIGEN_VECTOR(node, c_roll_moment_delta_ail);
    READ_EIGEN_VECTOR(node, c_roll_moment_delta_flp);

    READ_EIGEN_VECTOR(node, c_pitch_moment_alpha);
    READ_EIGEN_VECTOR(node, c_pitch_moment_q);
    READ_EIGEN_VECTOR(node, c_pitch_moment_delta_elv);

    READ_EIGEN_VECTOR(node, c_yaw_moment_beta);
    READ_EIGEN_VECTOR(node, c_yaw_moment_r);
    READ_EIGEN_VECTOR(node, c_yaw_moment_delta_rud);

    READ_EIGEN_VECTOR(node, c_thrust);
  }
};

struct ControlSurface {
  ControlSurface(int cs_channel,
                 double defl_min = kDefaultControlSurfaceDeflectionMin,
                 double defl_max = kDefaultControlSurfaceDeflectionMax)
      : channel(cs_channel),
        deflection_min(defl_min),
        deflection_max(defl_max) {}

  int channel;

  double deflection_min;
  double deflection_max;

  void LoadControlSurfaceNode(const YAML::Node& node) {
    READ_PARAM(node, channel);
    READ_PARAM(node, deflection_min);
    READ_PARAM(node, deflection_max);
  }
};

struct FWVehicleParameters {
  FWVehicleParameters()
      : wing_span(kDefaultWingSpan),
        wing_surface(kDefaultWingSurface),
        chord_length(kDefaultChordLength),
        thrust_inclination(kDefaultThrustInclination),
        throttle_channel(kDefaultThrottleChannel),
        aileron_left(kDefaultAileronLeftChannel),
        aileron_right(kDefaultAileronRightChannel),
        elevator(kDefaultElevatorChannel),
        flap(kDefaultFlapChannel),
        rudder(kDefaultRudderChannel) {}

  double wing_span;
  double wing_surface;
  double chord_length;
  double thrust_inclination;

  int throttle_channel;

  ControlSurface aileron_left;
  ControlSurface aileron_right;
  ControlSurface elevator;
  ControlSurface flap;
  ControlSurface rudder;

  void LoadVehicleParamsYAML(const std::string& yaml_path) {
    const YAML::Node node = YAML::LoadFile(yaml_path);

    READ_PARAM(node, wing_span);
    READ_PARAM(node, wing_surface);
    READ_PARAM(node, chord_length);
    READ_PARAM(node, thrust_inclination);

    READ_PARAM(node, throttle_channel);

    READ_CONTROL_SURFACE(node, aileron_left);
    READ_CONTROL_SURFACE(node, aileron_right);
    READ_CONTROL_SURFACE(node, elevator);
    READ_CONTROL_SURFACE(node, flap);
    READ_CONTROL_SURFACE(node, rudder);
  }
};

inline void YAMLReadControlSurface(const YAML::Node& node,
                                   const std::string& name,
                                   ControlSurface& surface) {
  const YAML::Node surface_node = node[name];
  surface.LoadControlSurfaceNode(surface_node);
}

template <typename Derived>
inline void YAMLReadEigenVector(const YAML::Node& node,
                                const std::string& name,
                                Eigen::MatrixBase<Derived>& value) {
  std::vector<typename Derived::RealScalar> vec =
      node[name].as<std::vector<typename Derived::RealScalar>>();
  assert(vec.size() == Derived::SizeAtCompileTime);
  value = Eigen::Map<Derived>(&vec[0], vec.size());
}

template <typename T>
inline void YAMLReadParam(const YAML::Node& node,
                          const std::string& name,
                          T& value) {
  value = node[name].as<T>();
}

}

#endif /* ROTORS_GAZEBO_PLUGINS_FW_PARAMETERS_H_ */

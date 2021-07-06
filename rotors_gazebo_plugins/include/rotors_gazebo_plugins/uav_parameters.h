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

#ifndef ROTORS_GAZEBO_PLUGINS_UAV_PARAMETERS_H_
#define ROTORS_GAZEBO_PLUGINS_UAV_PARAMETERS_H_

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <exception>

namespace gazebo {

/*
// Forward declaration.
struct ControlSurface;

/// \brief  Wrapper function for extracting control surface parameters from a
///         YAML node.
inline void YAMLReadAirfoil(const YAML::Node& node,
                                   const std::string& name,
                                   ControlSurface& surface);
*/

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Default airfoil parameter values (NACAXXXX airfoil and low-Re flat-plate)
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Non-stalled regime
static constexpr double kDefaultAlphaMaxNs = 0.27;
static constexpr double kDefaultAlphaMinNs = -0.27;

static const Eigen::Vector3d kDefaultCLiftAlpha =
    Eigen::Vector3d(0.4, 6.3, 0.0);

static const Eigen::Vector3d kDefaultCDragAlpha =
    Eigen::Vector3d(0.0, 0.0, 0.9);

static const Eigen::Vector2d kDefaultCPitchMomentAlpha =
    Eigen::Vector2d(-0.1, 0.0);

// Stalled regime
static constexpr double kDefaultAlphaBlend = 0.2;
static constexpr double kDefaultFpCLiftMax = 0.65;
static constexpr double kDefaultFpCDragMax = 1.20;
static constexpr double kDefaultFpCPitchMomentMax = 0.40;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Default propeller parameter values (similar 11x7E apc)
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
static constexpr double kDefaultDiameter = 0.28;
static constexpr double kDefaultMass = 0.023;
static constexpr double kDefaultKT = -0.13;
static constexpr double kDefaultKT0 = 0.11;
static constexpr double kDefaultKQ = -0.011;
static constexpr double kDefaultKQ0 = 0.01;
static constexpr double kDefaultRollMomCoeff = 1e-06;
static constexpr double kDefaultDragMomCoeff = 5.3849e-04;
static constexpr double kDefaultDFlow = 5.0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Default values for use with ADIS16448 IMU
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
static constexpr double kDefaultAdisGyroscopeNoiseDensity = 2.0 * 35.0 / 3600.0 / 180.0 * M_PI;
static constexpr double kDefaultAdisGyroscopeRandomWalk = 2.0 * 4.0 / 3600.0 / 180.0 * M_PI;
static constexpr double kDefaultAdisGyroscopeBiasCorrelationTime = 1.0e+3;
static constexpr double kDefaultAdisGyroscopeTurnOnBiasSigma = 0.5 / 180.0 * M_PI;
static constexpr double kDefaultAdisAccelerometerNoiseDensity = 2.0 * 2.0e-3;
static constexpr double kDefaultAdisAccelerometerRandomWalk = 2.0 * 3.0e-3;
static constexpr double kDefaultAdisAccelerometerBiasCorrelationTime = 300.0;
static constexpr double kDefaultAdisAccelerometerTurnOnBiasSigma = 20.0e-3 * 9.8;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Earth's gravity in Zurich (lat=+47.3667degN, lon=+8.5500degE, h=+500m, WGS84)
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
static constexpr double kDefaultGravityMagnitude = 9.8068;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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
//#define READ_CONTROL_SURFACE(node, item) YAMLReadControlSurface(node, #item, item);
#define READ_EIGEN_VECTOR(node, item) YAMLReadEigenVector(node, #item, item);
#define READ_PARAM(node, item) YAMLReadParam(node, #item, item);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct PropellerParameters {

    PropellerParameters():
        diameter(kDefaultDiameter),
        mass(kDefaultMass),
        k_t(kDefaultKT),
        k_t0(kDefaultKT0),
        k_q(kDefaultKQ),
        k_q0(kDefaultKQ0),
        rolling_moment_coefficient(kDefaultRollMomCoeff),
        rotor_drag_coefficient(kDefaultDragMomCoeff),
        d_flow(kDefaultDFlow){}

    double diameter;
    double mass;
    double k_t;
    double k_t0;
    double k_q;
    double k_q0;
    double rolling_moment_coefficient;
    double rotor_drag_coefficient;
    double d_flow;

    void LoadPropParamsYAML(const std::string& yaml_path) {

        gzdbg <<"loading propeller: "<< yaml_path <<std::endl;

        try{

            const YAML::Node node = YAML::LoadFile(yaml_path);

            try{

                READ_PARAM(node, diameter);
                READ_PARAM(node, mass);
                READ_PARAM(node, k_t);
                READ_PARAM(node, k_t0);
                READ_PARAM(node, k_q);
                READ_PARAM(node, k_q0);
                READ_PARAM(node, rolling_moment_coefficient);
                READ_PARAM(node, rotor_drag_coefficient);
                READ_PARAM(node, d_flow);

            } catch(const YAML::Exception& ex) {
                gzerr << ex.what();
            } catch (const std::exception& ex) {
                gzerr<<ex.what()<<std::endl;
            } catch (const std::string& ex) {
                gzerr<<ex<<std::endl;
            } catch (...) {
                gzerr<<"meeep"<<std::endl;
            }

        } catch(const YAML::Exception& ex) {
            gzerr << ex.what();
        } catch (const std::exception& ex) {
            gzerr<<ex.what()<<std::endl;
        } catch (const std::string& ex) {
            gzerr<<ex<<std::endl;
        } catch (...) {
            gzerr<<"meeep"<<std::endl;
        }
    }
};

struct AerodynamicParameters {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    AerodynamicParameters():
          alpha_max_ns(kDefaultAlphaMaxNs),
          alpha_min_ns(kDefaultAlphaMinNs),
          c_lift_alpha(kDefaultCLiftAlpha),
          c_drag_alpha(kDefaultCDragAlpha),
          c_pitch_moment_alpha(kDefaultCPitchMomentAlpha),
          alpha_blend(kDefaultAlphaBlend),
          fp_c_lift_max(kDefaultFpCLiftMax),
          fp_c_drag_max(kDefaultFpCDragMax),
          fp_c_pitch_moment_max(kDefaultFpCPitchMomentMax) {

          //gzdbg<<"aerodynamic struct created... \n";
    }

    double alpha_max_ns;
    double alpha_min_ns;

    Eigen::Vector3d c_lift_alpha;
    Eigen::Vector3d c_drag_alpha;
    Eigen::Vector2d c_pitch_moment_alpha;

    double alpha_blend;
    double fp_c_lift_max;
    double fp_c_drag_max;
    double fp_c_pitch_moment_max;

    void LoadAeroParamsYAML(const std::string& yaml_path) {

        gzdbg <<"loading airfoil: "<< yaml_path <<std::endl;

        try{

            const YAML::Node node = YAML::LoadFile(yaml_path);

            try{

                READ_PARAM(node, alpha_max_ns);
                READ_PARAM(node, alpha_min_ns);

                READ_EIGEN_VECTOR(node, c_lift_alpha);
                READ_EIGEN_VECTOR(node, c_drag_alpha);
                READ_EIGEN_VECTOR(node, c_pitch_moment_alpha);

                READ_PARAM(node, alpha_blend);
                READ_PARAM(node, fp_c_lift_max);
                READ_PARAM(node, fp_c_drag_max);
                READ_PARAM(node, fp_c_pitch_moment_max);

            } catch(const YAML::Exception& ex) {
                gzerr << ex.what();
            } catch (const std::exception& ex) {
                gzerr<<ex.what()<<std::endl;
            } catch (const std::string& ex) {
                gzerr<<ex<<std::endl;
            } catch (...) {
                gzerr<<"meeep"<<std::endl;
            }

        } catch(const YAML::Exception& ex) {
            gzerr << ex.what();
        } catch (const std::exception& ex) {
            gzerr<<ex.what()<<std::endl;
        } catch (const std::string& ex) {
            gzerr<<ex<<std::endl;
        } catch (...) {
            gzerr<<"meeep"<<std::endl;
        }
    }
};

/*
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
*/
/*
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

    try{
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

    } catch (const std::exception& ex) {
        gzerr<<ex.what()<<std::endl;
    } catch (const std::string& ex) {
        gzerr<<ex<<std::endl;
    } catch (...) {
        gzerr<<"meeep"<<std::endl;
    }
  }
};
*/
/*
inline void YAMLReadControlSurface(const YAML::Node& node,
                                   const std::string& name,
                                   ControlSurface& surface) {
  const YAML::Node surface_node = node[name];
  surface.LoadControlSurfaceNode(surface_node);
}
*/

// A description of the parameters:
// https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model-and-Intrinsics
// TODO(burrimi): Should I have a minimalistic description of the params here?
struct ImuParameters {
  /// Gyroscope noise density (two-sided spectrum) [rad/s/sqrt(Hz)]
  double gyroscope_noise_density;
  /// Gyroscope bias random walk [rad/s/s/sqrt(Hz)]
  double gyroscope_random_walk;
  /// Gyroscope bias correlation time constant [s]
  double gyroscope_bias_correlation_time;
  /// Gyroscope turn on bias standard deviation [rad/s]
  double gyroscope_turn_on_bias_sigma;
  /// Accelerometer noise density (two-sided spectrum) [m/s^2/sqrt(Hz)]
  double accelerometer_noise_density;
  /// Accelerometer bias random walk. [m/s^2/s/sqrt(Hz)]
  double accelerometer_random_walk;
  /// Accelerometer bias correlation time constant [s]
  double accelerometer_bias_correlation_time;
  /// Accelerometer turn on bias standard deviation [m/s^2]
  double accelerometer_turn_on_bias_sigma;
  /// Norm of the gravitational acceleration [m/s^2]
  double gravity_magnitude;

  ImuParameters()
      : gyroscope_noise_density(kDefaultAdisGyroscopeNoiseDensity),
        gyroscope_random_walk(kDefaultAdisGyroscopeRandomWalk),
        gyroscope_bias_correlation_time(
            kDefaultAdisGyroscopeBiasCorrelationTime),
        gyroscope_turn_on_bias_sigma(kDefaultAdisGyroscopeTurnOnBiasSigma),
        accelerometer_noise_density(kDefaultAdisAccelerometerNoiseDensity),
        accelerometer_random_walk(kDefaultAdisAccelerometerRandomWalk),
        accelerometer_bias_correlation_time(
            kDefaultAdisAccelerometerBiasCorrelationTime),
        accelerometer_turn_on_bias_sigma(
            kDefaultAdisAccelerometerTurnOnBiasSigma),
        gravity_magnitude(kDefaultGravityMagnitude) {}
};

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

template<typename ValueType>
bool SafeGet(const YAML::Node& node, const std::string& key, ValueType* value) {
  //CHECK_NOTNULL(value);
  bool success = false;
  if(!node.IsMap()) {
    gzerr << "Unable to get Node[\"" << key << "\"] because the node is not a map";
  } else {
    const YAML::Node sub_node = node[key];
    if(sub_node) {
      try {
        *value = sub_node.as<ValueType>();
        success = true;
      } catch(const YAML::Exception& e) {
        gzerr << "Error getting key \"" << key << "\" as type "
            << typeid(ValueType).name() << ": " << e.what();
      }
    } else {
      gzerr << "Key \"" << key << "\" does not exist";
    }
  }
  return success;
}

#endif /* ROTORS_GAZEBO_PLUGINS_UAV_PARAMETERS_H_ */

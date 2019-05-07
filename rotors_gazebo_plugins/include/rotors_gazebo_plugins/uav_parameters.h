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

//++++++++++++++++++++++++++++++++++++++++++
// Default airfoil parameter values (NACAXXXX airfoil and low-Re flat-plate)
//++++++++++++++++++++++++++++++++++++++++++
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

//++++++++++++++++++++++++++++++++++++++++++
// Default propeller parameter values (similar 11x7E apc)
//++++++++++++++++++++++++++++++++++++++++++
static constexpr double kDefaultDiameter = 0.28;
static constexpr double kDefaultKT = -0.13;
static constexpr double kDefaultKT0 = 0.11;
static constexpr double kDefaultKQ = -0.011;
static constexpr double kDefaultKQ0 = 0.01;
static constexpr double kDefaultRollMomCoeff = 1e-06;
static constexpr double kDefaultDragMomCoeff = 5.3849e-04;
static constexpr double kDefaultDFlow = 5.0;

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

struct PropellerParameters {

    PropellerParameters():
        diameter(kDefaultDiameter),
        k_T(kDefaultKT),
        k_T0(kDefaultKT0),
        k_Q(kDefaultKQ),
        k_Q0(kDefaultKQ0),
        rolling_moment_coefficient_(kDefaultRollMomCoeff),
        rotor_drag_coefficient_(kDefaultDragMomCoeff),
        d_flow(kDefaultDFlow){}

    double diameter;
    double k_T;
    double k_T0;
    double k_Q;
    double k_Q0;
    double rolling_moment_coefficient_;
    double rotor_drag_coefficient_;
    double d_flow;

    void LoadPropParamsYAML(const std::string& yaml_path) {

        gzdbg <<"loading propeller"<< yaml_path <<std::endl;

        try{

            const YAML::Node node = YAML::LoadFile(yaml_path);

            gzdbg<<"IsDefined"<<node.IsDefined()<<std::endl;
            gzdbg<<"IsMap"<<node.IsMap()<<std::endl;
            gzdbg<<"IsNull"<<node.IsNull()<<std::endl;
            gzdbg<<"IsScalar"<<node.IsScalar()<<std::endl;
            gzdbg<<"IsSequence"<<node.IsSequence()<<std::endl;

            try{

                READ_PARAM(node, diameter);
                READ_PARAM(node, k_T);
                READ_PARAM(node, k_T0);
                READ_PARAM(node, k_Q);
                READ_PARAM(node, k_Q0);
                READ_PARAM(node, rolling_moment_coefficient_);
                READ_PARAM(node, rotor_drag_coefficient_);
                READ_PARAM(node, d_flow);

            } catch (const std::exception& ex) {
                gzerr<<ex.what()<<std::endl;
            } catch (const std::string& ex) {
                gzerr<<ex<<std::endl;
            } catch (...) {
                gzerr<<"meeep"<<std::endl;
            }

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

        gzdbg <<"loading airfoil"<< yaml_path <<std::endl;

        try{

            const YAML::Node node = YAML::LoadFile(yaml_path);

            gzdbg<<"IsDefined"<<node.IsDefined()<<std::endl;
            gzdbg<<"IsMap"<<node.IsMap()<<std::endl;
            gzdbg<<"IsNull"<<node.IsNull()<<std::endl;
            gzdbg<<"IsScalar"<<node.IsScalar()<<std::endl;
            gzdbg<<"IsSequence"<<node.IsSequence()<<std::endl;

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

            } catch (const std::exception& ex) {
                gzerr<<ex.what()<<std::endl;
            } catch (const std::string& ex) {
                gzerr<<ex<<std::endl;
            } catch (...) {
                gzerr<<"meeep"<<std::endl;
            }

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

    gzdbg << yaml_path <<std::endl;
    gzdbg<<"IsDefined"<<node.IsDefined()<<std::endl;
    gzdbg<<"IsMap"<<node.IsMap()<<std::endl;
    gzdbg<<"IsNull"<<node.IsNull()<<std::endl;
    gzdbg<<"IsScalar"<<node.IsScalar()<<std::endl;
    gzdbg<<"IsSequence"<<node.IsSequence()<<std::endl;

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

#endif /* ROTORS_GAZEBO_PLUGINS_UAV_PARAMETERS_H_ */

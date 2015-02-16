#ifndef INCLUDE_ROTORS_CONTROL_PARAMETERS_H_
#define INCLUDE_ROTORS_CONTROL_PARAMETERS_H_

namespace rotors_control {
// Default values for the Asctec Firefly rotor configuration.
static constexpr double kDefaultRotor0Angle = 0.52359877559;
static constexpr double kDefaultRotor1Angle = 1.57079632679;
static constexpr double kDefaultRotor2Angle = 2.61799387799;
static constexpr double kDefaultRotor3Angle = -2.61799387799;
static constexpr double kDefaultRotor4Angle = -1.57079632679;
static constexpr double kDefaultRotor5Angle = -0.52359877559;

// Default vehicle parameters for Asctec Firefly.
static constexpr double kDefaultMass = 1.56779;
static constexpr double kDefaultArmLength = 0.215;
static constexpr double kDefaultInertiaXx = 0.0347563;
static constexpr double kDefaultInertiaYy = 0.0458929;
static constexpr double kDefaultInertiaZz = 0.0977;
static constexpr double kDefaultRotorForceConstant = 8.54858e-6;
static constexpr double kDefaultRotorMomentConstant = 1.6e-2;

// Default physics parameters.
static constexpr double kDefaultGravity = 9.81;

struct Rotor {
  Rotor()
      : angle(0.0),
        arm_length(kDefaultArmLength),
        rotor_force_constant(kDefaultRotorForceConstant),
        rotor_moment_constant(kDefaultRotorMomentConstant),
        direction(1) {}
  Rotor(double _angle, double _arm_length,
        double _rotor_force_constant, double _rotor_moment_constant,
        int _direction)
      : angle(_angle),
        arm_length(_arm_length),
        rotor_force_constant(_rotor_force_constant),
        rotor_moment_constant(_rotor_moment_constant),
        direction(_direction) {}
  double angle;
  double arm_length;
  double rotor_force_constant;
  double rotor_moment_constant;
  int direction;
};

struct RotorConfiguration {
  RotorConfiguration() {
    // Rotor configuration of Asctec Firefly.
    rotors.push_back(
      Rotor(kDefaultRotor0Angle, kDefaultArmLength, kDefaultRotorForceConstant,
            kDefaultRotorMomentConstant, 1));
    rotors.push_back(
      Rotor(kDefaultRotor1Angle, kDefaultArmLength, kDefaultRotorForceConstant,
            kDefaultRotorMomentConstant, -1));
    rotors.push_back(
      Rotor(kDefaultRotor2Angle, kDefaultArmLength, kDefaultRotorForceConstant,
            kDefaultRotorMomentConstant, 1));
    rotors.push_back(
      Rotor(kDefaultRotor3Angle, kDefaultArmLength, kDefaultRotorForceConstant,
            kDefaultRotorMomentConstant, -1));
    rotors.push_back(
      Rotor(kDefaultRotor4Angle, kDefaultArmLength, kDefaultRotorForceConstant,
            kDefaultRotorMomentConstant, 1));
    rotors.push_back(
      Rotor(kDefaultRotor5Angle, kDefaultArmLength, kDefaultRotorForceConstant,
            kDefaultRotorMomentConstant, -1));
  }
  std::vector<Rotor> rotors;
};

class VehicleParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VehicleParameters()
      : mass_(kDefaultMass),
        gravity_(kDefaultGravity),
        inertia_(Eigen::Vector3d(kDefaultInertiaXx, kDefaultInertiaYy,
                                 kDefaultInertiaZz).asDiagonal()) {}
  double mass_;
  const double gravity_;
  Eigen::Matrix3d inertia_;
  RotorConfiguration rotor_configuration_;
};

}

#endif /* INCLUDE_ROTORS_CONTROL_PARAMETERS_H_ */

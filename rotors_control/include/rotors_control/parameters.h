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
  Rotor () : angle(0.0), direction(1) {}
  double angle;
  // double arm_length;
  int direction;
};

struct RotorConfiguration {
  RotorConfiguration() {
    // Rotor configuration of Asctec Firefly.
    Rotor rotor0, rotor1, rotor2, rotor3, rotor4, rotor5;
    rotor0.angle = kDefaultRotor0Angle;
    // rotor0.arm_length = 0.215;
    rotor0.direction = 1;
    rotors.push_back(rotor0);
    rotor1.angle = kDefaultRotor1Angle;
    // rotor0.arm_length = 0.215;
    rotor1.direction = -1;
    rotors.push_back(rotor1);
    rotor2.angle = kDefaultRotor2Angle;
    // rotor0.arm_length = 0.215;
    rotor2.direction = 1;
    rotors.push_back(rotor2);
    rotor3.angle = kDefaultRotor3Angle;
    // rotor0.arm_length = 0.215;
    rotor3.direction = -1;
    rotors.push_back(rotor3);
    rotor4.angle = kDefaultRotor4Angle;
    // rotor0.arm_length = 0.215;
    rotor4.direction = 1;
    rotors.push_back(rotor4);
    rotor5.angle = kDefaultRotor5Angle;
    // rotor0.arm_length = 0.215;
    rotor5.direction = -1;
    rotors.push_back(rotor5);
  }
  std::vector<Rotor> rotors;
};

class VehicleParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VehicleParameters()
      : mass_(kDefaultMass),
        arm_length_(kDefaultArmLength),
        rotor_force_constant_(kDefaultRotorForceConstant),
        rotor_moment_constant_(kDefaultRotorMomentConstant),
        inertia_(Eigen::Vector3d(kDefaultInertiaXx, kDefaultInertiaYy,
                                 kDefaultInertiaZz).asDiagonal()) {}
  bool parameters_initialized_;
  double mass_;
  double arm_length_;
  double rotor_force_constant_;
  double rotor_moment_constant_;
  Eigen::Matrix3d inertia_;
  RotorConfiguration rotor_configuration_;
};


class PhysicsParameters {
 public:
  PhysicsParameters() : gravity_(kDefaultGravity) {};
  const double gravity_;
};

}

#endif /* INCLUDE_ROTORS_CONTROL_PARAMETERS_H_ */

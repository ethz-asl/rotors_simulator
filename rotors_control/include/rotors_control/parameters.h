#ifndef INCLUDE_ROTORS_CONTROL_PARAMETERS_H_
#define INCLUDE_ROTORS_CONTROL_PARAMETERS_H_
namespace rotors_control {

struct Rotor {
  double angle;
  double arm_length;
  int direction;
};

struct RotorConfiguration {
  std::vector<Rotor> rotors;
};

class VehicleParameters {
 public:
  bool parameters_initialized_;
  Eigen::Matrix3d inertia_;
  double mass_;
  double arm_length_;
  double rotor_force_constant_;
  double rotor_moment_constant_;
  RotorConfiguration rotor_configuration_;

  VehicleParameters()
      : mass_(1.0),
        arm_length_(1.0),
        rotor_force_constant_(8.0e-6),
        rotor_moment_constant_(1.6e-2) {}
 private:
  void SetParameters(VehicleParameters vehicle_parameters);
  void GetParameters();
};

class PhysicsParameters {
 public:
  const double gravity_;
  PhysicsParameters() : gravity_(9.81) {};
 private:
  void SetParameters(PhysicsParameters physics_parameters);
  void GetParameters();
};

}

#endif /* INCLUDE_ROTORS_CONTROL_PARAMETERS_H_ */

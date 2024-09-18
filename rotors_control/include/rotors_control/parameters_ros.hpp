#ifndef INCLUDE_ROTORS_CONTROL_PARAMETERS_ROS_HPP_
#define INCLUDE_ROTORS_CONTROL_PARAMETERS_ROS_HPP_


#include <rclcpp/rclcpp.hpp>
#include <rcpputils/asserts.hpp>
#include "rotors_control/parameters.hpp"

namespace rotors_control {

template<typename T> inline void GetRosParameter(const std::shared_ptr<rclcpp::Node>& node,
                                                 const std::string& key,
                                                 const T& default_value,
                                                 T* value) {
  rcpputils::assert_true(value != nullptr);
  node->declare_parameter(key,*value);
  bool have_parameter = node->get_parameter(key, *value);
  if (!have_parameter) {
    RCLCPP_WARN_STREAM(node->get_logger(), "[rosparam]: could not find parameter "
                        << node->get_name() << "/" << key
                        << ", setting to default: " << default_value);
    *value = default_value;
  }
}

// inline void GetRotorConfiguration(const std::shared_ptr<rclcpp::Node>& node,
//                                   RotorConfiguration* rotor_configuration) {
//   std::map<std::string, double> single_rotor;
//   std::string rotor_configuration_string = "rotor_configuration/";
//   unsigned int i = 0;
//   while (node->get_parameter(rotor_configuration_string + std::to_string(i), single_rotor)) {
//     if (i == 0) {
//       rotor_configuration->rotors.clear();
//     }
//     Rotor rotor;
//     node->get_parameter(rotor_configuration_string + std::to_string(i) + "/angle",
//                  rotor.angle);
//     node->get_parameter(rotor_configuration_string + std::to_string(i) + "/arm_length",
//                  rotor.arm_length);
//     node->get_parameter(rotor_configuration_string + std::to_string(i) + "/rotor_force_constant",
//                  rotor.rotor_force_constant);
//     node->get_parameter(rotor_configuration_string + std::to_string(i) + "/rotor_moment_constant",
//                  rotor.rotor_moment_constant);
//     node->get_parameter(rotor_configuration_string + std::to_string(i) + "/direction",
//                  rotor.direction);
//     rotor_configuration->rotors.push_back(rotor);
//     ++i;
//   }
// }

//check the below implementation
  inline void GetRotorConfiguration(const rclcpp::Node::SharedPtr& node,
                                  RotorConfiguration* rotor_configuration) {
  std::string rotor_configuration_string = "rotor_configuration/";
  unsigned int i = 0;

  while (true) {
    std::string param_name = rotor_configuration_string + std::to_string(i) + "/angle";
    if (!node->has_parameter(param_name)) {
      break;
    }

    Rotor rotor;
    node->get_parameter(param_name, rotor.angle);
    node->get_parameter(rotor_configuration_string + std::to_string(i) + "/arm_length", rotor.arm_length);
    node->get_parameter(rotor_configuration_string + std::to_string(i) + "/rotor_force_constant", rotor.rotor_force_constant);
    node->get_parameter(rotor_configuration_string + std::to_string(i) + "/rotor_moment_constant", rotor.rotor_moment_constant);
    node->get_parameter(rotor_configuration_string + std::to_string(i) + "/direction", rotor.direction);
    
    rotor_configuration->rotors.push_back(rotor);
    ++i;
  }
}


inline void GetVehicleParameters(const rclcpp::Node::SharedPtr& node, VehicleParameters* vehicle_parameters) {
  GetRosParameter(node, "mass",
                  vehicle_parameters->mass_,
                  &vehicle_parameters->mass_);
  GetRosParameter(node, "inertia/xx",
                  vehicle_parameters->inertia_(0, 0),
                  &vehicle_parameters->inertia_(0, 0));
  GetRosParameter(node, "inertia/xy",
                  vehicle_parameters->inertia_(0, 1),
                  &vehicle_parameters->inertia_(0, 1));
  vehicle_parameters->inertia_(1, 0) = vehicle_parameters->inertia_(0, 1);
  GetRosParameter(node, "inertia/xz",
                  vehicle_parameters->inertia_(0, 2),
                  &vehicle_parameters->inertia_(0, 2));
  vehicle_parameters->inertia_(2, 0) = vehicle_parameters->inertia_(0, 2);
  GetRosParameter(node, "inertia/yy",
                  vehicle_parameters->inertia_(1, 1),
                  &vehicle_parameters->inertia_(1, 1));
  GetRosParameter(node, "inertia/yz",
                  vehicle_parameters->inertia_(1, 2),
                  &vehicle_parameters->inertia_(1, 2));
  vehicle_parameters->inertia_(2, 1) = vehicle_parameters->inertia_(1, 2);
  GetRosParameter(node, "inertia/zz",
                  vehicle_parameters->inertia_(2, 2),
                  &vehicle_parameters->inertia_(2, 2));
  GetRotorConfiguration(node, &vehicle_parameters->rotor_configuration_);
}
}

#endif /* INCLUDE_ROTORS_CONTROL_PARAMETERS_ROS_HPP_ */

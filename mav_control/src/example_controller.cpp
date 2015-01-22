/*
 * Copyright (c) 2015, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <mav_control/example_controller.h>

namespace mav_control {


// ---- ros independent ----
ExampleController::ExampleController() {

}

ExampleController::~ExampleController() {
}

void ExampleController::SetReferenceRateThrust(const Eigen::Vector4d& control_rate_thrust_reference){
  control_attitude_thrust_reference_ = control_rate_thrust_reference;
}

void ExampleController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities){
  ;
}

const char* ExampleController::GetName() const{
  return "my_awesome_example_controller";
}

// ---- end ros independent ----

void ExampleControllerPlugin::LoadImpl(const VehicleParameters& vehicle_parameters, const std::string& parameter_namespace){
  controller_->setVehicleParameters(vehicle_parameters);

  // load vehicle specific parameters
  ros::NodeHandle nh(parameter_namespace);
  ExampleControllerParameters controller_parameters;

  const double default_kp = 0.0;
  GetParameter(nh, "kp", default_kp, &controller_parameters.kp_);

  // create dynamic reconfigure or similar to set parameters at runtime
}

std::shared_ptr<ControllerBase> ExampleControllerPlugin::GetControllerImpl(){
  return controller_;
}

} // namespace mav_control

MAV_CONTROL_REGISTER_CONTROLLER(mav_control, ExampleControllerPlugin);

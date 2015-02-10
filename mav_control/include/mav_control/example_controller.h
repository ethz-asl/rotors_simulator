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

#ifndef EXAMPLE_CONTROLLER_H_
#define EXAMPLE_CONTROLLER_H_

#include <mav_control/controller_base.h>

namespace mav_control {

// ---- Controller declaration/implementation, this is the ros independent part ----
class ExampleControllerParameters{
 public:
  double kp_;
};

class ExampleController : public ControllerBase{
 public:
  ExampleController();
  virtual ~ExampleController();

  void setVehicleParameters(const VehicleParameters& vehicle_parameters){
    vehicle_parameters_ = vehicle_parameters;
  }

  void setControllerParameters(const ExampleControllerParameters& controller_parameters) {
    controller_parameters_ = controller_parameters;
  }

  virtual void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities);
  virtual const char* GetName() const;
  virtual void SetReferenceRateThrust(const Eigen::Vector4d& control_rate_thrust_reference);

 protected:
  VehicleParameters vehicle_parameters_;
  ExampleControllerParameters controller_parameters_;
};

} // namespace mav_control
// ---- end ros independent part ----

// ---- now declare/implement the ros-dependent plugin ----
#include <mav_control/controller_plugin_base.h>

namespace mav_control {

class ExampleControllerPlugin : public ControllerPluginBase{
 public:
  virtual ~ExampleControllerPlugin();

 protected:
  virtual void LoadImpl(const VehicleParameters& vehicle_parameters, const std::string& parameter_namespace);
  virtual std::shared_ptr<ControllerBase> GetControllerImpl();

 private:
  std::shared_ptr<ExampleController> controller_;
};

} // namespace mav_control

#endif

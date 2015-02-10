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

#include <mav_control/controller_plugin_base.h>

namespace mav_control{

void ControllerPluginBase::Load(const std::string& vehicle_parameter_namespace,
                                const std::string& controller_parameter_namespace) {

  VehicleParameters vehicle_parameters;
  ros::NodeHandle nh(vehicle_parameter_namespace);
  GetParameter(nh, "mass", VehicleParameters::GetDefaultMass(), &vehicle_parameters.mass_);
  // continue with the remaining parameters ...

  LoadImpl(vehicle_parameters, controller_parameter_namespace);
  controller_ = GetControllerImpl();
}

}  // end namespace mav_control

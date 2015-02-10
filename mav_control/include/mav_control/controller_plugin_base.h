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

#ifndef CONTROLLER_PLUGIN_BASE_H_
#define CONTROLLER_PLUGIN_BASE_H_

#include <ros/ros.h>
#include <mav_control/controller_base.h>

namespace mav_control {

class ControllerPluginBase {
 public:
  ControllerPluginBase();
  virtual ~ControllerPluginBase();

  void Load(const std::string& vehicle_parameter_namespace, const std::string& controller_parameter_namespace);

  std::shared_ptr<ControllerBase> GetController() {
    return controller_;
  }

  std::shared_ptr<const ControllerBase> GetController() const {
    return controller_;
  }

 protected:
  template<typename T>
  void GetParameter(const ros::NodeHandle& nh, const std::string& key, const T& default_value, T* value);

  virtual void LoadImpl(const VehicleParameters& vehicle_parameters, const std::string& parameter_namespace) = 0;
  virtual std::shared_ptr<ControllerBase> GetControllerImpl() = 0;

 private:

  std::shared_ptr<ControllerBase> controller_;
};

template<typename T> void ControllerPluginBase::GetParameter(const ros::NodeHandle& nh, const std::string& key,
                                                             const T& default_value, T* value) {
  bool have_parameter = nh.getParam(key, *value);
  if (!have_parameter) {
    ROS_WARN_STREAM("[ControllerPlugin]: could not find parameter " << nh.getNamespace() << "/" << key
                    << "setting to default: " << default_value);
    *value = default_value;
  }
}

}  // end namespace mav_control

#endif /* CONTROLLER_PLUGIN_BASE_H_ */

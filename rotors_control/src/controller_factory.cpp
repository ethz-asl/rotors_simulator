/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#include "rotors_control/controller_factory.h"

#include <iostream>

namespace rotors_controller_factory {
std::shared_ptr<ControllerBase> ControllerFactory::CreateController(const std::string& controller_name) {

  ControllerMap::const_iterator it = name_to_controller_.find(controller_name);
  if (it == name_to_controller_.end()) {
    std::cout << "Controller for unregistered controller type: " << controller_name
              << " requested. Register all controllers before calling this method.";
    return std::shared_ptr<ControllerBase>();
  }
  return std::shared_ptr<ControllerBase>(it->second->Clone());
}

bool ControllerFactory::RegisterControllerTypeImpl(const std::string& controller_name,
                                                   const std::shared_ptr<ControllerBase>& controller) {
  std::cout << "Registered controller: " << controller_name << " \n";
  return name_to_controller_.insert(ControllerMap::value_type(controller_name, controller)).second;
}

bool ControllerFactory::UnregisterControllerTypeImpl(const std::string& controller_name) {
  return name_to_controller_.erase(controller_name) == 1;
}
}

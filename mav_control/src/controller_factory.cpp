/*
 * Copyright (C) 2014 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Pascal Gohl, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Sammy Omari, ASL, ETH Zurich, Switzerland
 * Copyright (C) 2014 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * This software is released to the Contestants of the european 
 * robotics challenges (EuRoC) for the use in stage 1. (Re)-distribution, whether 
 * in parts or entirely, is NOT PERMITTED. 
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 */

#include <mav_control/controller_factory.h>
#include <iostream>

namespace mav_controller_factory {
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

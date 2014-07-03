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

// controller_factory
#include <string>
#include <map>
#include <memory>
#include <mav_control/controller_base.h>

#define PP_CAT(a, b) a ## b

// Use this macro to register new types with the ControllerFactory:
#define MAV_CONTROL_REGISTER_CONTROLLER(type) \
  namespace mav_controller_factory { \
    static bool PP_CAT(type, __registered) = \
    ControllerFactory::RegisterControllerType<type>(#type); \
}

namespace mav_controller_factory {
class ControllerFactory {
 public:
  std::shared_ptr<ControllerBase> CreateController(const std::string& controller_name);

  static ControllerFactory& Instance() {
    static ControllerFactory factory;
    return factory;
  }

  template<typename ControllerType>
  static bool RegisterControllerType(const std::string& name) {
    return Instance().RegisterControllerTypeImpl(name, std::shared_ptr<ControllerType>(new ControllerType));
  }
 private:
  typedef std::map<std::string, std::shared_ptr<ControllerBase> > ControllerMap;
  ControllerMap name_to_controller_;

  ControllerFactory() = default;
  ControllerFactory(const ControllerFactory&) = delete;
  ControllerFactory& operator=(const ControllerFactory&) = delete;

  bool RegisterControllerTypeImpl(const std::string& controller_name, const std::shared_ptr<ControllerBase>& controller);
  bool UnregisterControllerTypeImpl(const std::string& controller_name);
  // ControllerBase* getActiveController();
};
}

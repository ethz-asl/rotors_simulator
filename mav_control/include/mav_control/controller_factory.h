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

// controller_factory
#include <map>
#include <memory>
#include <string>

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

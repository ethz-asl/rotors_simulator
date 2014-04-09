#include <mav_control/controller_factory.h>
#include <iostream>

namespace mav_controller_factory {
  std::shared_ptr<ControllerBase> ControllerFactory::CreateController(
    const std::string& controller_name) {

    ControllerMap::const_iterator it = name_to_controller_.find(controller_name);
    if (it == name_to_controller_.end()) {
      std::cout << "Controller for unregistered controller type: " 
        << controller_name 
        << " requested. Register all controllers before calling this method.";
      return std::shared_ptr<ControllerBase>();
    }
    return std::shared_ptr<ControllerBase>(it->second->New());
  }

  bool ControllerFactory::RegisterControllerTypeImpl(std::string& controller_name, 
    const std::shared_ptr<ControllerBase>& controller) {
    std::cout << "Registered\n";
    return name_to_controller_.insert(
      ControllerMap::value_type(controller_name,controller)).second;
  }

  bool ControllerFactory::UnregisterControllerTypeImpl(std::string& controller_name) {
    return name_to_controller_.erase(controller_name) == 1;
  }
  std::shared_ptr<ControllerBase> ControllerFactory::GetNewInstance(
    std::string controller_name) {
    return Instance().CreateController(controller_name);
  }
  ControllerFactory& ControllerFactory::Instance() {
    static ControllerFactory factory;
    return factory;
  }
  template<typename ControllerType>
  bool ControllerFactory::RegisterControllerType(std::string name) {
    return Instance().RegisterControllerTypeImpl(name,
      std::make_shared<ControllerType>());
  }
}



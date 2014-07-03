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
    return Instance().RegisterControllerTypeImpl(name, std::allocate_shared<ControllerType>
      (Eigen::aligned_allocator<ControllerType>()));
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

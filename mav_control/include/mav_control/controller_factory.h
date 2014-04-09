// controller_factory
#include <string>
#include <map>
#include <memory>
#include <mav_control/controller_base.h>

// Use this macro to register new controllers with the ControllerFactory
#define MAV_CONTROL_REGISTER_CONTROLLER(name, type) \
  namespace mav_controller_factory { \
    static bool name = ControllerFactory::RegisterControllerType<type>( \
      "name"); \
  }


namespace mav_controller_factory {
  class ControllerFactory {
    public:
      
      // ControllerFactory();
      // ~ControllerFactory();
      // ControllerFactory &operator=(const ControllerFactory &) { return *this; }
      
      template<typename ControllerType>
      static bool RegisterControllerType(std::string name);

      std::shared_ptr<ControllerBase> CreateController(
        const std::string& controller_name);

    // protected:
      static std::shared_ptr<ControllerBase> GetNewInstance(std::string name);
      
    private:
      typedef std::map<std::string, std::shared_ptr<ControllerBase> >
        ControllerMap;
      ControllerMap name_to_controller_;

      bool RegisterControllerTypeImpl(std::string& controller_name, 
        const std::shared_ptr<ControllerBase>& controller);
      bool UnregisterControllerTypeImpl(std::string& controller_name);
      // ControllerBase* getActiveController();

      static ControllerFactory& Instance();
      
  };
  
}

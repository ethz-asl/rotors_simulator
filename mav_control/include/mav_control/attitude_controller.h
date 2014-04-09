#include <mav_control/controller_base.h>
#include <mav_control/controller_factory.h>


class AttitudeController : public ControllerBase {
    AttitudeController();
    ~AttitudeController();
    void InitializeParams();
    void Publish();
  protected:
    void UpdateStates();
    void CalculateRefMotorVelocities();
  private:
    // 
};

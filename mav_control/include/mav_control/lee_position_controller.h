#include <mav_control/controller_base.h>
#include <mav_control/controller_factory.h>


class LeePositionController : public ControllerBase {
 public:
    LeePositionController();
    virtual ~LeePositionController();
    virtual void InitializeParams();
    virtual std::shared_ptr<ControllerBase> Clone();
    virtual void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const;

  private:
    Eigen::Matrix4Xd allocation_matrix_;
    Eigen::MatrixX4d angular_acc_to_rotor_velocities_;
    Eigen::Vector3d gain_position_;
    Eigen::Vector3d gain_velocity_;
    Eigen::Vector3d gain_attitude_;
    Eigen::Vector3d gain_angular_rate_;
    Eigen::Matrix3d inertia_matrix_;

    double mass_;

    const double gravity_;

    void ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
                                  Eigen::Vector3d* angular_acceleration) const;
    void ComputeDesiredAcceleration(Eigen::Vector3d* acceleration) const;

};

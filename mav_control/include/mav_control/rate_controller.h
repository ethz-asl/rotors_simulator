#include <mav_control/controller_base.h>
#include <mav_control/controller_factory.h>


class RateController : public ControllerBase {
 public:
    RateController();
    virtual ~RateController();
    virtual void InitializeParams();
    virtual std::shared_ptr<ControllerBase> Clone();
    virtual void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:
    Eigen::Matrix4Xd allocation_matrix_;
    Eigen::MatrixX4d angular_acc_to_rotor_velocities_;
    Eigen::Vector3d gain_angular_rate_;
    Eigen::Matrix3d inertia_matrix_;

    void ComputeDesiredAngularAcc(Eigen::Vector3d * angular_acceleration) const;

};

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

    double mass_;
    const double gravity_;

    void ComputeDesiredAngularAcc(Eigen::Vector3d * angular_acceleration) const;

};

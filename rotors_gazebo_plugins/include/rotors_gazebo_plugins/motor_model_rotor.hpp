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

#ifndef ROTORS_GAZEBO_PLUGINS_MOTOR_MODEL_ROTOR_H
#define ROTORS_GAZEBO_PLUGINS_MOTOR_MODEL_ROTOR_H

// 3RD PARTY
#include <Eigen/Core>
#include <boost/bind.hpp>
#include <gazebo/physics/physics.hh>

// USER
#include "rotors_gazebo_plugins/common.h"
#include "rotors_gazebo_plugins/motor_model.hpp"

namespace gazebo {

// Default values from KDE885 performance data chart
static constexpr double kDefaultMotorTorqueConst0 = 0.00218;
static constexpr double kDefaultMotorTorqueConst1 = 0.00318;
static constexpr double kDefaultMotorTorqueConst2 = 0.00047;

class MotorModelRotor : public MotorModel {
 public:
  MotorModelRotor(const physics::ModelPtr _model, const sdf::ElementPtr _motor)
      : MotorModel(),
        turning_direction_(spin::CCW),
        time_constant_up_(kDefaultTimeConstantUp),
        time_constant_down_(kDefaultTimeConstantDown),
        max_rot_velocity_(kDefaultMaxRotVelocity),
        min_rot_velocity_(kDefaultMinRotVelocity),
        thrust_constant_(kDefaultThrustConstant),
        moment_constant_(kDefaultMomentConstant),
        motor_torque_constant0_(kDefaultMotorTorqueConst0),
        motor_torque_constant1_(kDefaultMotorTorqueConst1),
        motor_torque_constant2_(kDefaultMotorTorqueConst2),
        rotor_drag_coefficient_(kDefaultRotorDragCoefficient),
        rolling_moment_coefficient_(kDefaultRollingMomentCoefficient),
        rotor_velocity_slowdown_sim_(kDefaultRotorVelocitySlowdownSim) {
    motor_ = _motor;
    joint_ =
        _model->GetJoint(motor_->GetElement("jointName")->Get<std::string>());
    link_ = _model->GetLink(motor_->GetElement("linkName")->Get<std::string>());
    InitializeParams();
  }

  virtual ~MotorModelRotor() {}

 protected:
  // Parameters
  std::string joint_name_;
  std::string link_name_;
  int turning_direction_;
  double time_constant_up_;
  double time_constant_down_;
  double max_rot_velocity_;
  double min_rot_velocity_;
  double thrust_constant_;
  double moment_constant_;
  // Motor torque to thrust relation (2nd order polynomial)
  double motor_torque_constant0_;
  double motor_torque_constant1_;
  double motor_torque_constant2_;
  double rotor_drag_coefficient_;
  double rolling_moment_coefficient_;
  double rotor_velocity_slowdown_sim_;

  std::unique_ptr<FirstOrderFilter<double>> rotor_velocity_filter_;

  sdf::ElementPtr motor_;
  physics::JointPtr joint_;
  physics::LinkPtr link_;

  void InitializeParams() {
    // Check spin direction.
    if (motor_->HasElement("spinDirection")) {
      std::string turning_direction =
          motor_->GetElement("spinDirection")->Get<std::string>();
      if (turning_direction == "cw")
        turning_direction_ = spin::CW;
      else if (turning_direction == "ccw")
        turning_direction = spin::CCW;
      else
        gzerr << "[motor_model_rotor] Spin not valid, using 'ccw.'\n";
    } else {
      gzwarn << "[motor_model_rotor] spinDirection not specified, using ccw.\n";
    }
    getSdfParam<double>(motor_, "rotorDragCoefficient", rotor_drag_coefficient_,
                        rotor_drag_coefficient_);
    getSdfParam<double>(motor_, "rollingMomentCoefficient",
                        rolling_moment_coefficient_,
                        rolling_moment_coefficient_);
    getSdfParam<double>(motor_, "maxRotVelocity", max_rot_velocity_,
                        max_rot_velocity_);
    getSdfParam<double>(motor_, "minRotVelocity", min_rot_velocity_,
                        min_rot_velocity_);
    getSdfParam<double>(motor_, "thrustConstant", thrust_constant_,
                        thrust_constant_);
    getSdfParam<double>(motor_, "momentConstant", moment_constant_,
                        moment_constant_);
    getSdfParam<double>(motor_, "motorTorqueConstant0", motor_torque_constant0_,
                        motor_torque_constant0_);
    getSdfParam<double>(motor_, "motorTorqueConstant1", motor_torque_constant1_,
                        motor_torque_constant1_);
    getSdfParam<double>(motor_, "motorTorqueConstant2", motor_torque_constant2_,
                        motor_torque_constant2_);
    getSdfParam<double>(motor_, "timeConstantUp", time_constant_up_,
                        time_constant_up_);
    getSdfParam<double>(motor_, "timeConstantDown", time_constant_down_,
                        time_constant_down_);
    getSdfParam<double>(motor_, "rotorVelocitySlowdownSim",
                        rotor_velocity_slowdown_sim_, 10);

    // Create the first order filter.
    rotor_velocity_filter_.reset(new FirstOrderFilter<double>(
        time_constant_up_, time_constant_down_, ref_motor_rot_vel_));
  }

  void Publish() {}  // No publishing here

  void UpdateForcesAndMoments() {
    double sim_motor_rot_vel = joint_->GetVelocity(0);
    if (sim_motor_rot_vel / (2 * M_PI) > 1 / (2 * sampling_time_)) {
      gzerr << "[motor_model_rotor] Aliasing on motor might occur. Consider "
               "making smaller simulation time steps or raising the "
               "rotor_velocity_slowdown_sim_ param.\n";
    }
    double real_motor_rot_velocity = sim_motor_rot_vel * rotor_velocity_slowdown_sim_;
    motor_rot_vel_ = turning_direction_ * real_motor_rot_velocity;
    // Get the direction of the rotor rotation.
    int real_motor_velocity_sign = (real_motor_rot_velocity > 0) - (real_motor_rot_velocity < 0);
    // Assuming symmetric propellers (or rotors) for the thrust calculation.
    double thrust = turning_direction_ * real_motor_velocity_sign *
                    real_motor_rot_velocity * real_motor_rot_velocity * thrust_constant_;

    // Apply a force to the link.
    link_->AddRelativeForce(ignition::math::Vector3d(0, 0, thrust));

    // Compute motor effort related to thrust force. It may be better to relate
    // this to drag torque as computed below. Collect experimental data to
    // determine.
    motor_rot_effort_ = motor_torque_constant0_ +
                        motor_torque_constant1_ * std::abs(thrust) +
                        motor_torque_constant2_ * thrust * thrust;

    // Forces from Philppe Martin's and Erwan Salaün's
    // 2010 IEEE Conference on Robotics and Automation paper
    // The True Role of Accelerometer Feedback in Quadrotor Control
    // - \omega * \lambda_1 * V_A^{\perp}
    ignition::math::Vector3d joint_axis = joint_->GlobalAxis(0);
    ignition::math::Vector3d body_velocity_W = link_->WorldLinearVel();
    ignition::math::Vector3d body_velocity_perpendicular =
        body_velocity_W - (body_velocity_W.Dot(joint_axis) * joint_axis);
    ignition::math::Vector3d air_drag = -std::abs(motor_rot_vel_) *
                                        rotor_drag_coefficient_ *
                                        body_velocity_perpendicular;

    // Apply air_drag to link.
    link_->AddForce(air_drag);
    // Moments get the parent link, such that the resulting torques can be
    // applied.
    physics::Link_V parent_links = link_->GetParentJointsLinks();
    // The tansformation from the parent_link to the link_.
    ignition::math::Pose3d pose_difference =
        link_->WorldCoGPose() - parent_links.at(0)->WorldCoGPose();
    ignition::math::Vector3d drag_torque(
        0, 0, -turning_direction_ * thrust * moment_constant_);

    // Transforming the drag torque into the parent frame to handle
    // arbitrary rotor orientations.
    ignition::math::Vector3d drag_torque_parent_frame =
        pose_difference.Rot().RotateVector(drag_torque);
    parent_links.at(0)->AddRelativeTorque(drag_torque_parent_frame);

    ignition::math::Vector3d rolling_moment;
    // - \omega * \mu_1 * V_A^{\perp}
    rolling_moment = -std::abs(motor_rot_vel_) * rolling_moment_coefficient_ *
                     body_velocity_perpendicular;
    parent_links.at(0)->AddTorque(rolling_moment);

    // Apply the filter on the motor's velocity.
    double ref_motor_rot_vel;
    ref_motor_rot_vel = rotor_velocity_filter_->updateFilter(ref_motor_rot_vel_,
                                                             sampling_time_);

    // Make sure max force is set, as it may be reset to 0 by a world reset any
    // time. (This cannot be done during Reset() because the change will be
    // undone by the Joint's reset function afterwards.)
    joint_->SetVelocity(0, turning_direction_ * ref_motor_rot_vel /
                               rotor_velocity_slowdown_sim_);
  }
};

}  // namespace gazebo

#endif  // ROTORS_GAZEBO_PLUGINS_MOTOR_MODEL_ROTOR_H

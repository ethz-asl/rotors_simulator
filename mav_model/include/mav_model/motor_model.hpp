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


#ifndef MAV_MODEL_MOTOR_MODEL_H
#define MAV_MODEL_MOTOR_MODEL_H
#include <Eigen/Eigen>

class MotorModel
{
  public:
    MotorModel() :
      motor_rot_vel_(0),
      ref_motor_rot_vel_(0) {}
    virtual ~MotorModel() {}
    void GetMotorVelocity(double &result) const {
      result = motor_rot_vel_;
    }
    void SetReferenceMotorVelocity(double ref_motor_rot_vel) {
      ref_motor_rot_vel_ = ref_motor_rot_vel;
    }

    virtual void InitializeParams() = 0;
    virtual void Publish() = 0;

  protected:
    double motor_rot_vel_;
    double ref_motor_rot_vel_;

    virtual void UpdateForcesAndMoments() = 0;
};

#endif // MAV_MODEL_MOTOR_MODEL_H

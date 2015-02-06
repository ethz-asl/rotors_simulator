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


#ifndef ROTORS_MODEL_MOTOR_MODEL_H
#define ROTORS_MODEL_MOTOR_MODEL_H

#include <Eigen/Eigen>

class MotorModel
{
  public:
    MotorModel()
        : motor_rot_vel_(0),
          ref_motor_rot_vel_(0),
          prev_sim_time_(0),
          sampling_time_(0.01) {}
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
    double prev_ref_motor_rot_vel_;
    double prev_sim_time_;
    double sampling_time_;


    virtual void UpdateForcesAndMoments() = 0;
};

#endif // ROTORS_MODEL_MOTOR_MODEL_H

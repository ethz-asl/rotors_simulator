/* Copyright (c) 2014 PX4 Development Team. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in
* the documentation and/or other materials provided with the
* distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
* used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
    * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

/**
 * @file px4_dummy_controller.h
 * Multicopter dummy controller to allow interface between euroc simulator and PX4
 * apps running on ROS
 *
 * @author Roman Bapst <romanbapst@yahoo.de>
 *
 */

#include <rotors_control/controller_base.h>
#include <rotors_control/controller_factory.h>
#include <ros/ros.h>
#include <mav_msgs/MotorSpeed.h>

class PX4dummyController : public ControllerBase {
 public:
  PX4dummyController();
  virtual ~PX4dummyController();
  virtual void InitializeParams();
  virtual std::shared_ptr<ControllerBase> Clone();
  virtual void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const;


  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
	ros::NodeHandle _n;
	ros::Subscriber _sub;

	float _motor_speeds[4];

	void MotorVelCallback(const mav_msgs::MotorSpeed &msg);

	void ComputeDesiredAngularAcc(Eigen::Vector3d * angular_acceleration) const;


};

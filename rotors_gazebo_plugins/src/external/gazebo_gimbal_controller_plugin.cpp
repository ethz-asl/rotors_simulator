/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include "rotors_gazebo_plugins/external/gazebo_gimbal_controller_plugin.h"

using namespace gazebo;
using namespace std;

GZ_REGISTER_MODEL_PLUGIN(GimbalControllerPlugin)

/////////////////////////////////////////////////
GimbalControllerPlugin::GimbalControllerPlugin()
  :status("closed")
{
  /// TODO: make these gains part of sdf xml
  this->pitchPid.Init(5, 0, 0, 0, 0, 0.3, -0.3);
  this->rollPid.Init(5, 0, 0, 0, 0, 0.3, -0.3);
  this->yawPid.Init(1.0, 0, 0, 0, 0, 1.0, -1.0);
  this->pitchCommand = 0.5* M_PI;
  this->rollCommand = 0;
  this->yawCommand = 0;
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::Load(
    physics::ModelPtr _model,
    sdf::ElementPtr _sdf)
{

  if(kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  this->model = _model;

  this->sdf = _sdf;

  std::string yawJointName = "cgo3_vertical_arm_joint";
  this->yawJoint = this->model->GetJoint(yawJointName);
  if (this->sdf->HasElement("joint_yaw"))
  {
    // Add names to map
    yawJointName = sdf->Get<std::string>("joint_yaw");
    if (this->model->GetJoint(yawJointName))
    {
      this->yawJoint = this->model->GetJoint(yawJointName);
    }
    else
    {
      gzwarn << "joint_yaw [" << yawJointName << "] does not exist?\n";
    }
  }
  if (!this->yawJoint)
  {
    gzerr << "GimbalControllerPlugin::Load ERROR! Can't get yaw joint '"
          << yawJointName << "' " << endl;
  }

  std::string rollJointName = "cgo3_horizontal_arm_joint";
  this->rollJoint = this->model->GetJoint(rollJointName);
  if (this->sdf->HasElement("joint_roll"))
  {
    // Add names to map
    rollJointName = sdf->Get<std::string>("joint_roll");
    if (this->model->GetJoint(rollJointName))
    {
      this->rollJoint = this->model->GetJoint(rollJointName);
    }
    else
    {
      gzwarn << "joint_roll [" << rollJointName << "] does not exist?\n";
    }
  }
  if (!this->rollJoint)
  {
    gzerr << "GimbalControllerPlugin::Load ERROR! Can't get roll joint '"
          << rollJointName << "' " << endl;
  }


  std::string pitchJointName = "cgo3_camera_joint";
  this->pitchJoint = this->model->GetJoint(pitchJointName);
  if (this->sdf->HasElement("joint_pitch"))
  {
    // Add names to map
    pitchJointName = sdf->Get<std::string>("joint_pitch");
    if (this->model->GetJoint(pitchJointName))
    {
      this->pitchJoint = this->model->GetJoint(pitchJointName);
    }
    else
    {
      gzwarn << "joint_pitch [" << pitchJointName << "] does not exist?\n";
    }
  }
  if (!this->pitchJoint)
  {
    gzerr << "GimbalControllerPlugin::Load ERROR! Can't get pitch joint '"
          << pitchJointName << "' " << endl;
  }


  // get imu sensor
  std::string imuSensorName = "camera_imu";
  if (this->sdf->HasElement("imu"))
  {
    // Add names to map
    imuSensorName = sdf->Get<std::string>("imu");
  }

  this->imuSensor = std::static_pointer_cast<sensors::ImuSensor>(
    sensors::SensorManager::Instance()->GetSensor(imuSensorName));

  if (!this->imuSensor)
  {
    gzerr << "GimbalControllerPlugin::Load ERROR! Can't get imu sensor '"
          << imuSensorName << "' " << endl;
  }
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::Init()
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  this->lastUpdateTime = this->model->GetWorld()->GetSimTime();

  // receive pitch command via gz transport
  std::string pitchTopic = std::string("~/") +  this->model->GetName() +
    "/gimbal_pitch_cmd";
  this->pitchSub = this->node->Subscribe(pitchTopic,
     &GimbalControllerPlugin::OnPitchStringMsg, this);
  // receive roll command via gz transport
  std::string rollTopic = std::string("~/") +  this->model->GetName() +
    "/gimbal_roll_cmd";
  this->rollSub = this->node->Subscribe(rollTopic,
     &GimbalControllerPlugin::OnRollStringMsg, this);
  // receive yaw command via gz transport
  std::string yawTopic = std::string("~/") +  this->model->GetName() +
    "/gimbal_yaw_cmd";
  this->yawSub = this->node->Subscribe(yawTopic,
     &GimbalControllerPlugin::OnYawStringMsg, this);

  // plugin update
  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GimbalControllerPlugin::OnUpdate, this)));

  // publish pitch status via gz transport
  pitchTopic = std::string("~/") +  this->model->GetName()
    + "/gimbal_pitch_status";
  this->pitchPub = node->Advertise<gazebo::msgs::Any>(pitchTopic);

  // publish roll status via gz transport
  rollTopic = std::string("~/") +  this->model->GetName()
    + "/gimbal_roll_status";
  this->rollPub = node->Advertise<gazebo::msgs::Any>(rollTopic);


  // publish yaw status via gz transport
  yawTopic = std::string("~/") +  this->model->GetName()
    + "/gimbal_yaw_status";
  this->yawPub = node->Advertise<gazebo::msgs::Any>(yawTopic);

  gzmsg << "GimbalControllerPlugin::Init" << std::endl;
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::OnPitchStringMsg(ConstAnyPtr &_msg)
{
//  gzdbg << "pitch command received " << _msg->double_value() << std::endl;
  this->pitchCommand = _msg->double_value();
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::OnRollStringMsg(ConstAnyPtr &_msg)
{
//  gzdbg << "roll command received " << _msg->double_value() << std::endl;
  this->rollCommand = _msg->double_value();
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::OnYawStringMsg(ConstAnyPtr &_msg)
{
//  gzdbg << "yaw command received " << _msg->double_value() << std::endl;
  this->yawCommand = _msg->double_value();
}

/////////////////////////////////////////////////
ignition::ignition::math::Vector3d GimbalControllerPlugin::ThreeAxisRot(
  double r11, double r12, double r21, double r31, double r32)
{
  return ignition::ignition::math::Vector3d d(
    atan2( r31, r32 ),
    asin ( r21 ),
    atan2( r11, r12 ));
}

/////////////////////////////////////////////////
ignition::ignition::math::Vector3d GimbalControllerPlugin::QtoZXY(
  const ignition::math::Quaterniond &_q)
{
  // taken from
  // http://bediyap.com/programming/convert-quaternion-to-euler-rotations/
  // case zxy:
  ignition::ignition::math::Vector3d result = this->ThreeAxisRot(
    -2*(_q.X()*_q.Y() - _q.W()*_q.Z()),
    _q.W()*_q.W() - _q.X()*_q.X() + _q.Y()*_q.Y() - _q.Z()*_q.Z(),
    2*(_q.Y()*_q.Z() + _q.W()*_q.X()),
    -2*(_q.X()*_q.Z() - _q.W()*_q.Y()),
    _q.W()*_q.W() - _q.X()*_q.X() - _q.Y()*_q.Y() + _q.Z()*_q.Z());
  return result;
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::OnUpdate()
{
  if (!this->pitchJoint || !this->rollJoint || !this->yawJoint)
    return;

  common::Time time = this->model->GetWorld()->GetSimTime();
  if (time < this->lastUpdateTime)
  {
    gzerr << "time reset event\n";
    this->lastUpdateTime = time;
    return;
  }
  else if (time > this->lastUpdateTime)
  {
    double dt = (this->lastUpdateTime - time).Double();

    // anything to do with gazebo joint has
    // hardcoded negative joint axis for pitch and roll
    // TODO: make joint direction a parameter
    const double rDir = -1;
    const double pDir = -1;
    const double yDir = 1;

    // truncate command inside joint angle limits
    double rollLimited = ignition::math::clamp(this->rollCommand,
      rDir*this->rollJoint->GetUpperLimit(0).Radian(),
	  rDir*this->rollJoint->GetLowerLimit(0).Radian());
    double pitchLimited = ignition::math::clamp(this->pitchCommand,
      pDir*this->pitchJoint->GetUpperLimit(0).Radian(),
      pDir*this->pitchJoint->GetLowerLimit(0).Radian());
    double yawLimited = ignition::math::clamp(this->yawCommand,
      yDir*this->yawJoint->GetLowerLimit(0).Radian(),
	  yDir*this->yawJoint->GetUpperLimit(0).Radian());

    ignition::math::Quaterniond commandRPY(
      rollLimited, pitchLimited, yawLimited);


    /// Get current joint angles (in sensor frame):

    /// currentAngleYPRVariable is defined in roll-pitch-yaw-fixed-axis
    /// and gimbal is constructed using yaw-roll-pitch-variable-axis
    ignition::ignition::math::Vector3d currentAngleYPRVariable(
      this->imuSensor->Orientation().Euler());
    ignition::ignition::math::Vector3d currentAnglePRYVariable(
      this->QtoZXY(currentAngleYPRVariable));

    /// get joint limits (in sensor frame)
    /// TODO: move to Load() if limits do not change
    ignition::ignition::math::Vector3d lowerLimitsPRY
      (pDir*this->pitchJoint->GetLowerLimit(0).Radian(),
       rDir*this->rollJoint->GetLowerLimit(0).Radian(),
       yDir*this->yawJoint->GetLowerLimit(0).Radian());
    ignition::ignition::math::Vector3d upperLimitsPRY
      (pDir*this->pitchJoint->GetUpperLimit(0).Radian(),
       rDir*this->rollJoint->GetUpperLimit(0).Radian(),
       yDir*this->yawJoint->GetUpperLimit(0).Radian());

    // normalize errors
    double pitchError = this->ShortestAngularDistance(
      pitchLimited, currentAnglePRYVariable.X());
    double rollError = this->ShortestAngularDistance(
      rollLimited, currentAnglePRYVariable.Y());
    double yawError = this->ShortestAngularDistance(
      yawLimited, currentAnglePRYVariable.Z());

    // Clamp errors based on current angle and estimated errors from rotations:
    // given error = current - target, then
    // if target (current angle - error) is outside joint limit, truncate error
    // so that current angle - error is within joint limit, i.e.:
    // lower limit < current angle - error < upper limit
    // or
    // current angle - lower limit > error > current angle - upper limit
    // re-expressed as clamps:
    // hardcoded negative joint axis for pitch and roll
    if (lowerLimitsPRY.X() < upperLimitsPRY.X())
    {
      pitchError = ignition::math::clamp(pitchError,
        currentAnglePRYVariable.X() - upperLimitsPRY.X(),
        currentAnglePRYVariable.X() - lowerLimitsPRY.X());
    }
    else
    {
      pitchError = ignition::math::clamp(pitchError,
        currentAnglePRYVariable.X() - lowerLimitsPRY.X(),
        currentAnglePRYVariable.X() - upperLimitsPRY.X());
    }
    if (lowerLimitsPRY.Y() < upperLimitsPRY.Y())
    {
      rollError = ignition::math::clamp(rollError,
        currentAnglePRYVariable.Y() - upperLimitsPRY.Y(),
        currentAnglePRYVariable.Y() - lowerLimitsPRY.Y());
    }
    else
    {
      rollError = ignition::math::clamp(rollError,
        currentAnglePRYVariable.Y() - lowerLimitsPRY.Y(),
        currentAnglePRYVariable.Y() - upperLimitsPRY.Y());
    }
    if (lowerLimitsPRY.Z() < upperLimitsPRY.Z())
    {
      yawError = ignition::math::clamp(yawError,
        currentAnglePRYVariable.Z() - upperLimitsPRY.Z(),
        currentAnglePRYVariable.Z() - lowerLimitsPRY.Z());
    }
    else
    {
      yawError = ignition::math::clamp(yawError,
        currentAnglePRYVariable.Z() - lowerLimitsPRY.Z(),
        currentAnglePRYVariable.Z() - upperLimitsPRY.Z());
    }

    // apply forces to move gimbal
    double pitchForce = this->pitchPid.Update(pitchError, dt);
    this->pitchJoint->SetForce(0, pDir*pitchForce);

    double rollForce = this->rollPid.Update(rollError, dt);
    this->rollJoint->SetForce(0, rDir*rollForce);

    double yawForce = this->yawPid.Update(yawError, dt);
    this->yawJoint->SetForce(0, yDir*yawForce);

    // ignition::ignition::math::Vector3d angles = this->imuSensor->Orientation().Euler();
    // gzerr << "ang[" << angles.X() << ", " << angles.Y() << ", " << angles.Z()
    //       << "] cmd[ " << this->rollCommand
    //       << ", " << this->pitchCommand << ", " << this->yawCommand
    //       << "] err[ " << rollError
    //       << ", " << pitchError << ", " << yawError
    //       << "] frc[ " << rollForce
    //       << ", " << pitchForce << ", " << yawForce << "]\n";


    this->lastUpdateTime = time;
  }

  static int i =1000;
  if (++i>100)
  {
    i = 0;

    gazebo::msgs::Any m;
    m.set_type(gazebo::msgs::Any_ValueType_DOUBLE);

    m.set_double_value(this->pitchJoint->GetAngle(0).Radian());
    this->pitchPub->Publish(m);

    m.set_double_value(this->rollJoint->GetAngle(0).Radian());
    this->rollPub->Publish(m);

    m.set_double_value(this->yawJoint->GetAngle(0).Radian());
    this->yawPub->Publish(m);
  }
}

/////////////////////////////////////////////////
double GimbalControllerPlugin::NormalizeAbout(double _angle, double reference)
{
  double diff = _angle - reference;
  // normalize diff about (-pi, pi], then add reference
  while (diff <= -M_PI)
  {
    diff += 2.0*M_PI;
  }
  while (diff > M_PI)
  {
    diff -= 2.0*M_PI;
  }
  return diff + reference;
}

/////////////////////////////////////////////////
double GimbalControllerPlugin::ShortestAngularDistance(double _from, double _to)
{
  return this->NormalizeAbout(_to, _from) - _from;
}

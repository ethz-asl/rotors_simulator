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
#include "rotors_gazebo_plugins/external/gazebo_front_gimbal_plugin.h"

namespace gazebo {


static double cmdTargetRoll = 0;
static double cmdTargetPitch = 0;

FrontGimbalPlugin::~FrontGimbalPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(_update_connection);

}

void FrontGimbalPlugin::Load( physics::ModelPtr model, sdf::ElementPtr sdf)
{
  _model = model;
  _sdf = sdf;
  
  _node = transport::NodePtr(new transport::Node());
  _node->Init();

  _targetRoll  = 0;
  _targetPitch = 0;
  _pidRoll.Init(1, 0.1, 0.2, 100, -100, 50, -50);
  _pidPitch.Init(1, 0.1, 0.2, 100, -100, 50, -50);
  
  std::string strRollName = "niv1/gimbal_roll_joint";
  _jointRoll = _model->GetJoint(strRollName);
  if (!_jointRoll)
  {
    gzerr << "FrontGimbalPlugin::Load ERROR!::" << strRollName << std::endl;
  }

  std::string strPitchName = "niv1/gimbal_pitch_joint";
  _jointPitch = _model->GetJoint(strPitchName);
  if (!_jointPitch)
  {
    gzerr << "FrontGimbalPlugin::Load ERROR!::" << strPitchName << std::endl;
  }

  _update_connection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&FrontGimbalPlugin::OnUpdate, this, _1));
  gzmsg << "FrontGimbalPlugin::Init" << std::endl;
}

void FrontGimbalPlugin::OnUpdate(const common::UpdateInfo& _info)
{
 //if (!_jointPitch || !_jointRoll) { return; }

  CreateCommunicationChannels();

  double angleRoll;
  double anglePitch;
  double forceRoll;
  double errorRoll;
  double forcePitch;
  double errorPitch;

  angleRoll = _jointPitch->GetWorldPose().rot.GetRoll() * 180.0 / 3.141592;  // TODO(burrimi): Check tf.
  anglePitch = _jointPitch->GetWorldPose().rot.GetPitch() * 180.0 / 3.141592;  // TODO(burrimi): Check tf.


  _targetRoll = cmdTargetRoll;
  errorRoll = angleRoll - _targetRoll;
  forceRoll = _pidRoll.Update( errorRoll, 0.0025);
  _jointRoll->SetForce(0,forceRoll);

  _targetPitch = cmdTargetPitch;
  errorPitch = anglePitch - _targetPitch;
  forcePitch = _pidPitch.Update( errorPitch, 0.0025);
  _jointPitch->SetForce(0,forcePitch);
  
}

void CommandMsgCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  cmdTargetRoll = msg->data[0];
  cmdTargetPitch = msg->data[1];
}

void FrontGimbalPlugin::CreateCommunicationChannels() 
{
  static bool created = false;

  if( created ) { return; }
  created = true;

  ros::NodeHandle nh;
  //_nodeRos = new ros::NodeHandle("niv1");
  _subCommand = nh.subscribe("/niv1/gimbal_front_cmd", 10, CommandMsgCallback);
  
}

GZ_REGISTER_MODEL_PLUGIN(FrontGimbalPlugin);

}  // namespace gazebo

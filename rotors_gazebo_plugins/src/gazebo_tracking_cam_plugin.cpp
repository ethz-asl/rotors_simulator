/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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
 *
 * Modifications by David Rohr, drohr@student.ethz.ch
 *
 */

#include "rotors_gazebo_plugins/gazebo_tracking_cam_plugin.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(GazeboTrackingCam)

/////////////////////////////////////////////////
GazeboTrackingCam::GazeboTrackingCam()
{
    gzdbg<<"template plugin constructed\n";
}

/////////////////////////////////////////////////
GazeboTrackingCam::~GazeboTrackingCam()
{
    gzdbg<<"template plugin destructed\n";
}

/////////////////////////////////////////////////
void GazeboTrackingCam::Load(physics::ModelPtr _model,
                              sdf::ElementPtr _sdf)
{
    gzdbg<<"load called"<<std::endl;

    GZ_ASSERT(_model, "TemplatePlugin _model pointer is NULL");
    GZ_ASSERT(_sdf, "TemplatePlugin _sdf pointer is NULL");
    this->model = _model;
    this->sdf = _sdf;
    
    this->world = this->model->GetWorld();
    GZ_ASSERT(this->world, "TemplatePlugin world pointer is NULL");
    
#if GAZEBO_MAJOR_VERSION >= 9
    this->physics = this->world->Physics();
#else
    this->physics = this->world->GetPhysicsEngine();
#endif
    GZ_ASSERT(this->physics, "TemplatePlugin physics pointer is NULL");
    
    GZ_ASSERT(_sdf, "TemplatePlugin _sdf pointer is NULL");
    
    namespace_.clear();
    
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboTrackingCam::OnUpdate, this));

    gzdbg<<"model name: "<<model->GetName()<<"\n";
    model->Print("");
    gzdbg<<"model child count: "<<model->GetChildCount()<<"\n";

    if (_sdf->HasElement("panJoint")) {
        std::string pan_joint = _sdf->GetElement("panJoint")->Get<std::string>();
        pan_joint_ = model->GetJoint(pan_joint);
        gzdbg<<"pan_joint: " + pan_joint + "\n";
    } else {
        gzerr << "[gazebo_tracking_cam] Please specify a panJoint.\n";
    }

    if (_sdf->HasElement("tiltJoint")) {
        std::string tilt_joint = _sdf->GetElement("tiltJoint")->Get<std::string>();
        tilt_joint_ = model->GetJoint(tilt_joint);
        gzdbg<<"tilt_joint: " + tilt_joint + "\n";
    } else {
        gzerr << "[gazebo_tracking_cam] Please specify a tiltJoint.\n";
    }

    if (_sdf->HasElement("cameraLink")) {
        std::string cam_link = _sdf->GetElement("cameraLink")->Get<std::string>();
        cam_link_ = model->GetLink(cam_link);
        gzdbg<<"cam_link: " + cam_link + "\n";

    } else {
        gzerr << "[gazebo_tracking_cam] Please specify a cameraLink.\n";
    }

    if (_sdf->HasElement("cameraBaseLink")) {
        std::string cam_base_link = _sdf->GetElement("cameraBaseLink")->Get<std::string>();
        cam_base_link_ = model->GetLink(cam_base_link);
        gzdbg<<"cam_base_link: " + cam_base_link + "\n";

    } else {
        gzerr << "[gazebo_tracking_cam] Please specify a cameraBaseLink.\n";
    }

    if (_sdf->HasElement("targetLink") && _sdf->HasElement("targetName")) {
        std::string target_link = _sdf->GetElement("targetLink")->Get<std::string>();
        std::string target_name = _sdf->GetElement("targetName")->Get<std::string>();
        target_ = world->ModelByName(target_name);
        target_link_ = target_->GetLink(target_link);
    } else {
        gzerr << "[gazebo_tracking_cam] Please specify a targetLink, targetName.\n";
    }

}

/////////////////////////////////////////////////

void GazeboTrackingCam::OnUpdate()
{

#if GAZEBO_MAJOR_VERSION >= 9
     ignition::math::Pose3d cam_pose = cam_link_->WorldPose();
     ignition::math::Pose3d cam_base_pose = cam_base_link_->WorldPose();
     ignition::math::Pose3d target_pose = target_link_->WorldPose();
#else
     ignition::math::Pose3d cam_pose = ignitionFromGazeboMath(cam_link_->GetWorldPose());
     ignition::math::Pose3d cam_base_pose = ignitionFromGazeboMath(cam_base_link_->GetWorldPose());
     ignition::math::Pose3d target_pose = ignitionFromGazeboMath(target_link_->GetWorldPose());
#endif

    V3D target_pos  = target_pose.Pos() - cam_pose.Pos();

    V3D forward = cam_base_pose.Rot().RotateVector(V3D(1,0,0));
    V3D upward = cam_base_pose.Rot().RotateVector(V3D(0,0,1));
    V3D left = upward.Cross(forward);

    double tilt = asin(upward.Dot(target_pos)/target_pos.Length());
    double pan = atan2(left.Dot(target_pos),forward.Dot(target_pos));

    pan_joint_->SetPosition(0,pan);
    tilt_joint_->SetPosition(0,tilt);

}



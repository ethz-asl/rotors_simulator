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
    this->model_ = _model;
    this->sdf_ = _sdf;
    
    this->world_ = this->model_->GetWorld();
    GZ_ASSERT(this->world_, "TemplatePlugin world pointer is NULL");
    
#if GAZEBO_MAJOR_VERSION >= 9
    this->physics_ = this->world_->Physics();
#else
    this->physics_ = this->world_->GetPhysicsEngine();
#endif
    GZ_ASSERT(this->physics_, "TemplatePlugin physics pointer is NULL");
    
    GZ_ASSERT(_sdf, "TemplatePlugin _sdf pointer is NULL");
    
    namespace_.clear();

    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init();
    
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboTrackingCam::OnUpdate, this));

    gzdbg<<"model name: "<<model_->GetName()<<"\n";
    model_->Print("");
    gzdbg<<"model child count: "<<model_->GetChildCount()<<"\n";

    if (_sdf->HasElement("panJoint")) {
        std::string pan_joint = _sdf->GetElement("panJoint")->Get<std::string>();
        pan_joint_ = model_->GetJoint(pan_joint);
        gzdbg<<"pan_joint: " + pan_joint + "\n";
    } else {
        gzerr << "[gazebo_tracking_cam] Please specify a panJoint.\n";
    }

    if (_sdf->HasElement("tiltJoint")) {
        std::string tilt_joint = _sdf->GetElement("tiltJoint")->Get<std::string>();
        tilt_joint_ = model_->GetJoint(tilt_joint);
        gzdbg<<"tilt_joint: " + tilt_joint + "\n";
    } else {
        gzerr << "[gazebo_tracking_cam] Please specify a tiltJoint.\n";
    }

    if (_sdf->HasElement("cameraLink")) {
        std::string cam_link = _sdf->GetElement("cameraLink")->Get<std::string>();
        cam_link_ = model_->GetLink(cam_link);
        gzdbg<<"cam_link: " + cam_link + "\n";

    } else {
        gzerr << "[gazebo_tracking_cam] Please specify a cameraLink.\n";
    }

    if (_sdf->HasElement("cameraBaseLink")) {
        std::string cam_base_link = _sdf->GetElement("cameraBaseLink")->Get<std::string>();
        cam_base_link_ = model_->GetLink(cam_base_link);
        gzdbg<<"cam_base_link: " + cam_base_link + "\n";

    } else {
        gzerr << "[gazebo_tracking_cam] Please specify a cameraBaseLink.\n";
    }

    if (_sdf->HasElement("targetPosTopic")) {
        target_pos_subtopic_ = _sdf->GetElement("targetPosTopic")->Get<std::string>();
        gzwarn<<"targetPosTopic: " + target_pos_subtopic_ + "\n";

    } else {
        gzwarn << "[gazebo_tracking_cam] Please specify a targetPosTopic.\n";
    }
}

/////////////////////////////////////////////////


void GazeboTrackingCam::OnUpdate()
{

    if (!pubs_and_subs_created_) {
        target_pos_sub_ = node_handle_->Subscribe(target_pos_subtopic_, &GazeboTrackingCam::TargetPosCallback, this);
        pubs_and_subs_created_ = true;
    }

    /*if (!init_) {
        // Have to wait until simulation start to make sure tracking target has spawned before accessing it
        init_ = true;
        if (sdf_->HasElement("targetLink") && sdf_->HasElement("targetName")) {
            std::string target_link = sdf_->GetElement("targetLink")->Get<std::string>();
            std::string target_name = sdf_->GetElement("targetName")->Get<std::string>();

            common::Time::MSleep(10000);
            target_ = world_->ModelByName(target_name);

            while(!target_){
                 target_ = world_->ModelByName(target_name);
                 common::Time::MSleep(1500);
                 gzdbg<<"waiting for target to spawn... \n";
            }

            target_link_ = target_->GetLink(target_link);

            if(!target_link_)
                gzerr<<"couldn't find specified link\n";

        } else {
            gzerr << "[gazebo_tracking_cam] Please specify a targetLink, targetName.\n";
        }
    } else {}*/

#if GAZEBO_MAJOR_VERSION >= 9
     ignition::math::Pose3d cam_pose = cam_link_->WorldPose();
     ignition::math::Pose3d cam_base_pose = cam_base_link_->WorldPose();
     //ignition::math::Pose3d target_pose = target_link_->WorldPose();
#else
     ignition::math::Pose3d cam_pose = ignitionFromGazeboMath(cam_link_->GetWorldPose());
     ignition::math::Pose3d cam_base_pose = ignitionFromGazeboMath(cam_base_link_->GetWorldPose());
     //ignition::math::Pose3d target_pose = ignitionFromGazeboMath(target_link_->GetWorldPose());
#endif

    V3D rel_target_pos  = target_pos_ - cam_pose.Pos();

    V3D forward = cam_base_pose.Rot().RotateVector(V3D(1,0,0));
    V3D upward = cam_base_pose.Rot().RotateVector(V3D(0,0,1));
    V3D left = upward.Cross(forward);

    double tilt = asin(upward.Dot(rel_target_pos)/rel_target_pos.Length());
    double pan = atan2(left.Dot(rel_target_pos),forward.Dot(rel_target_pos));

    pan_joint_->SetPosition(0,pan);
    tilt_joint_->SetPosition(0,tilt);


}



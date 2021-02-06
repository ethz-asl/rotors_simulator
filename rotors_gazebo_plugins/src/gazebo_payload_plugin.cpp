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

#include "rotors_gazebo_plugins/gazebo_payload_plugin.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(GazeboPayloadPlugin)

/////////////////////////////////////////////////
GazeboPayloadPlugin::GazeboPayloadPlugin()
{
    gzdbg<<"GazeboPayloadPlugin constructed\n";
}

/////////////////////////////////////////////////
GazeboPayloadPlugin::~GazeboPayloadPlugin()
{
    gzdbg<<"GazeboPayloadPlugin destructed\n";
}

/////////////////////////////////////////////////
void GazeboPayloadPlugin::Load(physics::ModelPtr _model,
                              sdf::ElementPtr _sdf)
{
    gzdbg<<"load called"<<std::endl;

    GZ_ASSERT(_model, "GazeboPayloadPlugin _model pointer is NULL");
    GZ_ASSERT(_sdf, "GazeboPayloadPlugin _sdf pointer is NULL");

    this->model_ = _model;
    this->world_ = this->model_->GetWorld();
    GZ_ASSERT(this->world_, "GazeboPayloadPlugin world pointer is NULL");
    
    namespace_.clear();
    
    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init();

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboPayloadPlugin::OnUpdate, this));

    /*
    if (_sdf->HasElement("link")) {
        std::string link_name = _sdf->Get<std::string>("link");
        this->link_ = this->model_->GetLink(link_name);
    } else if (!this->link_) {
        gzerr << "Link not found\n";
    }
    */

    /*
    std::string joint_name;
    if (_sdf->HasElement("joint")) {
        joint_name = _sdf->Get<std::string>("joint");
        this->joint_ = this->model_->GetJoint(joint_name);
    }

    if (!this->joint_){
        gzerr<<"Joint "<<joint_name<<" not found \n";

    } else {
        payload_ = joint_->GetChild();
        parent_ = joint_->GetParent();
    }
    */

    if (_sdf->HasElement("payload")) {
        std::string payload = _sdf->Get<std::string>("payload");
        this->payload_ = this->model_->GetLink(payload);
    }

    if (_sdf->HasElement("parent")) {
        std::string parent = _sdf->Get<std::string>("parent");
        this->parent_ = this->model_->GetLink(parent);
    }

    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzerr<<"Please specify a robotNamespace.\n";

    if (_sdf->HasElement("hoistPosParent"))
        hoist_pos_parent_ = _sdf->Get<ignition::math::Vector3d>("hoistPosParent");
    else
        hoist_pos_parent_ = ignition::math::Vector3d(0,0,0);

    if (_sdf->HasElement("hoistPosPayload"))
        hoist_pos_payload_ = _sdf->Get<ignition::math::Vector3d>("hoistPosPayload");
    else
        hoist_pos_payload_ = ignition::math::Vector3d(0,0,0);

    if (_sdf->HasElement("dropTopic")) {
        drop_topic_ = _sdf->Get<std::string>("dropTopic");
    }
}

/////////////////////////////////////////////////
void GazeboPayloadPlugin::CreatePubsAndSubs(){

    drop_sub_ = node_handle_->Subscribe("~/" + namespace_ + "/" + drop_topic_, &GazeboPayloadPlugin::DropCallback, this);
}

/////////////////////////////////////////////////
void GazeboPayloadPlugin::DropCallback(Float32Ptr &drop_msg){

    if (drop_msg->data()>0.0) {
        trigg_ = true;
        gzdbg<<"trigg\n";
    } else {
        trigg_ = false;
    }
}

/////////////////////////////////////////////////
void GazeboPayloadPlugin::OnUpdate()
{

    if (!pubs_and_subs_created_) {
      CreatePubsAndSubs();
      pubs_and_subs_created_ = true;
    }

    common::Time current_time = world_->SimTime();

    ignition::math::Pose3d pose_parent = parent_->WorldPose();
    ignition::math::Pose3d pose_payload = payload_->WorldPose();
    ignition::math::Pose3d pose_payload_reset;
    pose_payload_reset.Pos() = pose_parent.Pos() + pose_parent.Rot().RotateVector(hoist_pos_parent_) - pose_payload.Rot().RotateVector(hoist_pos_payload_);
    pose_payload_reset.Rot() = pose_parent.Rot();

    //pose_parent.Pos() += pose_parent.Rot().RotateVector(hoist_pos_parent_);
    //pose_payload.Pos() += pose_payload.Rot().RotateVector(hoist_pos_payload_);

    if(ini_){
        q_pr_pa_ = pose_parent.Rot().Inverse()*pose_payload.Rot();
        ini_=false;
    }

    // Position error quantities (expressed in world frame)
    ignition::math::Vector3d lin_vel_err =  payload_->WorldLinearVel(hoist_pos_payload_) - parent_->WorldLinearVel(hoist_pos_parent_);
    //ignition::math::Vector3d pos_err = pose_parent.Pos()-pose_payload.Pos();
    ignition::math::Vector3d pos_err = pose_payload_reset.Pos()-pose_payload.Pos();
    lin_vel_err.Correct();

    // Orientation error quantities (expressed in payload frame)
    ignition::math::Vector3d rot_vel_err = pose_payload.Rot().RotateVectorReverse(payload_->WorldAngularVel()-parent_->WorldAngularVel());
    ignition::math::Quaterniond rot_err_q = pose_parent.Rot().Inverse()*pose_payload.Rot();
    double angle_err;
    ignition::math::Vector3d rot_err;
    rot_err_q.ToAxis(rot_err,angle_err);
    rot_err *= -angle_err;

    //state machine
    bool close;
    if (pos_err.Length()<0.05 && lin_vel_err.Length()<0.1 && rot_err.Length()<0.17 && rot_vel_err.Length()<0.17)
        close = true;
    else
        close = false;

    if (trigg_ && !reload_ && !drop_) {
        drop_ = true;
        reload_ = false;
        reload_timer_.Start();

    } else if (drop_ && reload_timer_.GetElapsed().Float()>8.0) {
        drop_ = false;
        reload_ = true;
        reload_timer_.Stop();
        reload_timer_.Reset();

    } else if (close && reload_) {
        drop_ = false;
        reload_ = false;
    }

    //ignition::math::Quaterniond rot_err = q_pr_pa_.Inverse()*pose_parent.Rot().Inverse()*pose_payload.Rot();

    double omega = 3*33.3;    // natural frequency
    double zeta = 1.0;      // damping ratio
    double mass = 0.25;
    double inertia = 0.4*mass*0.0025; // inertia of solid sphere: 0.4*m*r²
    double k_p_lin = omega*omega*mass;
    double k_p_rot = omega*omega*inertia;
    double k_d_lin = 2*zeta*omega*mass;
    double k_d_rot = 2*zeta*omega*inertia;
    double reactio;

    if(drop_) {
        k_p_lin = 0;
        k_d_lin = 0;
        k_p_rot = 0;
        k_d_rot = 0;
        reactio = 0;
        reset_ = false;

    } else if(reload_) {
        reactio = 0;
        if(!reset_){
            //get it somewhat close...
            payload_->SetWorldPose(pose_payload_reset);
            payload_->SetAngularVel(pose_payload.Rot().RotateVectorReverse(parent_->WorldAngularVel()));
            payload_->SetLinearVel(pose_payload.Rot().RotateVectorReverse(parent_->WorldLinearVel(hoist_pos_parent_)));
            k_p_lin = 0;
            k_d_lin = 0;
            k_p_rot = 0;
            k_d_rot = 0;
            reset_ = true;
        }

    } else {
        reactio = 1;
        reset_ = false;
    }

    ignition::math::Vector3d force  = k_p_lin*pos_err-k_d_lin*lin_vel_err;
    ignition::math::Vector3d torque = k_p_rot*rot_err-k_d_rot*rot_vel_err;

    /*
    if(force.Length()>100)
        force = force/force.Length()*100;
    */

    force.Correct();
    torque.Correct();

    /*
    gzdbg<<"x: "<<force.X()<<" y: "<<force.Y()<<" z: "<<force.Z()<<"\n";
    gzdbg<<"rx: "<<pos_err.X()<<" ry: "<<pos_err.Y()<<" rz: "<<pos_err.Z()<<"\n";
    gzdbg<<"vx: "<<lin_vel_err.X()<<" vy: "<<lin_vel_err.Y()<<" vz: "<<lin_vel_err.Z()<<"\n";
    */

    parent_->AddForceAtRelativePosition(-reactio*force, hoist_pos_parent_);
    payload_->AddForceAtRelativePosition(force, hoist_pos_payload_);
    payload_->AddRelativeTorque(torque);

}



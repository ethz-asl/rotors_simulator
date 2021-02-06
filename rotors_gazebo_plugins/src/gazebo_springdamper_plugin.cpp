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

#include "rotors_gazebo_plugins/gazebo_springdamper_plugin.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(GazeboSpringDamper)

/////////////////////////////////////////////////
GazeboSpringDamper::GazeboSpringDamper()
{
    gzdbg<<"GazeboSpringDamper plugin constructed\n";
}

/////////////////////////////////////////////////
GazeboSpringDamper::~GazeboSpringDamper()
{
    gzdbg<<"GazeboSpringDamper plugin destructed\n";
}

/////////////////////////////////////////////////
void GazeboSpringDamper::Load(physics::ModelPtr _model,
                              sdf::ElementPtr _sdf)
{
    gzdbg<<"load called"<<std::endl;

    GZ_ASSERT(_model, "GazeboSpringDamper _model pointer is NULL");
    GZ_ASSERT(_sdf, "GazeboSpringDamper _sdf pointer is NULL");
    this->model = _model;
    this->sdf = _sdf;
    
    this->world = this->model->GetWorld();
    GZ_ASSERT(this->world, "TemplatePlugin world pointer is NULL");
    
#if GAZEBO_MAJOR_VERSION >= 9
    this->physics = this->world->Physics();
#else
    this->physics = this->world->GetPhysicsEngine();
#endif
    GZ_ASSERT(this->physics, "GazeboSpringDamper physics pointer is NULL");
    
    GZ_ASSERT(_sdf, "GazeboSpringDamper _sdf pointer is NULL");
    
    namespace_.clear();

    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzerr << "[GazeboSpringDamper] Please specify a robotNamespace.\n";

    if (_sdf->HasElement("damping"))
        c_ = _sdf->GetElement("damping")->Get<double>();
    else
        gzerr << "[GazeboSpringDamper] Please specify a damping.\n";


    if (_sdf->HasElement("stiffness"))
        k_ = _sdf->GetElement("stiffness")->Get<double>();
    else
        gzerr << "[GazeboSpringDamper] Please specify a stiffness.\n";

    if (_sdf->HasElement("jointName")) {
        joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
        joint_ = model->GetJoint(joint_name_);

        if (joint_){
            joint_->SetStiffnessDamping(0,k_,c_,0);
            //this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboSpringDamper::OnUpdate, this));
            //joint_->SetPosition(0,0);

        } else {
            gzwarn << "joint [" << joint_name_ << "] not found, won't run. \n";
        }

    } else {
        gzerr << "[GazeboSpringDamper] Please specify a jointName, won't run.\n";
    }

}

/////////////////////////////////////////////////

void GazeboSpringDamper::OnUpdate()
{

    joint_->SetForce(0,-joint_->Position()*k_ - joint_->GetVelocity(0)*c_);
}



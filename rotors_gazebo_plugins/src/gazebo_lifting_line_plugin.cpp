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

#include "rotors_gazebo_plugins/gazebo_lifting_line_plugin.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(GazeboLiftingLine)

/////////////////////////////////////////////////
GazeboLiftingLine::GazeboLiftingLine()
{
    gzdbg<<"lifting line plugin constructed\n";
}

/////////////////////////////////////////////////
GazeboLiftingLine::~GazeboLiftingLine()
{
    gzdbg<<"lifting line plugin destructed\n";
}

/////////////////////////////////////////////////
void GazeboLiftingLine::Load(physics::ModelPtr _model,
                              sdf::ElementPtr _sdf)
{
    gzdbg<<"load called"<<std::endl;

    GZ_ASSERT(_model, "GazeboLiftingLine _model pointer is NULL");
    GZ_ASSERT(_sdf, "GazeboLiftingLine _sdf pointer is NULL");
    this->model = _model;
    this->sdf = _sdf;
    
    this->world = this->model->GetWorld();
    GZ_ASSERT(this->world, "GazeboLiftingLine world pointer is NULL");
    
#if GAZEBO_MAJOR_VERSION >= 9
    this->physics = this->world->Physics();
#else
    this->physics = this->world->GetPhysicsEngine();
#endif
    GZ_ASSERT(this->physics, "GazeboLiftingLine physics pointer is NULL");
    
    GZ_ASSERT(_sdf, "GazeboLiftingLine _sdf pointer is NULL");
    
    namespace_.clear();
    
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboLiftingLine::OnUpdate, this));

    gzdbg<<"model name: "<<model->GetName()<<"\n";
    model->Print("");
    gzdbg<<"model child count: "<<model->GetChildCount()<<"\n";

    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";
}

/////////////////////////////////////////////////

void GazeboLiftingLine::OnUpdate()
{
    gzdbg<<"looping\n";
}



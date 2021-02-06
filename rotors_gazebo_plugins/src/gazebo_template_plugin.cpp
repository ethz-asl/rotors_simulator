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

#include "rotors_gazebo_plugins/gazebo_template_plugin.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(GazeboTemplate)

/////////////////////////////////////////////////
GazeboTemplate::GazeboTemplate()
{
    gzdbg<<"TemplatePlugin constructed\n";
}

/////////////////////////////////////////////////
GazeboTemplate::~GazeboTemplate()
{
    gzdbg<<"TemplatePlugin destructed\n";
}

/////////////////////////////////////////////////
void GazeboTemplate::Load(physics::ModelPtr _model,
                              sdf::ElementPtr _sdf)
{
    gzdbg<<"loading...\n";

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
    
    namespace_.clear();

    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init();
    
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboTemplate::OnUpdate, this));

    gzdbg<<"model name: "<<model_->GetName()<<"\n";
    model_->Print("");
    gzdbg<<"model child count: "<<model_->GetChildCount()<<"\n";

    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzerr << "Please specify a robotNamespace.\n";
}

/////////////////////////////////////////////////

void GazeboTemplate::OnUpdate()
{
    gzdbg<<"looping\n";
}



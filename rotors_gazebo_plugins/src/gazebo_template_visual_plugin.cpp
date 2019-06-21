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

#include "rotors_gazebo_plugins/gazebo_template_visual_plugin.h"

namespace gazebo{
namespace rendering{

GZ_REGISTER_VISUAL_PLUGIN(GazeboTemplateVisual)

/////////////////////////////////////////////////
GazeboTemplateVisual::GazeboTemplateVisual():
    line_(NULL)
{
    gzdbg<<"GazeboTemplateVisual plugin constructed\n";
}

/////////////////////////////////////////////////
GazeboTemplateVisual::~GazeboTemplateVisual()
{
    gzdbg<<"GazeboTemplateVisual plugin destructed\n";
}

/////////////////////////////////////////////////
void GazeboTemplateVisual::Load(VisualPtr _parent, sdf::ElementPtr _sdf)
{
    gzdbg<<"load called"<<std::endl;

    GZ_ASSERT(_parent, "GazeboTemplateVisual _parent pointer is NULL");
    GZ_ASSERT(_sdf, "GazeboTemplateVisual _sdf pointer is NULL");
    this->visual_ = _parent;

    namespace_.clear();

    /*
    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init();

    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzerr << "[GazeboTemplateVisual] Please specify a robotNamespace.\n";

    if (_sdf->HasElement("visualTopic"))
        namespace_ = _sdf->GetElement("visualTopic")->Get<std::string>();
    else
        gzerr << "[GazeboTemplateVisual] Please specify a visualTopic.\n";
    */

    /*
    visual_sub_ = node_handle_->Subscribe("~/" + namespace_ + "/" + visual_topic_,
                                          &GazeboTemplateVisual::VisualCallback,
                                          this);
    */

    this->update_connection_ = event::Events::ConnectPreRender(boost::bind(&GazeboTemplateVisual::OnUpdate, this));

    /*
    gzdbg<<"visual geometry: "<<visual_->GetGeometryType()<<"\n";
    visual_->GetScene()->PrintSceneGraph();
    visual_->SetVisible(false);
    */

    //this->scene_ = this->visual_->GetScene();
    //this->scene_->PrintSceneGraph();
    //this->visual_other_ = this->scene_->GetVisual("pylon7::pylon::v5");
    //this->visual_->SetTransparency(0.5);

    /*
    this->line_ = this->visual_->CreateDynamicLine(RENDERING_LINE_STRIP);
    this->line_->AddPoint(0,0,0);
    this->line_->AddPoint(2,2,2);
    this->line_->setMaterial("Gazebo/Purple");
    this->line_->setVisibilityFlags(GZ_VISIBILITY_GUI);
    this->line_->setVisible(true);
    */

    visual_->SetRibbonTrail(true,common::Color(1,0,0,0.5),common::Color(1,0,0,0.5));

}

/////////////////////////////////////////////////

void GazeboTemplateVisual::OnUpdate()
{

    ignition::math::Vector3d scale1 = ignition::math::Vector3d(1,1,1);
    ignition::math::Vector3d scale2 = ignition::math::Vector3d(2,2,2);

    if(blink_counter_%2000==0){
        //visual_->SetVisible(false);
        //visual_->SetAmbient(common::Color(1,0,0,1));
        //visual_other_->SetVisible(false);
        //this->line_->setVisible(false);
        //visual_->SetRibbonTrail(true,common::Color(1,0,0,0.5),common::Color(1,0,0,0.5));
        gzdbg<<"visual off \n";
    }

    if((blink_counter_+1000)%2000==0){
        //visual_->SetVisible(true);
        //visual_->SetAmbient(common::Color(0,1,0,1));
        //this->line_->setVisible(true);
        //visual_other_->SetVisible(true);
        //visual_->SetRibbonTrail(false,common::Color(1,0,0,0.5),common::Color(1,0,0,0.5));
        gzdbg<<"visual on\n";
    }


    ++blink_counter_;
}

void GazeboTemplateVisual::VisualCallback(const gz_visualization_msgs::VisVectorArray &visual_msg)
{

}

}
}





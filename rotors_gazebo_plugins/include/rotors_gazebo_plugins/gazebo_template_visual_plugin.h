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
*/
#ifndef _GAZEBO_TEMPLATE_VISUAL_PLUGIN_HH_
#define _GAZEBO_TEMPLATE_VISUAL_PLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/MessageTypes.hh"

#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/ros.h>

#include <geometry_msgs/Point.h>
// if you want some positions of the model use this....
#include <gazebo_msgs/ModelStates.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include "VisVectorArray.pb.h"

namespace gazebo
{
namespace rendering
{

typedef const boost::shared_ptr<const gz_visualization_msgs::VisVectorArray> GzVisVectorArrayMsgPtr;

/// \brief A template visual plugin
class GAZEBO_VISIBLE GazeboTemplateVisual : public VisualPlugin
{

  public:
    /// \brief Constructor.
    GazeboTemplateVisual();

    /// \brief Destructor.
    ~GazeboTemplateVisual();

    // Documentation Inherited.
    void Load(VisualPtr _parent, sdf::ElementPtr _sdf);

  protected:
    /// \brief Callback for World Update events.
    void OnUpdate();

    void VisualCallback(const gz_visualization_msgs::VisVectorArray &visual_msg);

  private:
    /// \brief Connection to World Update events.
    event::ConnectionPtr update_connection_;

    /// \brief The visual pointer used to visualize the force.
    VisualPtr visual_;
    VisualPtr visual_other_;

    /// \brief The scene pointer.
    ScenePtr scene_;

    /// \brief For example a line to visualize the force
    DynamicLines *line_;

    /// \brief Topic subscription
    transport::NodePtr node_handle_;
    std::string visual_topic_;
    transport::SubscriberPtr visual_sub_;
    std::string namespace_;

    int blink_counter_ = 0;

};
}
}
#endif

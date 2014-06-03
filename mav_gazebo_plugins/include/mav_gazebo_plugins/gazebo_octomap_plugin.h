//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>
// All rights reserved.
//
// TODO(ff): Enter some license
//==============================================================================

#include <iostream>
#include <math.h>
#include <boost/shared_ptr.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/transport/transport.hh>
#include <sdf/sdf.hh>
#include <gazebo/msgs/msgs.hh>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

namespace gazebo
{
  class OctomapCreator : public WorldPlugin {
   public:
    /// \brief Constructor
    OctomapCreator();
    /// \brief Destructor
    virtual ~OctomapCreator();

   protected:
    /// \brief Load the plugin.
    /// \param[in] _parent Pointer to the world that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
    void Create(/*OctomapCreatorMsgPtr &msg*/);

   private:
    physics::WorldPtr world_;
    ros::NodeHandle node_handle_;
    ros::ServiceServer srv_;

    bool ServiceCallback(std_srvs::Empty::Request& req,
      std_srvs::Empty::Response& res);
  };

}

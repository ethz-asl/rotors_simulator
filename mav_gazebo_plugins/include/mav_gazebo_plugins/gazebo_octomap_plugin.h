//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>
// All rights reserved.
//
// TODO(ff): Enter some license
//==============================================================================

#include <iostream>
#include <math.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <octomap/octomap.h>
#include <planning_msgs/Octomap.h>

namespace gazebo {
class OctomapFromGazeboWorld : public WorldPlugin {
 public:
  /// \brief Constructor
  OctomapFromGazeboWorld();
  /// \brief Destructor
  virtual ~OctomapFromGazeboWorld();

 protected:
  /// \brief Load the plugin.
  /// \param[in] _parent Pointer to the world that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
  void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
  bool CheckIfInsideObject(const std::string& name, const math::Vector3& central_point, gazebo::physics::RayShapePtr ray);
  void CreateOctomap(const planning_msgs::Octomap::Request& msg);

 private:
  physics::WorldPtr world_;
  ros::NodeHandle node_handle_;
  ros::ServiceServer srv_;
  octomap::OcTree* octomap_;
  bool ServiceCallback(planning_msgs::Octomap::Request& req, planning_msgs::Octomap::Response& res);
};

}

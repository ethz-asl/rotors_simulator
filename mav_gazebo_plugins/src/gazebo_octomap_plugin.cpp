//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>
// All rights reserved.
//
// TODO(ff): Enter some license
//==============================================================================

#include <mav_gazebo_plugins/gazebo_octomap_plugin.h>


namespace gazebo
{
  OctomapCreator::OctomapCreator() : WorldPlugin(), node_handle_()  {}
  OctomapCreator::~OctomapCreator() {
    // if (node_handle_) {
    //   node_handle_->shutdown();
    //   delete node_handle_;
    // }
  }

  void OctomapCreator::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
    world_ = _parent;
    std::cout << "Subscribing to: " << "/octomap/command" << std::endl;
    srv_ = node_handle_.advertiseService(
      "/octomap/command", &OctomapCreator::ServiceCallback, this);
  }

  bool OctomapCreator::ServiceCallback(std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res) {
    Create();
    return true;
  }

  void OctomapCreator::Create(/*CollisionMapRequestPtr &msg*/) {
    gazebo::common::Time::Sleep(gazebo::common::Time(5));
    std::cout << "Received message" << std::endl;

    // math::Vector3 point_origin = msg->point_origin();
    // math::Vector3 bounding_box = msg->bounding_box();

    double dZ_vertical = 20;
    double mag_vertical = dZ_vertical;
    dZ_vertical = 0.1 * dZ_vertical / mag_vertical;
    double step_horizontal = 0.1;

    double dX_horizontal = 20;
    double dY_horizontal = 20;
    double mag_horizontal = sqrt(dX_horizontal * dX_horizontal + dY_horizontal * dY_horizontal);
    dX_horizontal = step_horizontal * dX_horizontal / mag_horizontal;
    dY_horizontal = step_horizontal * dY_horizontal / mag_horizontal;

    int count_vertical = mag_vertical / step_horizontal;
    int count_horizontal = mag_horizontal / step_horizontal;

    if (count_vertical == 0 || count_horizontal == 0)
    {
      std::cout << "Octomap has a zero dimension, check input msg." << std::endl;
      return;
    }

    double x, y, z;

    double dist;
    std::string entityName;
    math::Vector3 start, end;

    gazebo::physics::PhysicsEnginePtr engine = world_->GetPhysicsEngine();
    engine->InitForThread();
    gazebo::physics::RayShapePtr ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
      engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

    std::cout << "Rasterizing model and checking collisions" << std::endl;

    for (int i = 0; i < count_horizontal; ++i) {
      std::cout << "Percent complete: " << i * 100.0 / count_horizontal << std::endl;
      x = i * dX_horizontal + (-10);

      for (int j = 0; j < count_horizontal; ++j) {
        y = j * dY_horizontal + (-10);
        for (int k = 0; k < count_vertical; ++k) {
          z = k * dZ_vertical + (-5);
          start.x = x;
          end.x = x + dX_horizontal;

          start.y = end.y = y;
          start.z = end.z = z;
          ray->SetPoints(start, end);
          ray->GetIntersection(dist, entityName);
          // maybe use gazebo::physics::World::GetEntityBelowPoint(Vector3&)
          if (!entityName.empty()) {
            std::cout << "Found "<<entityName<<" at x="<<x<<" and y="<<y<<" and z="<<z<<" with dist="<<dist<<".\n";
          }
        }
      }
    }

    std::cout << "Publishing Octomap." << std::endl;
  }

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(OctomapCreator)
}

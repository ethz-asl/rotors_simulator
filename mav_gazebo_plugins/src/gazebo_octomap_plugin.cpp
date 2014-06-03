//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>
// All rights reserved.
//
// TODO(ff): Enter some license
//==============================================================================

#include <mav_gazebo_plugins/gazebo_octomap_plugin.h>
#include <gazebo/math/Vector3.hh>
#include <gazebo/common/Time.hh>


namespace gazebo
{
  OctomapCreator::OctomapCreator() : WorldPlugin(), node_handle_(), octomap_() {}
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

  bool OctomapCreator::ServiceCallback(planning_msgs::Octomap::Request& req,
      planning_msgs::Octomap::Response& res) {
    std::cout << "Creating octomap with origin at ("
      << req.bounding_box_origin.x << ", " << req.bounding_box_origin.y
      << ", " << req.bounding_box_origin.z << "), and bounding box lengths ("
      << req.bounding_box_lengths.x << ", " << req.bounding_box_lengths.y
      << ", " << req.bounding_box_lengths.z << "), and leaf size: "
      << req.leaf_size << ".\n";
    Create(req);
    if (req.filename != "") {
      std::cout << "Storing Octomap as: " << req.filename << "\n";
    }
    req = octomap_;
    return true;
  }

  void OctomapCreator::Create(const planning_msgs::Octomap::Request& msg) {
    // gazebo::common::Time::Sleep(gazebo::common::Time(5));
    std::cout << "Received message" << std::endl;

    math::Vector3 bounding_box_origin(
      msg.bounding_box_origin.x,
      msg.bounding_box_origin.y,
      msg.bounding_box_origin.z);
    math::Vector3 bounding_box_lengths(
      msg.bounding_box_lengths.x,
      msg.bounding_box_lengths.y,
      msg.bounding_box_lengths.z
    );
    double leaf_size = msg.leaf_size;

    int count_x = bounding_box_lengths.x / leaf_size;
    int count_y = bounding_box_lengths.y / leaf_size;
    int count_z = bounding_box_lengths.z / leaf_size;

    if (count_z == 0 || count_x == 0 || count_z == 0)
    {
      std::cout << "Octomap has a zero dimension, check input msg." << std::endl;
      return;
    }

    std::string entityName;
    math::Vector3 start, end;

    gazebo::physics::PhysicsEnginePtr engine = world_->GetPhysicsEngine();
    engine->InitForThread();
    gazebo::physics::RayShapePtr ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
      engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

    std::cout << "Rasterizing model and checking collisions" << std::endl;

    for (int i = 0; i < count_x; ++i) {
      std::cout << "Percent complete: " << i * 100.0 / count_x << std::endl;
      double x = i * leaf_size + (bounding_box_origin.x - bounding_box_lengths.x / 2);
      for (int j = 0; j < count_y; ++j) {
        double y = j * leaf_size
          + (bounding_box_origin.y - bounding_box_lengths.y / 2);
        for (int k = 0; k < count_z; ++k) {
          double z = k * leaf_size
            + (bounding_box_origin.z - bounding_box_lengths.z / 2);
          start.x = x;
          end.x = x + leaf_size;

          start.y = end.y = y;
          start.z = end.z = z;
          ray->SetPoints(start, end);
          double dist;
          std::string entity_name;
          ray->GetIntersection(dist, entity_name);
          // maybe use gazebo::physics::World::GetEntityBelowPoint(Vector3&)
          if (!entity_name.empty()) {
            std::cout << "Found "<<entity_name<<" at x="<<x<<" and y="<<y<<" and z="<<z<<" with dist="<<dist<<".\n";
          }
        }
      }
    }

    std::cout << "Publishing Octomap." << std::endl;
  }

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(OctomapCreator)
}

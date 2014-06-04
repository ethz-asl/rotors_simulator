//==============================================================================
// Copyright (c) 2014, Fadri Furrer <ffurrer@gmail.com>
// All rights reserved.
//
// TODO(ff): Enter some license
//==============================================================================

#include <mav_gazebo_plugins/gazebo_octomap_plugin.h>
#include <gazebo/math/Vector3.hh>
#include <gazebo/common/Time.hh>
#include <octomap_msgs/conversions.h>


namespace gazebo
{
  OctomapCreator::OctomapCreator() : WorldPlugin(), node_handle_(), octomap_(NULL) {}
  OctomapCreator::~OctomapCreator() {
    delete octomap_;
    octomap_ = NULL;
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
      if(octomap_) {
        std::string path = req.filename;
        octomap_->writeBinary(path);
        std::cout << std::endl << "Octree saved as " << path << std::endl;
      }
      else {
        std::cout<<"The octree is NULL. Will not save that."<<std::endl;
      }
    }
    common::Time now = world_->GetSimTime();
    res.map.header.frame_id = "world";
    res.map.header.stamp = ros::Time(now.sec, now.nsec);

    if (octomap_msgs::binaryMapToMsgData(*octomap_, res.map.data)){
      res.map.id = "OcTree";
      res.map.binary = true;
    }
    else {
      ROS_ERROR("Error serializing OctoMap");
    }
    return true;
  }

  void OctomapCreator::Create(const planning_msgs::Octomap::Request& msg) {
    double epsilon = 0.00001;
    int far_away = 100000;
    math::Vector3 bounding_box_origin(msg.bounding_box_origin.x,
                                      msg.bounding_box_origin.y,
                                      msg.bounding_box_origin.z);
    math::Vector3 bounding_box_lengths(msg.bounding_box_lengths.x + epsilon,
                                       msg.bounding_box_lengths.y + epsilon,
                                       msg.bounding_box_lengths.z + epsilon);
    double leaf_size = msg.leaf_size;
    octomap_ =  new octomap::OcTree(leaf_size);
    octomap_->clear();
    octomap_->setProbHit(0.7);
    octomap_->setProbMiss(0.4);
    octomap_->setClampingThresMin(0.12);
    octomap_->setClampingThresMax(0.97);
    octomap_->setOccupancyThres(0.7);

    int count_x = bounding_box_lengths.x / leaf_size;
    int count_y = bounding_box_lengths.y / leaf_size;
    int count_z = bounding_box_lengths.z / leaf_size;

    if (count_x == 0 || count_y == 0 || count_z == 0)
    {
      std::cout << "Octomap has a zero dimension, check input msg." << std::endl;
      return;
    }

    gazebo::physics::PhysicsEnginePtr engine = world_->GetPhysicsEngine();
    engine->InitForThread();
    gazebo::physics::RayShapePtr ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
      engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

    std::cout << "Rasterizing world and checking collisions" << std::endl;
    bool in_object = false;
    for (int i = 0; i < count_z; ++i) {
      std::cout << "Percent complete: " << i * 100.0 / count_z << std::endl;
      double z = i * leaf_size + (bounding_box_origin.z - bounding_box_lengths.z / 2);
      for (int j = 0; j < count_y; ++j) {
        double y = j * leaf_size + (bounding_box_origin.y - bounding_box_lengths.y / 2);

        std::string entity_name, entity_name_backwards;
        double dist, x;
        math::Vector3 start, end;

        start.y = end.y = y;
        start.z = end.z = z;

        // Set the start point of the ray to the beginning of the bounding box
        start.x = bounding_box_origin.x - bounding_box_lengths.x / 2 - leaf_size / 2;
        x = start.x + leaf_size / 2;

        // Set the end of the ray to some large value behind itself to check
        // if there is a boundary of an object at the current y-z location in -x
        end.x = - far_away;
        ray->SetPoints(start, end);

        // Check if there is an intersection in the back
        ray->GetIntersection(dist, entity_name_backwards);

        // Set the end of the ray to somewhere far away
        end.x = far_away;
        ray->SetPoints(start, end);

        // Check if there is an intersection in the front
        ray->GetIntersection(dist, entity_name);

        // Set in_object to true, if currently in an object
        if (!entity_name_backwards.empty()
            && entity_name == entity_name_backwards) {
          in_object = true;
        }
        else {
          in_object = false;
        }

        // Set the end of the ray to the end of the bounding box
        end.x = bounding_box_origin.x + bounding_box_lengths.x / 2 +
                leaf_size / 2;

        ray->SetPoints(start, end);

        // Check if there is an object at the current location
        ray->GetIntersection(dist, entity_name);

        while (!entity_name.empty() && start.x < end.x) {
          while(x < start.x + dist && x < end.x - leaf_size / 2) {
            if (in_object) {
              octomap_->setNodeValue(x, y, z, 1);
            }
            else {
              octomap_->setNodeValue(x, y, z, 0);
            }
            x += leaf_size;
          }
          // Set the new starting point just after the last intersection
          start.x += dist + epsilon;
          ray->SetPoints(start, end);
          ray->GetIntersection(dist, entity_name);
          if (in_object) {
            in_object = false;
          }
          else {
            in_object = true;
          }
        }
        // Loop until the end of the bounding box and fill the leafs
        while(x < end.x - leaf_size / 2) {
          if (in_object) {
            octomap_->setNodeValue(x, y, z, 1);
          }
          else {
            octomap_->setNodeValue(x, y, z, 0);
          }
          x += leaf_size;
        }
      }
    }
    octomap_->prune();
    octomap_->updateInnerOccupancy();

    std::cout << "Publishing Octomap." << std::endl;
  }

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(OctomapCreator)
}

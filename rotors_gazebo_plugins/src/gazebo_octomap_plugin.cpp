/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "rotors_gazebo_plugins/gazebo_octomap_plugin.h"

#include <gazebo/common/Time.hh>
#include <gazebo/math/Vector3.hh>
#include <octomap_msgs/conversions.h>

namespace gazebo {

OctomapFromGazeboWorld::~OctomapFromGazeboWorld() {
  delete octomap_;
  octomap_ = NULL;
}

void OctomapFromGazeboWorld::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
  world_ = _parent;
  std::string service_name = "world/get_octomap";
  gzlog << "Advertising service: " << service_name << std::endl;
  srv_ = node_handle_.advertiseService(service_name, &OctomapFromGazeboWorld::ServiceCallback, this);
}

bool OctomapFromGazeboWorld::ServiceCallback(planning_msgs::Octomap::Request& req,
                                             planning_msgs::Octomap::Response& res) {
  std::cout << "Creating octomap with origin at (" << req.bounding_box_origin.x << ", " << req.bounding_box_origin.y
            << ", " << req.bounding_box_origin.z << "), and bounding box lengths (" << req.bounding_box_lengths.x
            << ", " << req.bounding_box_lengths.y << ", " << req.bounding_box_lengths.z << "), and leaf size: "
            << req.leaf_size << ".\n";
  CreateOctomap(req);
  if (req.filename != "") {
    if (octomap_) {
      std::string path = req.filename;
      octomap_->writeBinary(path);
      std::cout << std::endl << "Octree saved as " << path << std::endl;
    }
    else {
      std::cout << "The octree is NULL. Will not save that." << std::endl;
    }
  }
  common::Time now = world_->GetSimTime();
  res.map.header.frame_id = "world";
  res.map.header.stamp = ros::Time(now.sec, now.nsec);

  if (octomap_msgs::binaryMapToMsgData(*octomap_, res.map.data)) {
    res.map.id = "OcTree";
    res.map.binary = true;
    res.map.resolution = octomap_->getResolution();
  }
  else {
    ROS_ERROR("Error serializing OctoMap");
  }
  std::cout << "Publishing Octomap." << std::endl;
  return true;
}

bool OctomapFromGazeboWorld::CheckIfInsideObject(const std::string& name, const math::Vector3& central_point,
                                                 gazebo::physics::RayShapePtr ray) {
  const double epsilon = 0.00001;
  const int far_away = 100000;
  double dist;
  std::string entity_name;
  // Set the end of the ray to somewhere far away

  math::Vector3 start_point = central_point;
  math::Vector3 end_point = central_point;

  end_point.x = far_away;

  ray->SetPoints(start_point, end_point);

  // Check if there is an intersection in the front
  ray->GetIntersection(dist, entity_name);

  int counter = 0;

  while (entity_name != name && start_point.x < far_away && counter < 6) {
    start_point.x += dist + epsilon;
    ray->SetPoints(start_point, end_point);
    // Check if there is an intersection in the front
    ray->GetIntersection(dist, entity_name);
    ++counter;
  }

  counter = 0;
  start_point = central_point;
  end_point = central_point;
  end_point.y = far_away;

  while (entity_name != name && start_point.y < far_away && counter < 6) {
    start_point.y += dist + epsilon;
    ray->SetPoints(start_point, end_point);
    // Check if there is an intersection in the front
    ray->GetIntersection(dist, entity_name);
    ++counter;
  }

  counter = 0;
  start_point = central_point;
  end_point = central_point;
  end_point.z = far_away;

  while (entity_name != name && start_point.z < far_away && counter < 6) {
    start_point.z += dist + epsilon;
    ray->SetPoints(start_point, end_point);
    // Check if there is an intersection in the front
    ray->GetIntersection(dist, entity_name);
    ++counter;
  }

  if (entity_name == name)
    return true;

  return false;
}

bool OctomapFromGazeboWorld::CheckIfInsideObjectInX(const std::string& name, const math::Vector3& central_point,
                                                    gazebo::physics::RayShapePtr ray) {
  const double epsilon = 0.00001;
  const int far_away = 100000;
  double dist;
  std::string entity_name;
  // Set the end of the ray to somewhere far away

  math::Vector3 start_point = central_point;
  math::Vector3 end_point = central_point;

  end_point.x = far_away;

  ray->SetPoints(start_point, end_point);

  // Check if there is an intersection in the front
  ray->GetIntersection(dist, entity_name);

  int counter = 0;

  while (entity_name != name && start_point.x < far_away && counter < 6) {
    start_point.x += dist + epsilon;
    ray->SetPoints(start_point, end_point);
    // Check if there is an intersection in the front
    ray->GetIntersection(dist, entity_name);
    ++counter;
  }

  if (entity_name == name)
    return true;

  return false;
}

void OctomapFromGazeboWorld::CreateOctomap(const planning_msgs::Octomap::Request& msg) {
  const double epsilon = 0.00001;
  const int far_away = 100000;
  math::Vector3 bounding_box_origin(msg.bounding_box_origin.x, msg.bounding_box_origin.y, msg.bounding_box_origin.z);
  // add epsilon to the box, because octomap has undefined behaviour if the
  // point you want to insert is exactly between two leafs (doesn't look nice :).
  math::Vector3 bounding_box_lengths(msg.bounding_box_lengths.x + epsilon, msg.bounding_box_lengths.y + epsilon,
                                     msg.bounding_box_lengths.z + epsilon);
  double leaf_size = msg.leaf_size;
  octomap_ = new octomap::OcTree(leaf_size);
  octomap_->clear();
  octomap_->setProbHit(0.7);
  octomap_->setProbMiss(0.4);
  octomap_->setClampingThresMin(0.12);
  octomap_->setClampingThresMax(0.97);
  octomap_->setOccupancyThres(0.7);

  int count_x = bounding_box_lengths.x / leaf_size;
  int count_y = bounding_box_lengths.y / leaf_size;
  int count_z = bounding_box_lengths.z / leaf_size;

  if (count_x == 0 || count_y == 0 || count_z == 0) {
    std::cout << "Octomap has a zero dimension, check input msg." << std::endl;
    return;
  }

  gazebo::physics::PhysicsEnginePtr engine = world_->GetPhysicsEngine();
  engine->InitForThread();
  gazebo::physics::RayShapePtr ray = boost::dynamic_pointer_cast < gazebo::physics::RayShape
      > (engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

  std::cout << "Rasterizing world and checking collisions" << std::endl;
  bool in_object = false;
  for (int i = 0; i < count_z; ++i) {
    std::cout << "Percent complete: " << i * 100.0 / count_z << std::endl;
    double z = i * leaf_size + (bounding_box_origin.z - bounding_box_lengths.z / 2);
    for (int j = 0; j < count_y; ++j) {
      double y = j * leaf_size + (bounding_box_origin.y - bounding_box_lengths.y / 2);

      std::map<std::string, bool> objects_in_collision;

      std::string entity_name, entity_name_backwards;
      double dist, x;
      math::Vector3 start, end, start_prev;

      start.y = end.y = y;
      start.z = end.z = z;

      // Set the start point of the ray to the beginning of the bounding box
      start.x = bounding_box_origin.x - bounding_box_lengths.x / 2 - leaf_size / 2;
      x = start.x + leaf_size / 2;

      // Set the end of the ray to some large value behind itself to check
      // if there is a boundary of an object at the current y-z location in -x
      end.x = -far_away;
      ray->SetPoints(start, end);

      // Check if there is an intersection in the back
      ray->GetIntersection(dist, entity_name_backwards);

      bool inside_object = CheckIfInsideObject(entity_name_backwards, start, ray);

      // Set in_object to true, if currently in an object
      if (!entity_name_backwards.empty() && inside_object) {
        objects_in_collision.insert(std::pair<std::string, bool>(entity_name_backwards, true));
      }

      // Set the end of the ray to the end of the bounding box
      end.x = bounding_box_origin.x + bounding_box_lengths.x / 2 + leaf_size / 2;

      ray->SetPoints(start, end);

      // Check if there is an object at the current location
      ray->GetIntersection(dist, entity_name);

      while (!entity_name.empty() && start.x < end.x) {

        while (x < start.x + dist && x < end.x - leaf_size / 2) {
          if (!objects_in_collision.empty()) {
            octomap_->setNodeValue(x, y, z, 1);
          } else {
            octomap_->setNodeValue(x, y, z, 0);
          }
          x += leaf_size;
        }

        std::map<std::string, bool>::iterator it = objects_in_collision.find(entity_name);
        if (it != objects_in_collision.end()) {
          objects_in_collision.erase(it);
        } else {
          objects_in_collision.insert(std::pair<std::string, bool>(entity_name, true));
        }

        start_prev = start;

        // Set the new starting point just after the last intersection
        start.x += dist + epsilon;

        start_prev.x = start.x - epsilon;

        // search back
        ray->SetPoints(start, start_prev);
        ray->GetIntersection(dist, entity_name_backwards);
        if (!entity_name_backwards.empty() && entity_name != entity_name_backwards) {
          std::map<std::string, bool>::iterator it = objects_in_collision.find(entity_name_backwards);
          if (it != objects_in_collision.end()) {
            std::cout << "NOT THE SAME ELEMENT ERASE" << std::endl;

            objects_in_collision.erase(it);
          } else {
            std::cout << "NOT THE SAME ELEMENT ADD" << std::endl;

            objects_in_collision.insert(std::pair<std::string, bool>(entity_name_backwards, true));
          }
        }

//        // garbage collector
//        for(std::map<std::string,bool>::iterator it = objects_in_collision.begin(); it != objects_in_collision.end(); it++) {
//            if(!CheckIfInsideObjectInX(it->first,start,ray)){
//              objects_in_collision.erase(it);
//            }
//        }
        ray->SetPoints(start, end);
        ray->GetIntersection(dist, entity_name);
      }

      math::Vector3 end_new = end;
      end_new.x -= (leaf_size / 2 + epsilon);

      start.x = x;

      // garbage collector
      for (std::map<std::string, bool>::iterator it = objects_in_collision.begin();
           it != objects_in_collision.end(); it++) {
        if (!CheckIfInsideObjectInX(it->first, start, ray)) {
          objects_in_collision.erase(it);
        }
      }

      // Loop until the end of the bounding box and fill the leafs
      while (x < end.x - leaf_size / 2) {
        if (!objects_in_collision.empty()) {
          octomap_->setNodeValue(x, y, z, 1);
        } else {
          octomap_->setNodeValue(x, y, z, 0);
        }
        x += leaf_size;
      }
    }
  }
  octomap_->prune();
  octomap_->updateInnerOccupancy();
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(OctomapFromGazeboWorld)
}

/*
  Copyright (C) 2018~ Nearthlab Inc.

 */

#ifndef _GAZEBO_FRONT_GIMBAL_PLUGIN_HH_
#define _GAZEBO_FRONT_GIMBAL_PLUGIN_HH_

#include <string>
#include <vector>

#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/util/system.hh>
#include <gazebo/sensors/sensors.hh>

#include "rotors_gazebo_plugins/common.h"

#include <mav_msgs/default_topics.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"

namespace gazebo
{
  class FrontGimbalPlugin : public ModelPlugin
  {    
    public: 
        FrontGimbalPlugin()
            :ModelPlugin() {}
        virtual ~FrontGimbalPlugin();

    protected:
        void Load(physics::ModelPtr model, sdf::ElementPtr sdf); 
        void OnUpdate(const common::UpdateInfo& /*_info*/);

    private: 
        sdf::ElementPtr _sdf;
        std::vector<event::ConnectionPtr> _connections;

    private: 
        transport::SubscriberPtr _subPitch;
        transport::SubscriberPtr _subRoll;
        transport::NodePtr       _node;

    private: 
        transport::PublisherPtr pitchPub;
        transport::PublisherPtr rollPub;
        transport::PublisherPtr yawPub;

    private:
        physics::ModelPtr _model;
        physics::JointPtr _jointRoll;
        physics::JointPtr _jointPitch;

    private:
        double _targetRoll;
        double _targetPitch;
        event::ConnectionPtr _update_connection;

    private:
        common::PID _pidRoll;
        common::PID _pidPitch;

    private:    
        ros::NodeHandle* _nodeRos;
        ros::Subscriber  _subCommand;
        void CreateCommunicationChannels();        
  };
}
#endif

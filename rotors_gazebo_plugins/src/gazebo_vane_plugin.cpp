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

#include "rotors_gazebo_plugins/gazebo_vane_plugin.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(GazeboVanePlugin)

/////////////////////////////////////////////////
GazeboVanePlugin::GazeboVanePlugin()
{
    gzdbg<<"GazeboVanePlugin constructed\n";
}

/////////////////////////////////////////////////
GazeboVanePlugin::~GazeboVanePlugin()
{
    gzdbg<<"GazeboVanePlugin destructed\n";
}

/////////////////////////////////////////////////
void GazeboVanePlugin::Load(physics::ModelPtr _model,
                              sdf::ElementPtr _sdf)
{
    gzdbg<<"loading... \n";

    GZ_ASSERT(_model, "GazeboVanePlugin _model pointer is NULL");
    GZ_ASSERT(_sdf, "GazeboVanePlugin _sdf pointer is NULL");
    this->model_ = _model;
    this->sdf_ = _sdf;
    
    this->world_ = this->model_->GetWorld();
    GZ_ASSERT(this->world_, "GazeboVanePlugin world pointer is NULL");
    
    namespace_.clear();

    node_handle_ = gazebo::transport::NodePtr(new transport::Node());
    node_handle_->Init();
    
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboVanePlugin::OnUpdate, this));

    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzerr << "Please specify a robotNamespace.\n";

    joint_name_;
    if (_sdf->HasElement("vaneJoint")) {
        joint_name_ = _sdf->Get<std::string>("vaneJoint");
        gzdbg<<"jointName "<<joint_name_<<"\n";
        this->vane_joint_ = this->model_->GetJoint(joint_name_);
    }

    if (!this->vane_joint_){
        gzerr<<"Joint "<<joint_name_<<" not found \n";
    }

    if (_sdf->HasElement("vaneTopic")) {
        vane_topic_ = _sdf->Get<std::string>("vaneTopic");
        gzdbg<<"vaneTopic "<<vane_topic_<<"\n";
    }
}

/////////////////////////////////////////////////
void GazeboVanePlugin::OnUpdate() {

    if (!pubs_and_subs_created_) {
      CreatePubsAndSubs();
      pubs_and_subs_created_ = true;
    }

    common::Time current_time = world_->SimTime();

    vane_msg_.mutable_header()->mutable_stamp()->set_sec(current_time.sec);
    vane_msg_.mutable_header()->mutable_stamp()->set_nsec(current_time.nsec);
    vane_msg_.mutable_header()->set_frame_id("");
    vane_msg_.clear_name();
    vane_msg_.clear_position();

    vane_msg_.add_name(joint_name_);
    vane_msg_.add_position(vane_joint_->Position());

    vane_pub_->Publish(vane_msg_);

}

/////////////////////////////////////////////////
void GazeboVanePlugin::CreatePubsAndSubs(){
    // Create temporary "ConnectGazeboToRosTopic" publisher and message

      gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
          node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
              "~/" + kConnectGazeboToRosSubtopic, 1);

      // ============================================ //
      // =============== Vane MSG SETUP ============= //
      // ============================================ //
      gzdbg<<"advertised  ~/" + namespace_ + "/" + vane_topic_ + " gazebo message.\n";
      vane_pub_ = node_handle_->Advertise<gz_sensor_msgs::JointState>(
          "~/" + namespace_ + "/" + vane_topic_, 1);

      gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;
      connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                       vane_topic_);
      connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" + vane_topic_);
      connect_gazebo_to_ros_topic_msg.set_msgtype(gz_std_msgs::ConnectGazeboToRosTopic::JOINT_STATE);
      connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg, true);

}

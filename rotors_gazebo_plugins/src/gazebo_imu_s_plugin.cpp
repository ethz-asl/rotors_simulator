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

#include "rotors_gazebo_plugins/gazebo_imu_s_plugin.h"

using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(GazeboImuSPlugin)

/////////////////////////////////////////////////
GazeboImuSPlugin::GazeboImuSPlugin()
{
    gzdbg<<"GazeboImuSPlugin constructed\n";
}

/////////////////////////////////////////////////
GazeboImuSPlugin::~GazeboImuSPlugin()
{
    gzdbg<<"GazeboImuSPlugin destructed\n";
}

/////////////////////////////////////////////////
void GazeboImuSPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    gzdbg<<"load called"<<std::endl;

    GZ_ASSERT(_sensor, "GazeboImuSPlugin _sensor pointer is NULL");
    GZ_ASSERT(_sdf, "GazeboImuSPlugin _sdf pointer is NULL");

    // Store the pointer to the parent sensor
    parent_sensor_ = std::dynamic_pointer_cast<sensors::ImuSensor>(_sensor);
    
    world_ = physics::get_world(parent_sensor_->WorldName());
    GZ_ASSERT(world_, "GazeboImuSPlugin world pointer is NULL");

#if GAZEBO_MAJOR_VERSION >= 9
    physics_ = world_->Physics();
#else
    physics_ = world_->GetPhysicsEngine();
#endif
    GZ_ASSERT(physics_, "GazeboImuSPlugin physics pointer is NULL");
    
    namespace_.clear();

    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzerr << "Please specify a robotNamespace.\n";

    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init();

    std::string link_name;
    if (_sdf->HasElement("linkName"))
        link_name = _sdf->GetElement("linkName")->Get<std::string>();
    else
        gzerr << "Please specify a linkName.\n";

    // Fill IMU message (static part)
    imu_message_.mutable_header()->set_frame_id(link_name);

    // Get the pointer to the link that holds the sensor.
    link_ = boost::dynamic_pointer_cast<physics::Link>(world_->EntityByName(link_name));
    if (link_ == NULL)
        gzerr << "Couldn't find specified link \"" << link_name
              << "\"\n";

    if (_sdf->HasElement("imuTopic"))
        imu_topic_ = _sdf->GetElement("imuTopic")->Get<std::string>();
    else
        gzerr << "Please specify imuTopic.\n";

    // Connect to the sensor update event.
    this->updateConnection_ = this->parent_sensor_->ConnectUpdated(
                boost::bind(&GazeboImuSPlugin::OnUpdate, this));

    // Make sure the parent sensor is active.
    parent_sensor_->SetActive(true);
}

/////////////////////////////////////////////////
void GazeboImuSPlugin::OnUpdate()
{
    if (!pubs_and_subs_created_) {
      CreatePubsAndSubs();
      pubs_and_subs_created_ = true;
    }

    ignition::math::Vector3d lin_acc = parent_sensor_->LinearAcceleration(true);
    ignition::math::Vector3d ang_vel = parent_sensor_->AngularVelocity(true);

    common::Time current_time = world_->SimTime();
    double dt = (current_time - last_time_).Double();
    last_time_ = current_time;

    ignition::math::Pose3d T_W_I = link_->WorldPose();
    ignition::math::Quaterniond C_W_I = T_W_I.Rot();

    // Fill IMU message.
    imu_message_.mutable_header()->mutable_stamp()->set_sec(current_time.sec);
    imu_message_.mutable_header()->mutable_stamp()->set_nsec(current_time.nsec);

    imu_message_.mutable_linear_acceleration()->set_x(lin_acc.X());
    imu_message_.mutable_linear_acceleration()->set_y(lin_acc.Y());
    imu_message_.mutable_linear_acceleration()->set_z(lin_acc.Z());

    imu_message_.mutable_angular_velocity()->set_x(ang_vel.X());
    imu_message_.mutable_angular_velocity()->set_y(ang_vel.Y());
    imu_message_.mutable_angular_velocity()->set_z(ang_vel.Z());

    imu_message_.mutable_orientation()->set_w(C_W_I.W());
    imu_message_.mutable_orientation()->set_x(C_W_I.X());
    imu_message_.mutable_orientation()->set_y(C_W_I.Y());
    imu_message_.mutable_orientation()->set_z(C_W_I.Z());

    imu_message_.set_seq(seq_++);

    // Publish the IMU message
    imu_pub_->Publish(imu_message_);
}

/////////////////////////////////////////////////
void GazeboImuSPlugin::CreatePubsAndSubs()
{
    // Create temporary "ConnectGazeboToRosTopic" publisher and message
    gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
        node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
            "~/" + kConnectGazeboToRosSubtopic, 1);

    // ============================================ //
    // =============== IMU MSG SETUP ============== //
    // ============================================ //

    imu_pub_ = node_handle_->Advertise<gz_sensor_msgs::Imu>(
        "~/" + namespace_ + "/" + imu_topic_, 1);
    gzdbg<<"advertised  ~/" + namespace_ + "/" + imu_topic_ + " gazebo message."<<std::endl;

    gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;
    connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                     imu_topic_);
    connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" + imu_topic_);
    connect_gazebo_to_ros_topic_msg.set_msgtype(gz_std_msgs::ConnectGazeboToRosTopic::IMU);
    connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg, true);
}



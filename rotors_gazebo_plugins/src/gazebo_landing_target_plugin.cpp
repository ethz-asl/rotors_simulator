/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
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

// MODULE HEADER INCLUDE
#include "rotors_gazebo_plugins/gazebo_landing_target_plugin.h"

// STANDARD LIB INCLUDES
#include <ctime>
#include <cmath>

#include "ConnectGazeboToRosTopic.pb.h"

#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

namespace gazebo
{

GazeboLandingTargetPlugin::~GazeboLandingTargetPlugin()
{
    delete ros_node_handle_;
}

void GazeboLandingTargetPlugin::Load(physics::ModelPtr _model,
                                     sdf::ElementPtr _sdf)
{
    if (kPrintOnPluginLoad)
    {
        gzdbg << __FUNCTION__ << "() called." << std::endl;
    }

    model_ = _model;
    world_ = model_->GetWorld();
    namespace_.clear();

    getSdfParam<std::string>(_sdf, "robotNamespace", namespace_, namespace_,
                             true);
    getSdfParam<std::string>(_sdf, "linkName", link_name_, link_name_, true);

    node_handle_ = gazebo::transport::NodePtr(new transport::Node());

    // Initialise with default namespace (typically /gazebo/default/)
    node_handle_->Init();

    frame_id_ = link_name_;

    link_ = model_->GetLink(link_name_);
    if (link_ == NULL)
        gzthrow("[gazebo_multirotor_base_plugin] Couldn't find specified link \""
                << link_name_ << "\".");

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GazeboLandingTargetPlugin::OnUpdate, this, _1));
}

// This gets called by the world update start event.
void GazeboLandingTargetPlugin::OnUpdate(const common::UpdateInfo &_info)
{
    if (kPrintOnUpdates)
    {
        gzdbg << __FUNCTION__ << "() called." << std::endl;
    }

    if (!pubs_and_subs_created_)
    {
        CreatePubsAndSubs();
        pubs_and_subs_created_ = true;
    }

    // Get the current simulation time.
    common::Time now = world_->GetSimTime();

    odom_msg_.mutable_header()->mutable_stamp()->set_sec(now.sec);
    odom_msg_.mutable_header()->mutable_stamp()->set_nsec(now.nsec);
    odom_msg_.mutable_header()->set_frame_id("world");
    odom_msg_.set_child_frame_id("target/base_link");

    // Calculate odometry
    const math::Pose T_WR = model_->GetWorldPose();
    const math::Vector3 r_WR = T_WR.pos;
    const math::Quaternion q_WR = T_WR.rot;
    const math::Vector3 linear_velocity = model_->GetWorldLinearVel();
    const math::Vector3 angular_velocity = model_->GetWorldAngularVel();
    odom_msg_.mutable_pose()->mutable_pose()->mutable_position()->set_x(r_WR.x);
    odom_msg_.mutable_pose()->mutable_pose()->mutable_position()->set_y(r_WR.y);
    odom_msg_.mutable_pose()->mutable_pose()->mutable_position()->set_z(r_WR.z);
    odom_msg_.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(q_WR.w);
    odom_msg_.mutable_pose()->mutable_pose()->mutable_orientation()->set_x(q_WR.x);
    odom_msg_.mutable_pose()->mutable_pose()->mutable_orientation()->set_y(q_WR.y);
    odom_msg_.mutable_pose()->mutable_pose()->mutable_orientation()->set_z(q_WR.z);
    odom_msg_.mutable_pose()->mutable_covariance()->Clear();
    odom_msg_.mutable_twist()->mutable_twist()->mutable_linear()->set_x(linear_velocity.x);
    odom_msg_.mutable_twist()->mutable_twist()->mutable_linear()->set_y(linear_velocity.y);
    odom_msg_.mutable_twist()->mutable_twist()->mutable_linear()->set_z(linear_velocity.z);
    odom_msg_.mutable_twist()->mutable_twist()->mutable_angular()->set_x(angular_velocity.x);
    odom_msg_.mutable_twist()->mutable_twist()->mutable_angular()->set_y(angular_velocity.y);
    odom_msg_.mutable_twist()->mutable_twist()->mutable_angular()->set_z(angular_velocity.z);
    odom_msg_.mutable_twist()->mutable_covariance()->Clear();
    for (int i = 0; i < 36; i++)
    {
        odom_msg_.mutable_pose()->mutable_covariance()->Add(0.);
        odom_msg_.mutable_twist()->mutable_covariance()->Add(0.);
    }
    // Set joint state
    joint_state_msg_.mutable_header()->mutable_stamp()->set_sec(now.sec);
    joint_state_msg_.mutable_header()->mutable_stamp()->set_nsec(now.nsec);
    joint_state_msg_.mutable_header()->set_frame_id(frame_id_);

    // Broadcast TF
    gazebo::msgs::Vector3d *p =
        odom_msg_.mutable_pose()->mutable_pose()->mutable_position();
    gazebo::msgs::Quaternion *q_W_L =
        odom_msg_.mutable_pose()->mutable_pose()->mutable_orientation();
    gz_geometry_msgs::TransformStampedWithFrameIds
        transform_stamped_with_frame_ids_msg;
    transform_stamped_with_frame_ids_msg.mutable_header()->CopyFrom(
        odom_msg_.header());
    transform_stamped_with_frame_ids_msg.mutable_transform()
        ->mutable_translation()
        ->set_x(p->x());
    transform_stamped_with_frame_ids_msg.mutable_transform()
        ->mutable_translation()
        ->set_y(p->y());
    transform_stamped_with_frame_ids_msg.mutable_transform()
        ->mutable_translation()
        ->set_z(p->z());
    transform_stamped_with_frame_ids_msg.mutable_transform()
        ->mutable_rotation()
        ->CopyFrom(*q_W_L);
    transform_stamped_with_frame_ids_msg.set_parent_frame_id("world");
    transform_stamped_with_frame_ids_msg.set_child_frame_id("target/base_link");

    joint_state_pub_->Publish(joint_state_msg_);
    odom_pub_->Publish(odom_msg_);
    broadcast_transform_pub_->Publish(transform_stamped_with_frame_ids_msg);
}

void GazeboLandingTargetPlugin::CreatePubsAndSubs()
{
    // Create temporary "ConnectGazeboToRosTopic" publisher and message
    gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
        node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
            "~/" + kConnectGazeboToRosSubtopic, 1);

    gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;

    // =========================================== //
    // =========== ODOMETRY MSG SETUP ============ //
    // =========================================== //
    odom_pub_ = node_handle_->Advertise<gz_geometry_msgs::Odometry>(
        "~/" + namespace_ + "/" + "odometry", 10);

    // connect_gazebo_to_ros_topic_msg.set_gazebo_namespace(namespace_);
    connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + model_->GetName() +
                                                     "/" + "odometry");
    connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                  "odometry");
    connect_gazebo_to_ros_topic_msg.set_msgtype(
        gz_std_msgs::ConnectGazeboToRosTopic::ODOMETRY);
    connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                             true);

    // ============================================ //
    // ========== JOINT STATE MSG SETUP =========== //
    // ============================================ //
    joint_state_pub_ = node_handle_->Advertise<gz_sensor_msgs::JointState>(
        "~/" + namespace_ + "/" + joint_state_pub_topic_, 1);

    // connect_gazebo_to_ros_topic_msg.set_gazebo_namespace(namespace_);
    connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                     joint_state_pub_topic_);
    connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                  joint_state_pub_topic_);
    connect_gazebo_to_ros_topic_msg.set_msgtype(
        gz_std_msgs::ConnectGazeboToRosTopic::JOINT_STATE);
    connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                             true);

    // ========================================== //
    // ===== BROADCAST TRANSFORM MSG SETUP =====  //
    // ========================================== //
    broadcast_transform_pub_ =
        node_handle_->Advertise<gz_geometry_msgs::TransformStampedWithFrameIds>(
            "~/" + kBroadcastTransformSubtopic, 1);

    ros_node_handle_ = new ros::NodeHandle();
    std::string twist_topic_name = namespace_ + "/" + "set_twist";
    twist_sub_ = ros_node_handle_->subscribe(twist_topic_name, 1, &GazeboLandingTargetPlugin::TwistCallback, this);
    std::string pose_topic_name = namespace_ + "/" + "set_pose";
    pose_sub_ = ros_node_handle_->subscribe(pose_topic_name, 1, &GazeboLandingTargetPlugin::PoseCallback, this);
}

void GazeboLandingTargetPlugin::PoseCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
    math::Pose T_WR = model_->GetWorldPose();
    math::Vector3 pos{msg->position.x, msg->position.y, msg->position.z};
    math::Quaternion rot{msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z};
    T_WR.pos = pos;
    T_WR.rot = rot;
    model_->SetWorldPose(T_WR);
}

void GazeboLandingTargetPlugin::TwistCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    math::Pose T_WR = model_->GetWorldPose();
    math::Quaternion rot = T_WR.rot;
    math::Vector3 ang = rot.GetAsEuler();

    double vel_x = msg->linear.x * cos(ang.z);
    double vel_y = msg->linear.x * sin(ang.z);
    double vel_th = msg->angular.z;

    math::Vector3 lin_cmd{vel_x, vel_y, 0.};
    math::Vector3 ang_cmd{0., 0., vel_th};

    model_->SetLinearVel(lin_cmd);
    model_->SetAngularVel(ang_cmd);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboLandingTargetPlugin);

} // namespace gazebo

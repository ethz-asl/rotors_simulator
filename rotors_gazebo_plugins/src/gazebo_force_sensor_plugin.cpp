/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Simone Comari, ASL, ETH Zurich, Switzerland
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

#include "../include/rotors_gazebo_plugins/gazebo_force_sensor_plugin.h"

#include <chrono>
#include <iostream>


namespace gazebo {

GazeboForceSensorPlugin::~GazeboForceSensorPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

// void GazeboForceSensorPlugin::InitializeParams() {};
// void GazeboForceSensorPlugin::Publish() {};

void GazeboForceSensorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  sdf::Vector3 noise_normal_linear_force;
  sdf::Vector3 noise_normal_torque;
  sdf::Vector3 noise_uniform_linear_force;
  sdf::Vector3 noise_uniform_torque;
  const sdf::Vector3 zeros3(0.0, 0.0, 0.0);

  wrench_queue_.clear();

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_force_sensor_plugin] Please specify a robotNamespace.\n";

  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_force_sensor_plugin] Please specify a linkName.\n";

  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_force_sensor_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  joint_ = model_->GetJoint(link_name_);
  if (joint_ == NULL)
    gzthrow("[gazebo_force_sensor_plugin] Couldn't find specified joint \"" << link_name_ << "\".");

  if (_sdf->HasElement("randomEngineSeed")) {
    random_generator_.seed(_sdf->GetElement("randomEngineSeed")->Get<unsigned int>());
  }
  else {
    random_generator_.seed(std::chrono::system_clock::now().time_since_epoch().count());
  }

  getSdfParam<std::string>(_sdf, "forceSensorTopic", force_sensor_pub_topic_, force_sensor_pub_topic_);
  getSdfParam<std::string>(_sdf, "forceSensorTruthTopic", force_sensor_truth_pub_topic_, force_sensor_truth_pub_topic_);
  getSdfParam<std::string>(_sdf, "wrenchVectorTopic", wrench_vector_pub_topic_, wrench_vector_pub_topic_);
  getSdfParam<std::string>(_sdf, "parentFrameId", parent_frame_id_, parent_frame_id_);
  getSdfParam<std::string>(_sdf, "referenceFrameId", reference_frame_id_, reference_frame_id_);
  getSdfParam<sdf::Vector3>(_sdf, "noiseNormalLinearForce", noise_normal_linear_force, zeros3);
  getSdfParam<sdf::Vector3>(_sdf, "noiseNormalTorque", noise_normal_torque, zeros3);
  getSdfParam<sdf::Vector3>(_sdf, "noiseUniformLinearForce", noise_uniform_linear_force, zeros3);
  getSdfParam<sdf::Vector3>(_sdf, "noiseUniformTorque", noise_uniform_torque, zeros3);
  getSdfParam<int>(_sdf, "measurementDelay", measurement_delay_, measurement_delay_);
  getSdfParam<int>(_sdf, "measurementDivisor", measurement_divisor_, measurement_divisor_);
  getSdfParam<double>(_sdf, "unknownDelay", unknown_delay_, unknown_delay_);
  getSdfParam<double>(_sdf, "cutoffFrequency", cutoff_frequency_, cutoff_frequency_);
  getSdfParam<bool>(_sdf, "linForceMeasEnabled", lin_force_meas_enabled_, lin_force_meas_enabled_);
  getSdfParam<bool>(_sdf, "torqueMeasEnabled", torque_meas_enabled_, torque_meas_enabled_);
  getSdfParam<bool>(_sdf, "dispWrenchVector", disp_wrench_vector_, disp_wrench_vector_);

  parent_link_ = model_->GetLink(parent_frame_id_);
  if (parent_link_ == NULL && parent_frame_id_ != kDefaultParentFrameId) {
    gzthrow("[gazebo_force_sensor_plugin] Couldn't find specified parent link \"" << parent_frame_id_ << "\".");
  }

  reference_link_ = model_->GetLink(reference_frame_id_);
  if (reference_link_ == NULL && reference_frame_id_ != kDefaultReferenceFrameId) {
    gzthrow("[gazebo_force_sensor_plugin] Couldn't find specified reference frame \"" << reference_frame_id_ << "\".");
  }

  if (lin_force_meas_enabled_) {
    linear_force_n_[0] = NormalDistribution(0, noise_normal_linear_force.x);
    linear_force_n_[1] = NormalDistribution(0, noise_normal_linear_force.y);
    linear_force_n_[2] = NormalDistribution(0, noise_normal_linear_force.z);

    linear_force_u_[0] = UniformDistribution(-noise_uniform_linear_force.x, noise_uniform_linear_force.x);
    linear_force_u_[1] = UniformDistribution(-noise_uniform_linear_force.y, noise_uniform_linear_force.y);
    linear_force_u_[2] = UniformDistribution(-noise_uniform_linear_force.z, noise_uniform_linear_force.z);

    // Linear forces publisher
    lin_force_sensor_pub_ = node_handle_->advertise<geometry_msgs::Vector3Stamped>(force_sensor_pub_topic_+"/linear", 10);
    lin_force_sensor_truth_pub_ = node_handle_->advertise<geometry_msgs::Vector3Stamped>(force_sensor_truth_pub_topic_+"/linear", 10);
  }

  if (torque_meas_enabled_) {
    torque_n_[0] = NormalDistribution(0, noise_normal_torque.x);
    torque_n_[1] = NormalDistribution(0, noise_normal_torque.y);
    torque_n_[2] = NormalDistribution(0, noise_normal_torque.z);

    torque_u_[0] = UniformDistribution(-noise_uniform_torque.x, noise_uniform_torque.x);
    torque_u_[1] = UniformDistribution(-noise_uniform_torque.y, noise_uniform_torque.y);
    torque_u_[2] = UniformDistribution(-noise_uniform_torque.z, noise_uniform_torque.z);

    // Torques publisher
    ang_force_sensor_pub_ = node_handle_->advertise<geometry_msgs::Vector3Stamped>(force_sensor_pub_topic_+"/angular", 10);
    ang_force_sensor_truth_pub_ = node_handle_->advertise<geometry_msgs::Vector3Stamped>(force_sensor_truth_pub_topic_+"/angular", 10);
  }

  if (disp_wrench_vector_) {
    // Wrench vector publisher for RViz visualization
    wrench_vector_pub_ = node_handle_->advertise<geometry_msgs::WrenchStamped>(wrench_vector_pub_topic_, 10);
  }

  // Listen to the update event. This event is broadcast every simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboForceSensorPlugin::OnUpdate, this, _1));

}


// This gets called by the world update start event.
void GazeboForceSensorPlugin::OnUpdate(const common::UpdateInfo& _info) {
  /* APPLICATION POINT COMPUTATION */
  // C denotes child frame, P parent frame, R reference frame and W world frame.
  // Further C_pose_W_P denotes pose of P wrt. W expressed in C.
  math::Pose W_pose_W_C = link_->GetWorldCoGPose();
//  math::Pose W_pose_W_C = joint_->GetWorldPose();
  math::Pose gazebo_pose = W_pose_W_C;
  math::Pose gazebo_parent_pose = math::Pose::Zero;
  math::Pose gazebo_reference_pose = math::Pose::Zero;
  math::Pose W_pose_W_P = math::Pose::Zero;
  math::Pose W_pose_W_R = math::Pose::Zero;

  if (parent_frame_id_ != kDefaultParentFrameId) {
//    W_pose_W_P = parent_link_->GetWorldCoGPose();
    W_pose_W_P = model_->GetJoint(parent_frame_id_)->GetWorldPose();
    math::Pose C_pose_P_C = W_pose_W_C - W_pose_W_P;
    gazebo_pose = C_pose_P_C;
    gazebo_parent_pose = W_pose_W_P;
  }

  if (reference_frame_id_ != kDefaultReferenceFrameId && reference_frame_id_ != parent_frame_id_) {
    W_pose_W_R = reference_link_->GetWorldCoGPose();
    math::Pose P_pose_R_P = W_pose_W_P - W_pose_W_R;
    gazebo_parent_pose = P_pose_R_P;
    gazebo_reference_pose = W_pose_W_R;
  }

  /* FORCE PARSING */
  // The wrench vectors consist of two vectors representing the internal resultant force and torque applied on force
  // sensor joint by child link (i.e. force sensor itself). The values of the two 3D vectors are the coordinates in
  // parent frame coordinates system of the tip of an arrow starting at the origin of parent frame.

  math::Vector3 torque;
  math::Vector3 force;
  bool publish_forces = true;

  // Get internal forces and torques at force sensor joint.
  joint_wrench_ = joint_->GetForceTorque(0u);

  // First order filter
  if (prev_sim_time_ == 0.0) {
    // Initialize the first order filter.
    double time_constant_ = 1.0 / (2 * M_PI * cutoff_frequency_);
    sensor_data_filter_.reset(new FirstOrderFilterJointWrench(time_constant_, time_constant_, joint_wrench_));
  }
  else {
    // Smooth row data by first order filter.
    sampling_time_ = _info.simTime.Double() - prev_sim_time_;
    joint_wrench_ = sensor_data_filter_->updateFilter(joint_wrench_,sampling_time_);
  }
  prev_sim_time_ = _info.simTime.Double();


  // Get forces applied to force sensor joint from child link (i.e. force sensor) in parent frame coordinates.
  force = -joint_wrench_.body1Force;

  // Get torques applied to sensor joint from child link (i.e. force sensor) in parent frame coordinates.
  torque = -joint_wrench_.body1Torque;

  if (gazebo_sequence_ % measurement_divisor_ == 0) {
    // Copy data into wrench message.
    wrench_msg_.header.frame_id = parent_frame_id_;
    wrench_msg_.header.seq = wrench_sequence_++;
    wrench_msg_.header.stamp.sec = (world_->GetSimTime()).sec + ros::Duration(unknown_delay_).sec;
    wrench_msg_.header.stamp.nsec = (world_->GetSimTime()).nsec + ros::Duration(unknown_delay_).nsec;

    wrench_msg_.wrench.force.x  = force.x;
    wrench_msg_.wrench.force.y  = force.y;
    wrench_msg_.wrench.force.z  = force.z;
    wrench_msg_.wrench.torque.x = torque.x;
    wrench_msg_.wrench.torque.y = torque.y;
    wrench_msg_.wrench.torque.z = torque.z;

    if (publish_forces)
      wrench_queue_.push_back(std::make_pair(gazebo_sequence_ + measurement_delay_, wrench_msg_));
  }

  // Is it time to publish the front element?
  if (gazebo_sequence_ == wrench_queue_.front().first) {
    // Init wrench vector for RViz visualization purpose
    const geometry_msgs::Vector3 zeros3;
    wrench_msg_.wrench.force = zeros3;
    wrench_msg_.wrench.torque = zeros3;

    if (lin_force_meas_enabled_) {
      // Init true and noisy linear force message.
      geometry_msgs::Vector3StampedPtr true_lin_forces(new geometry_msgs::Vector3Stamped);
      geometry_msgs::Vector3StampedPtr noisy_lin_forces(new geometry_msgs::Vector3Stamped);

      // Fill true force message with latest linear forces data.
      true_lin_forces->vector = wrench_queue_.front().second.wrench.force;

      // Update headers.
      true_lin_forces->header = wrench_queue_.front().second.header;
      noisy_lin_forces->header = true_lin_forces->header;

      // Calculate linear force distortions.
      Eigen::Vector3d linear_force_n;
      linear_force_n << linear_force_n_[0](random_generator_) + linear_force_u_[0](random_generator_),
                        linear_force_n_[1](random_generator_) + linear_force_u_[1](random_generator_),
                        linear_force_n_[2](random_generator_) + linear_force_u_[2](random_generator_);
      // Fill linear force fields.
      noisy_lin_forces->vector.x = true_lin_forces->vector.x + linear_force_n[0];
      noisy_lin_forces->vector.y = true_lin_forces->vector.y + linear_force_n[1];
      noisy_lin_forces->vector.z = true_lin_forces->vector.z + linear_force_n[2];

      // Publish all the topics, for which the topic name is specified.
      if (lin_force_sensor_pub_.getNumSubscribers() > 0) {
        lin_force_sensor_pub_.publish(noisy_lin_forces);
      }
      if (lin_force_sensor_truth_pub_.getNumSubscribers() > 0) {
        lin_force_sensor_truth_pub_.publish(true_lin_forces);
      }

      // Update wrench vector forces for possible future use.
      wrench_msg_.wrench.force = noisy_lin_forces->vector;
    }

    if (torque_meas_enabled_) {
      // Init true and noisy torque message.
      geometry_msgs::Vector3StampedPtr true_torques(new geometry_msgs::Vector3Stamped);
      geometry_msgs::Vector3StampedPtr noisy_torques(new geometry_msgs::Vector3Stamped);

      // Fill true force message with latest torques data.
      true_torques->vector = wrench_queue_.front().second.wrench.torque;

      // Update headers.
      true_torques->header = wrench_queue_.front().second.header;
      noisy_torques->header = true_torques->header;

      // Calculate torque distortions.
      Eigen::Vector3d torque_n;
      torque_n << torque_n_[0](random_generator_) + torque_u_[0](random_generator_),
                  torque_n_[1](random_generator_) + torque_u_[1](random_generator_),
                  torque_n_[2](random_generator_) + torque_u_[2](random_generator_);
      // Fill torque fields.
      noisy_torques->vector.x = true_torques->vector.x + torque_n[0];
      noisy_torques->vector.y = true_torques->vector.y + torque_n[1];
      noisy_torques->vector.z = true_torques->vector.z + torque_n[2];

      // Publish all the topics, for which the topic name is specified.
      if (ang_force_sensor_pub_.getNumSubscribers() > 0) {
        ang_force_sensor_pub_.publish(noisy_torques);
      }
      if (ang_force_sensor_truth_pub_.getNumSubscribers() > 0) {
        ang_force_sensor_truth_pub_.publish(true_torques);
      }

      // Update wrench vector torques for possible future use.
      wrench_msg_.wrench.torque = noisy_torques->vector;
    }

    if (disp_wrench_vector_) {
      // Complete wrench message.
      wrench_msg_.header = wrench_queue_.front().second.header;
      // Publish all the topics, for which the topic name is specified.
      if (wrench_vector_pub_.getNumSubscribers() > 0) {
        wrench_vector_pub_.publish(wrench_msg_);
      }
    }

    // Transformation between sensor link and parent link.
    tf::Quaternion tf_q_P_C(gazebo_pose.rot.x, gazebo_pose.rot.y, gazebo_pose.rot.z, gazebo_pose.rot.w);
    tf::Vector3 tf_p_P_C(gazebo_pose.pos.x, gazebo_pose.pos.y, gazebo_pose.pos.z);
    tf_ = tf::Transform(tf_q_P_C, tf_p_P_C);
    transform_broadcaster_.sendTransform(tf::StampedTransform(tf_, wrench_queue_.front().second.header.stamp, parent_frame_id_, link_name_));
    if (parent_frame_id_ != kDefaultParentFrameId && reference_frame_id_ != parent_frame_id_) {
      // Transformation between parent link and reference frame.
      tf::Quaternion tf_q_R_P(gazebo_parent_pose.rot.x, gazebo_parent_pose.rot.y, gazebo_parent_pose.rot.z, gazebo_parent_pose.rot.w);
      tf::Vector3 tf_p_R_P(gazebo_parent_pose.pos.x, gazebo_parent_pose.pos.y, gazebo_parent_pose.pos.z);
      tf_.setOrigin(tf_p_R_P);
      tf_.setRotation(tf_q_R_P);
      transform_broadcaster_.sendTransform(tf::StampedTransform(tf_, wrench_queue_.front().second.header.stamp, reference_frame_id_, parent_frame_id_));
    }
    if (reference_frame_id_ != kDefaultReferenceFrameId) {
      // Transformation between reference frame and world (default).
      tf::Quaternion tf_q_W_R(gazebo_reference_pose.rot.x, gazebo_reference_pose.rot.y, gazebo_reference_pose.rot.z, gazebo_reference_pose.rot.w);
      tf::Vector3 tf_p_W_R(gazebo_reference_pose.pos.x, gazebo_reference_pose.pos.y, gazebo_reference_pose.pos.z);
      tf_.setOrigin(tf_p_W_R);
      tf_.setRotation(tf_q_W_R);
      transform_broadcaster_.sendTransform(tf::StampedTransform(tf_, wrench_queue_.front().second.header.stamp, "world", reference_frame_id_));
    }

    // Remove first element from wrench array (LIFO policy).
    wrench_queue_.pop_front();

  }
  ++gazebo_sequence_;
}

void GazeboForceSensorPlugin::PrintJointWrench(physics::JointWrench jw_){
  ROS_INFO("[gazebo_force_sensor_plugin]\nJoint Wrench Forces:\n  Fx = %f  Fy = %f  Fz = %f\nJoint Wrench Torques:\n  Tx = %f  Ty = %f  Tz = %f\n",
           jw_.body1Force.x, jw_.body1Force.y, jw_.body1Force.z, jw_.body1Torque.x, jw_.body1Torque.y, jw_.body1Torque.z);
}


GZ_REGISTER_MODEL_PLUGIN(GazeboForceSensorPlugin);
}

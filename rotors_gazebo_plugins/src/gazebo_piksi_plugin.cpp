/*
 * Copyright 2015 Daniel Eckert, ASL, ETH Zurich, Switzerland
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

#include "rotors_gazebo_plugins/gazebo_piksi_plugin.h"

#include <chrono>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <rotors_gazebo_plugins/common.h>

namespace gazebo
{
GazeboPiksiPlugin::~GazeboPiksiPlugin()
{
  updateConnection_.reset();
  if (node_handle_)
  {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

// void GazeboPiksiPlugin::InitializeParams() {};
// void GazeboPiksiPlugin::Publish() {};

void GazeboPiksiPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model
  model_ = _model;
  world_ = model_->GetWorld();

  ignition::math::Vector3d spp_noise_normal;
  ignition::math::Vector3d rtk_fixed_noise_normal;
  std::string start_fixed;
  std::string spp_navsatfix_covariance;
  std::string rtk_navsatfix_covariance;
  const ignition::math::Vector3d zeros3(0.0, 0.0, 0.0);

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_piksi_plugin] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_piksi_plugin] Please specify a linkName.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_piksi_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  if (_sdf->HasElement("randomEngineSeed"))
  {
    random_generator_.seed(_sdf->GetElement("randomEngineSeed")->Get<unsigned int>());
  }
  else
  {
    random_generator_.seed(std::chrono::system_clock::now().time_since_epoch().count());
  }
  getSdfParam<std::string>(_sdf, "sppPositionTopic", spp_position_pub_topic_, spp_position_pub_topic_);
  getSdfParam<std::string>(_sdf, "rtkPositionTopic", rtk_position_pub_topic_, rtk_position_pub_topic_);
  getSdfParam<std::string>(_sdf, "rtkModeTopic", rtk_mode_pub_topic_, rtk_mode_pub_topic_);
  getSdfParam<std::string>(_sdf, "rtkPiksiTopic", rtk_piksi_pub_topic_, rtk_piksi_pub_topic_);
  getSdfParam<std::string>(_sdf, "baselineTopic", baseline_pub_topic_, baseline_pub_topic_);
  getSdfParam<std::string>(_sdf, "parentFrameId", parent_frame_id_, parent_frame_id_);
  getSdfParam<std::string>(_sdf, "publishGroundTruth", publish_ground_truth_, publish_ground_truth_);
  getSdfParam<std::string>(_sdf, "startFixed", start_fixed, start_fixed);
  getSdfParam<std::string>(_sdf, "frameId", frame_id_, frame_id_);
  getSdfParam<ignition::math::Vector3d>(_sdf, "sppNoiseNormal", spp_noise_normal, zeros3);
  getSdfParam<ignition::math::Vector3d>(_sdf, "sppOffset", offset_spp_, zeros3);
  getSdfParam<ignition::math::Vector3d>(_sdf, "rtkFixedNoiseNormal", rtk_fixed_noise_normal, zeros3);
  getSdfParam<ignition::math::Vector3d>(_sdf, "rtkFixedOffset", offset_rtk_fixed_, zeros3);
  getSdfParam<ignition::math::Vector3d>(_sdf, "gpsStartPosition", gps_start_position_, { 0, 0, 0 });
  getSdfParam<std::string>(_sdf, "sppNavsatfixCovariance", spp_navsatfix_covariance, spp_navsatfix_covariance);
  getSdfParam<std::string>(_sdf, "rtkNavsatfixCovariance", rtk_navsatfix_covariance, rtk_navsatfix_covariance);
  strToDoubleArray(spp_navsatfix_covariance, spp_navsatfix_covariance_, 9);
  strToDoubleArray(rtk_navsatfix_covariance, rtk_navsatfix_covariance_, 9);

  getSdfParam<double>(_sdf, "updateRate", update_rate_, update_rate_);
  getSdfParam<double>(_sdf, "convergenceSpeed", convergence_speed_, convergence_speed_);
  getSdfParam<double>(_sdf, "fixLossProbability", fix_loss_probability_, fix_loss_probability_);
  getSdfParam<double>(_sdf, "fixLossTime", fix_loss_time_, fix_loss_time_);

  parent_link_ = world_->EntityByName(parent_frame_id_);
  if (parent_link_ == NULL && parent_frame_id_ != kDefaultParentFrameId)
  {
    gzthrow("[gazebo_piksi_plugin] Couldn't find specified parent link \"" << parent_frame_id_ << "\".");
  }

  spp_position_n_[0] = NormalDistribution(0, spp_noise_normal.X());
  spp_position_n_[1] = NormalDistribution(0, spp_noise_normal.Y());
  spp_position_n_[2] = NormalDistribution(0, spp_noise_normal.Z());

  rtk_position_n_[0] = NormalDistribution(0, rtk_fixed_noise_normal.X());
  rtk_position_n_[1] = NormalDistribution(0, rtk_fixed_noise_normal.Y());
  rtk_position_n_[2] = NormalDistribution(0, rtk_fixed_noise_normal.Z());

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboPiksiPlugin::OnUpdate, this, _1));
  spp_position_pub_ = node_handle_->advertise<sensor_msgs::NavSatFix>(spp_position_pub_topic_, 10);
  rtk_position_pub_ = node_handle_->advertise<sensor_msgs::NavSatFix>(rtk_position_pub_topic_, 10);
  rtk_mode_pub_ = node_handle_->advertise<std_msgs::String>(rtk_mode_pub_topic_, 10);
  rtk_piksi_pub_ = node_handle_->advertise<rotors_comm::PiksiRTKPos>(rtk_piksi_pub_topic_, 10);
  baseline_pub_ = node_handle_->advertise<rotors_comm::PiksiBaseline>(baseline_pub_topic_, 10);

  if (publish_ground_truth_ == "true")
  {
    ground_truth_pub_ = node_handle_->advertise<sensor_msgs::NavSatFix>("position_ground_truth", 10);
  }

  lat_start_ = gps_start_position_.X();
  lon_start_ = gps_start_position_.Y();
  alt_start_ = gps_start_position_.Z();

  // Calculate position scaling factors (m to lat/lon)
  lon_to_m_ = 111412.84 * cos(lat_start_ * M_PI / 180) - 93.5 * cos(3 * lat_start_ * M_PI / 180) +
              0.118 * cos(5 * lat_start_ * M_PI / 180);
  lat_to_m_ = 111132.954 - 559.822 * cos(2 * lat_start_ * M_PI / 180) + 1.175 * cos(4 * lat_start_ * M_PI / 180) -
              0.0023 * cos(6 * lat_start_ * M_PI / 180);
  m_to_lat_ = 1 / lat_to_m_;
  m_to_lon_ = 1 / lon_to_m_;

  // Initialize in RTK Float or Fix mode
  if (start_fixed == "true")
    mode_rtk_.data = "Fixed";
  else
  {
    mode_rtk_.data = "Float";
    UniformDistribution rtk_initialize_u =
        UniformDistribution(-rtk_float_start_error_width_, rtk_float_start_error_width_);
    // Determine random starting position
    sol_rtk_.latitude = lat_start_ + rtk_initialize_u(random_generator_) * m_to_lat_;
    sol_rtk_.longitude = lon_start_ + rtk_initialize_u(random_generator_) * m_to_lon_;
    sol_rtk_.altitude = alt_start_ + rtk_initialize_u(random_generator_) / 10;
  }

  // Set constant attributes of the NavSatFix messages
  sol_spp_.header.frame_id = frame_id_;
  sol_spp_.status.status = sol_spp_.status.STATUS_FIX;
  sol_spp_.status.service = sol_spp_.status.SERVICE_GPS;
  for (int i = 0; i <= 9; i++)
  {
    sol_spp_.position_covariance.elems[i] = spp_navsatfix_covariance_[i];
  }
  sol_spp_.position_covariance_type = sol_spp_.COVARIANCE_TYPE_DIAGONAL_KNOWN;

  sol_rtk_.header.frame_id = frame_id_;
  sol_rtk_.status.status = sol_rtk_.status.STATUS_GBAS_FIX;
  sol_rtk_.status.service = sol_rtk_.status.SERVICE_GPS;
  for (int i = 0; i <= 9; i++)
  {
    sol_rtk_.position_covariance.elems[i] = rtk_navsatfix_covariance_[i];
  }
  sol_rtk_.position_covariance_type = sol_rtk_.COVARIANCE_TYPE_DIAGONAL_KNOWN;
}

// This gets called by the world update start event.
void GazeboPiksiPlugin::OnUpdate(const common::UpdateInfo& _info)
{
  // Set update rate
  if ((_info.simTime - prev_update_time_).Double() <= 1 / update_rate_)
    return;
  prev_update_time_ = _info.simTime;

  // C denotes child frame, P parent frame, and W world frame.
  // Further C_pose_W_P denotes pose of P wrt. W expressed in C.
  ignition::math::Pose3d W_pose_W_C = link_->WorldCoGPose();
  ignition::math::Vector3d C_linear_velocity_W_C = link_->RelativeLinearVel();
  ignition::math::Vector3d C_angular_velocity_W_C = link_->RelativeAngularVel();

  ignition::math::Vector3d gazebo_linear_velocity = C_linear_velocity_W_C;
  ignition::math::Vector3d gazebo_angular_velocity = C_angular_velocity_W_C;
  ignition::math::Pose3d gazebo_pose = W_pose_W_C;

  if (parent_frame_id_ != kDefaultParentFrameId)
  {
    ignition::math::Pose3d W_pose_W_P = parent_link_->WorldPose();
    ignition::math::Vector3d P_linear_velocity_W_P = parent_link_->RelativeLinearVel();
    ignition::math::Vector3d P_angular_velocity_W_P = parent_link_->RelativeAngularVel();
    ignition::math::Pose3d C_pose_P_C_ = W_pose_W_C - W_pose_W_P;
    ignition::math::Vector3d C_linear_velocity_P_C;

    C_linear_velocity_P_C = -C_pose_P_C_.Rot().Inverse() * P_angular_velocity_W_P.Cross(C_pose_P_C_.Pos()) +
                            C_linear_velocity_W_C - C_pose_P_C_.Rot().Inverse() * P_linear_velocity_W_P;

    gazebo_angular_velocity = C_angular_velocity_W_C - C_pose_P_C_.Rot().Inverse() * P_angular_velocity_W_P;
    gazebo_linear_velocity = C_linear_velocity_P_C;
    gazebo_pose = C_pose_P_C_;
  }

  // Calculate Ground Truth in LLA coordinates
  sol_gt_.longitude = lon_start_ + m_to_lon_ * gazebo_pose.Pos().X();
  sol_gt_.latitude = lat_start_ + m_to_lat_ * gazebo_pose.Pos().Y();
  sol_gt_.altitude = alt_start_ + gazebo_pose.Pos().Z();

  // Calculate position with errors for SPP GPS.
  Eigen::Vector3d spp_pos_n;
  spp_pos_n << spp_position_n_[0](random_generator_), spp_position_n_[1](random_generator_),
      spp_position_n_[2](random_generator_);
  sol_spp_.longitude = lon_start_ + m_to_lon_ * (gazebo_pose.Pos().X() + offset_spp_.X() + spp_pos_n.x());
  sol_spp_.latitude = lat_start_ + m_to_lat_ * (gazebo_pose.Pos().Y() + offset_spp_.Y() + spp_pos_n.y());
  sol_spp_.altitude = alt_start_ + gazebo_pose.Pos().Z() + offset_spp_.Z() + spp_pos_n.z();
  sol_spp_.header.stamp = ros::Time().now();

  // Calculate position with errors for RTK GPS.
  sol_rtk_.header.stamp = ros::Time().now();
  sol_baseline_.header.stamp = ros::Time().now();
  if (mode_rtk_.data == "Float")
  {
    // For RTK Float: Random walk towards actual position
    double dist_x = lon_to_m_ * sol_gt_.longitude - baseline_.x;
    double dist_y = lat_to_m_ * sol_gt_.latitude - baseline_.y;
    double dist_z = sol_gt_.altitude - baseline_.z;

    double c_s = convergence_speed_ / 100;
    double step_x =
        NormalDistribution((c_s * sin(dist_x) + 1.1 * c_s) * dist_x, 0.01 * abs(dist_x) + 0.005)(random_generator_);
    double step_y =
        NormalDistribution((c_s * cos(dist_y) + 1.1 * c_s) * dist_y, 0.01 * abs(dist_y) + 0.005)(random_generator_);
    double step_z =
        NormalDistribution((c_s * sin(dist_z) + 1.1 * c_s) * dist_z, 0.01 * abs(dist_z) + 0.005)(random_generator_);

    baseline_.x = baseline_.x + step_x;
    baseline_.y = baseline_.y + step_y;
    baseline_.z = baseline_.z + step_z;

    // If solution converged close to real solution set to RTK Fixed, with a certain probability.
    if (sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z) < 0.7 &&
        UniformDistribution(0, 1)(random_generator_) > 0.95)
      mode_rtk_.data = "Fixed";
  }
  else if (mode_rtk_.data == "Fixed")
  {
    // For RTK Fix: Augment ground truth with noise
    Eigen::Vector3d rtk_pos_n;
    rtk_pos_n << rtk_position_n_[0](random_generator_), rtk_position_n_[1](random_generator_),
        rtk_position_n_[2](random_generator_);

    baseline_.x = gazebo_pose.Pos().X() + offset_rtk_fixed_.X() + rtk_pos_n.x();
    baseline_.y = gazebo_pose.Pos().Y() + offset_rtk_fixed_.Y() + rtk_pos_n.y();
    baseline_.z = gazebo_pose.Pos().Z() + offset_rtk_fixed_.Z() + rtk_pos_n.z();

    // Loose fix with a certain probability (or at fix_loss_time_), and jump to random nearby position
    if (UniformDistribution(0, 1)(random_generator_) <= fix_loss_probability_ ||
        std::abs((world_->RealTime()).sec - fix_loss_time_) <= 0.5 / update_rate_)
    {
      NormalDistribution loose_fix_pos = NormalDistribution(0, rtk_float_start_error_width_ / 7);
      baseline_.x = loose_fix_pos(random_generator_);
      baseline_.y = loose_fix_pos(random_generator_);
      baseline_.z = loose_fix_pos(random_generator_) / 5;
      mode_rtk_.data = "Float";
    }
  }

  // Use baseline output to LLA coordinates
  sol_rtk_.longitude = lon_start_ + baseline_.x * m_to_lon_;
  sol_rtk_.latitude = lat_start_ + baseline_.y * m_to_lat_;
  sol_rtk_.altitude = alt_start_ + baseline_.z;
  sol_piksi_rtk_.navsatfix = sol_rtk_;

  // Set RTK fix/float status variables
  if (mode_rtk_.data == "Fixed")
  {
    sol_piksi_rtk_.mode_fixed = 1;
    sol_baseline_.mode_fixed = 1;
  }
  else
  {
    sol_piksi_rtk_.mode_fixed = 0;
    sol_baseline_.mode_fixed = 0;
  }

  // Scale Baseline message, and convert frem ENU to NED
  sol_baseline_.baseline.x = baseline_.y * 1000;
  sol_baseline_.baseline.y = baseline_.x * 1000;
  sol_baseline_.baseline.z = -baseline_.z * 1000;

  // Publish messages
  if (rtk_piksi_pub_.getNumSubscribers() > 0)
  {
    rtk_piksi_pub_.publish(sol_piksi_rtk_);
  }
  if (baseline_pub_.getNumSubscribers() > 0)
  {
    baseline_pub_.publish(sol_baseline_);
  }
  if (rtk_mode_pub_.getNumSubscribers() > 0)
  {
    rtk_mode_pub_.publish(mode_rtk_);
  }
  if (rtk_position_pub_.getNumSubscribers() > 0)
  {
    rtk_position_pub_.publish(sol_rtk_);
  }
  if (spp_position_pub_.getNumSubscribers() > 0)
  {
    spp_position_pub_.publish(sol_spp_);
  }
  if (publish_ground_truth_ == "true" && ground_truth_pub_.getNumSubscribers() > 0)
  {
    ground_truth_pub_.publish(sol_gt_);
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboPiksiPlugin);
}  // namespace gazebo

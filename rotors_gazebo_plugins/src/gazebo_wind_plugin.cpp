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

#include "rotors_gazebo_plugins/gazebo_wind_plugin.h"

#include <fstream>
#include <math.h>

#include "ConnectGazeboToRosTopic.pb.h"

namespace gazebo {

GazeboWindPlugin::~GazeboWindPlugin() {
  event::Events::DisconnectWorldUpdateBegin(update_connection_);
}

void GazeboWindPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  // Store the pointer to the model.
  model_ = _model;
  world_ = model_->GetWorld();

  double wind_gust_start = kDefaultWindGustStart;
  double wind_gust_duration = kDefaultWindGustDuration;

  //==============================================//
  //========== READ IN PARAMS FROM SDF ===========//
  //==============================================//

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_wind_plugin] Please specify a robotNamespace.\n";

  // Create Gazebo Node.
  node_handle_ = gazebo::transport::NodePtr(new transport::Node());

  // Initialise with default namespace (typically /gazebo/default/).
  node_handle_->Init();

  if (_sdf->HasElement("xyzOffset"))
    xyz_offset_ = _sdf->GetElement("xyzOffset")->Get<math::Vector3>();
  else
    gzerr << "[gazebo_wind_plugin] Please specify a xyzOffset.\n";

  getSdfParam<std::string>(_sdf, "windForcePubTopic", wind_force_pub_topic_,
                           wind_force_pub_topic_);
  getSdfParam<std::string>(_sdf, "windSpeedPubTopic", wind_speed_pub_topic_,
                           wind_speed_pub_topic_);
  getSdfParam<std::string>(_sdf, "frameId", frame_id_, frame_id_);
  getSdfParam<std::string>(_sdf, "linkName", link_name_, link_name_);
  // Get the wind speed params from SDF.
  getSdfParam<double>(_sdf, "windSpeedMean", wind_speed_mean_,
                      wind_speed_mean_);
  getSdfParam<double>(_sdf, "windSpeedVariance", wind_speed_variance_,
                      wind_speed_variance_);
  getSdfParam<math::Vector3>(_sdf, "windDirection", wind_direction_,
                      wind_direction_);
  // Check if a custom static wind field should be used.
  getSdfParam<bool>(_sdf, "customStaticWindField", custom_static_wind_field_,
                      custom_static_wind_field_);
  if(!custom_static_wind_field_) {
    // Get the wind params from SDF.
    getSdfParam<double>(_sdf, "windForceMean", wind_force_mean_,
                        wind_force_mean_);
    getSdfParam<double>(_sdf, "windForceVariance", wind_force_variance_,
                        wind_force_variance_);
    // Get the wind gust params from SDF.
    getSdfParam<double>(_sdf, "windGustStart", wind_gust_start, wind_gust_start);
    getSdfParam<double>(_sdf, "windGustDuration", wind_gust_duration,
                        wind_gust_duration);
    getSdfParam<double>(_sdf, "windGustForceMean", wind_gust_force_mean_,
                        wind_gust_force_mean_);
    getSdfParam<double>(_sdf, "windGustForceVariance", wind_gust_force_variance_,
                        wind_gust_force_variance_);
    getSdfParam<math::Vector3>(_sdf, "windGustDirection", wind_gust_direction_,
                        wind_gust_direction_);

    wind_direction_.Normalize();
    wind_gust_direction_.Normalize();
    wind_gust_start_ = common::Time(wind_gust_start);
    wind_gust_end_ = common::Time(wind_gust_start + wind_gust_duration);
  } else {
    // Get the wind field text file path, read it and save data.
    std::string custom_wind_field_path = kDefaultCustomWindFieldPath;
    getSdfParam<std::string>(_sdf, "customWindFieldPath", custom_wind_field_path,
                        custom_wind_field_path);
    ReadCustomWindField(custom_wind_field_path);
  }

  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_wind_plugin] Couldn't find specified link \"" << link_name_
                                                                   << "\".");

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboWindPlugin::OnUpdate, this, _1));
}

// This gets called by the world update start event.
void GazeboWindPlugin::OnUpdate(const common::UpdateInfo& _info) {
  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  // Get the current simulation time.
  common::Time now = world_->GetSimTime();
  
  math::Vector3 wind_velocity = math::Vector3(0,0,0);

  // Choose user-specified method for calculating wind velocity.
  if (!custom_static_wind_field_) {
    // Calculate the wind force.
    double wind_strength = wind_force_mean_;
    math::Vector3 wind = wind_strength * wind_direction_;
    // Apply a force from the constant wind to the link.
    link_->AddForceAtRelativePosition(wind, xyz_offset_);

    math::Vector3 wind_gust(0, 0, 0);
    // Calculate the wind gust force.
    if (now >= wind_gust_start_ && now < wind_gust_end_) {
      double wind_gust_strength = wind_gust_force_mean_;
      wind_gust = wind_gust_strength * wind_gust_direction_;
      // Apply a force from the wind gust to the link.
      link_->AddForceAtRelativePosition(wind_gust, xyz_offset_);
    }

    wrench_stamped_msg_.mutable_header()->set_frame_id(frame_id_);
    wrench_stamped_msg_.mutable_header()->mutable_stamp()->set_sec(now.sec);
    wrench_stamped_msg_.mutable_header()->mutable_stamp()->set_nsec(now.nsec);

    wrench_stamped_msg_.mutable_wrench()->mutable_force()->set_x(wind.x +
                                                                 wind_gust.x);
    wrench_stamped_msg_.mutable_wrench()->mutable_force()->set_y(wind.y +
                                                                 wind_gust.y);
    wrench_stamped_msg_.mutable_wrench()->mutable_force()->set_z(wind.z +
                                                                 wind_gust.z);

    // No torque due to wind, set x,y and z to 0.
    wrench_stamped_msg_.mutable_wrench()->mutable_torque()->set_x(0);
    wrench_stamped_msg_.mutable_wrench()->mutable_torque()->set_y(0);
    wrench_stamped_msg_.mutable_wrench()->mutable_torque()->set_z(0);

    wind_force_pub_->Publish(wrench_stamped_msg_);

    // Calculate the wind speed.
    double wind_speed = wind_speed_mean_;
    wind_velocity = wind_speed * wind_direction_;
  } else {
    // Get the current position of the aircraft in world coordinates.
    math::Vector3 link_position = link_->GetWorldPose().pos;

    // Find the x, y indices of the eight grid points enclosing the aircraft.
    int i_inf = floor((link_position.x - min_x_) / res_x_);
    int j_inf = floor((link_position.y - min_y_) / res_y_);

    // In case aircraft is on one of the boundary surfaces at max_x or max_y.
    if (i_inf == n_x_ - 1) i_inf = n_x_ - 2;
    if (j_inf == n_y_ - 1) j_inf = n_y_ - 2;

    int i_sup = i_inf + 1;
    int j_sup = j_inf + 1;

    int idx_i[8] = {i_inf,i_inf,i_sup,i_sup,i_inf,i_inf,i_sup,i_sup};
    int idx_j[8] = {j_inf,j_inf,j_inf,j_inf,j_sup,j_sup,j_sup,j_sup};

    // Find the vertical factor of the aircraft in each of the four surrounding 
    // grid columns, and their minimal/maximal value.
    float vertical_factors_columns[4];
    for (int i = 0; i < 4; i++) {
      vertical_factors_columns[i] = (
        link_position.z - bottom_z_[idx_i[2*i] + idx_j[2*i] * n_x_]) /
        (top_z_[idx_i[2*i] + idx_j[2*i] * n_x_] - bottom_z_[idx_i[2*i] +
                                                            idx_j[2*i] * n_x_]);
    }

    float vertical_factors_min = std::min(std::min(std::min(
      vertical_factors_columns[0],vertical_factors_columns[1]),
      vertical_factors_columns[2]),vertical_factors_columns[3]);
    float vertical_factors_max = std::max(std::max(std::max(
      vertical_factors_columns[0],vertical_factors_columns[1]),
      vertical_factors_columns[2]),vertical_factors_columns[3]);

    // Check if aircraft is out of wind field or not, and act accordingly.
    if (!(i_inf < 0 || j_inf < 0 || vertical_factors_max < 0 || 
        i_sup > (n_x_ - 1) || j_sup > (n_y_ - 1) || vertical_factors_min > 1)) {
      // Find indices in z-direction for each of the vertices. If link is not 
      // within the range of one of the columns, set to lowest or highest two.
      int idx_k[8] = {0,vertical_spacing_factors_.size()-1,0,
                      vertical_spacing_factors_.size()-1,0,
                      vertical_spacing_factors_.size()-1,0,
                      vertical_spacing_factors_.size()-1};

      for (int i = 0; i < 4; i++) {
        if (vertical_factors_columns[i] < 0) {
          // Link z-position below lowest grid point of that column.
          idx_k[2*i+1] = 1;
        } else if (vertical_factors_columns[i] >= 1) {
          // Link z-position above highest grid point of that column.
          idx_k[2*i] = vertical_spacing_factors_.size() - 2;
        } else {
          // Link z-position between two grid points in that column.
          for (int j = 0; j < vertical_spacing_factors_.size() - 1; j++) {
            if (vertical_spacing_factors_[j] <= vertical_factors_columns[i] && 
                vertical_spacing_factors_[j+1] > vertical_factors_columns[i]) {
              idx_k[2*i] = j;
              idx_k[2*i+1] = j + 1;
              break;
            }
          }
        }
      }

      // Extract the wind velocities corresponding to each vertex.
      math::Vector3 wind_at_vertices[8];
      for (int i = 0; i < 8; i++) {
        wind_at_vertices[i].x = u_[idx_i[i] + idx_j[i] * n_x_ + idx_k[i] * n_x_
                                                                        * n_y_];
        wind_at_vertices[i].y = v_[idx_i[i] + idx_j[i] * n_x_ + idx_k[i] * n_x_
                                                                        * n_y_];
        wind_at_vertices[i].z = w_[idx_i[i] + idx_j[i] * n_x_ + idx_k[i] * n_x_
                                                                        * n_y_];
      }

      // Extract the relevant coordinate of every point needed for trilinear 
      // interpolation (first z-direction, then x-direction, then y-direction).
      float interpolation_points[14];
      for (int i = 0; i < 14; i++) {
        if (i < 8) {
          interpolation_points[i] = (
            top_z_[idx_i[i] + idx_j[i] * n_x_] - bottom_z_[idx_i[i] + idx_j[i] * n_x_])
            * vertical_spacing_factors_[idx_k[i]] + bottom_z_[idx_i[i] + idx_j[i] * n_x_];
        } else if (i >= 8 && i < 12) {
          interpolation_points[i] = min_x_ + res_x_ * idx_i[2*(i-8)];
        } else {
          interpolation_points[i] = min_y_ + res_y_ * idx_j[i-9];
        }
      }

      wind_velocity = TrilinearInterpolation(
        link_position, wind_at_vertices, interpolation_points);
    } else {
      // Set the wind velocity to the default constant value specified by user.
      double wind_speed = wind_speed_mean_;
      wind_velocity = wind_speed * wind_direction_;
    }
  }
  
  wind_speed_msg_.mutable_header()->set_frame_id(frame_id_);
  wind_speed_msg_.mutable_header()->mutable_stamp()->set_sec(now.sec);
  wind_speed_msg_.mutable_header()->mutable_stamp()->set_nsec(now.nsec);

  wind_speed_msg_.mutable_velocity()->set_x(wind_velocity.x);
  wind_speed_msg_.mutable_velocity()->set_y(wind_velocity.y);
  wind_speed_msg_.mutable_velocity()->set_z(wind_velocity.z);

  wind_speed_pub_->Publish(wind_speed_msg_);
}

void GazeboWindPlugin::CreatePubsAndSubs() {
  // Create temporary "ConnectGazeboToRosTopic" publisher and message.
  gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
          "~/" + kConnectGazeboToRosSubtopic, 1);

  gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;

  // ============================================ //
  // ========= WRENCH STAMPED MSG SETUP ========= //
  // ============================================ //
  wind_force_pub_ = node_handle_->Advertise<gz_geometry_msgs::WrenchStamped>(
      "~/" + namespace_ + "/" + wind_force_pub_topic_, 1);

  // connect_gazebo_to_ros_topic_msg.set_gazebo_namespace(namespace_);
  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   wind_force_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                wind_force_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::WRENCH_STAMPED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);

  // ============================================ //
  // ========== WIND SPEED MSG SETUP ============ //
  // ============================================ //
  wind_speed_pub_ = node_handle_->Advertise<gz_mav_msgs::WindSpeed>(
      "~/" + namespace_ + "/" + wind_speed_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   wind_speed_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                wind_speed_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::WIND_SPEED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);
}

// This functions opens a text file containing the custom wind field, 
// reads it and saves the data.
void GazeboWindPlugin::ReadCustomWindField(std::string& custom_wind_field_path) {
  std::ifstream fin;
  fin.open(custom_wind_field_path);
  if (fin.is_open()) {
    std::string data_name;
    float data;
    // Read the line with the variable name.
    while (fin >> data_name) {
      // Save data on following line into the correct variable.
      if (data_name == "min_x:") {
        fin >> min_x_;
      } else if (data_name == "min_y:") {
        fin >> min_y_;
      } else if (data_name == "n_x:") {
        fin >> n_x_;
      } else if (data_name == "n_y:") {
        fin >> n_y_;
      } else if (data_name == "res_x:") {
        fin >> res_x_;
      } else if (data_name == "res_y:") {
        fin >> res_y_;
      } else if (data_name == "vertical_spacing_factors:") {
        while (fin >> data) {
          vertical_spacing_factors_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "bottom_z:") {
        while (fin >> data) {
          bottom_z_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "top_z:") {
        while (fin >> data) {
          top_z_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "u:") {
        while (fin >> data) {
          u_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "v:") {
        while (fin >> data) {
          v_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "w:") {
        while (fin >> data) {
          w_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else {
        // If invalid data name, read the rest of the invalid line, 
        // publish a message and ignore data on next line. Then resume reading.
        std::string restOfLine;
        getline(fin, restOfLine);
        gzerr << " [gazebo_wind_plugin] Invalid data name '" << data_name << restOfLine <<
              "' in custom wind field text file. Ignoring data on next line.\n";
        fin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      }
    }
    fin.close();
    ROS_INFO_STREAM("[gazebo_wind_plugin] Successfully read custom wind field from text file.");
  } else {
    gzerr << "[gazebo_wind_plugin] Could not open custom wind field text file.\n";
  }

}

/*
  Linear interpolation
    position: y-coordinate of the target point.
    values:   a pointer to an array of size 2 containing the wind values of 
              the two points to interpolate from (12 and 13).
    points:   a pointer to an array of size 2 containing the y-coordinate 
              of the two points to interpolate from.
*/
math::Vector3 GazeboWindPlugin::LinearInterpolation(
  double position, math::Vector3 *values, float *points) {
  math::Vector3 value = values[0] + (values[1] - values[0]) /
                        (points[1] - points[0]) * (position - points[0]);
  return value;
}

/*
  Bilinear interpolation
    position: a pointer to an aray of size 2 containing the x- and y-coordinates
              of the target point.
    values:   a pointer to an array of size 4 containing the wind values 
              of the four points to interpolate from (8, 9, 10 and 11).
    points:   a pointer to an array of size 6 containing the x-coordinate 
              of the four points to interpolate from,and the y-coordinate 
              of the two intermediate points (12 and 13).
*/
math::Vector3 GazeboWindPlugin::BilinearInterpolation(
  double *position, math::Vector3 *values, float *points) {
  math::Vector3 intermediate_values[2] = {
                                            LinearInterpolation(
                                              position[0],&(values[0]),&(points[0])),
                                            LinearInterpolation(
                                              position[0],&(values[2]),&(points[2]))
                                         };
  math::Vector3 value = LinearInterpolation(
                          position[1], intermediate_values, &(points[4]));
  return value;
}

/*
  Trilinear interpolation
    link_position:  a Vector3 containing the x, y and z-coordinates
                    of the target point.
    values:   a pointer to an array of size 8 containing the wind values of the 
              eight points to interpolate from (0, 1, 2, 3, 4, 5, 6 and 7).
    points:   a pointer to an array of size 14 containing the z-coordinate
              of the eight points to interpolate from, the x-coordinate 
              of the four intermediate points (8, 9, 10 and 11), and the 
              y-coordinate of the last two intermediate points (12 and 13).
*/
math::Vector3 GazeboWindPlugin::TrilinearInterpolation(
  math::Vector3 link_position, math::Vector3 *values, float *points) {
  double position[3] = {link_position.x,link_position.y,link_position.z};
  math::Vector3 intermediate_values[4] = {
                                            LinearInterpolation(
                                              position[2],&(values[0]),&(points[0])),
                                            LinearInterpolation(
                                              position[2],&(values[2]),&(points[2])),
                                            LinearInterpolation(
                                              position[2],&(values[4]),&(points[4])),
                                            LinearInterpolation(
                                              position[2],&(values[6]),&(points[6]))
                                         };
  math::Vector3 value = BilinearInterpolation(
    &(position[0]),intermediate_values,&(points[8]));
  return value;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboWindPlugin);

}  // namespace gazebo

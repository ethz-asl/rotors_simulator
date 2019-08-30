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


#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_BETA_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_BETA_PLUGIN_H

#include <string>
#include <random>
#include <complex>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <eigen3/Eigen/Dense>

#include <mav_msgs/default_topics.h>  // This comes from the mav_comm repo

#include "rotors_gazebo_plugins/common.h"

#include "WindSpeed.pb.h"             // Wind speed message
#include "WindSpeedBeta.pb.h"         // Wind speed message
#include "WrenchStamped.pb.h"         // Wind force message

namespace gazebo {

typedef ignition::math::Vector3d V3D;
typedef ignition::math::Vector3<std::complex<double>> V3C;
typedef ignition::math::Matrix3d M3D;
typedef ignition::math::Matrix3<std::complex<double>> M3C;

/// \brief    This gazebo plugin simulates wind acting on a model.
/// \details  This plugin publishes on a Gazebo topic and instructs the ROS interface plugin to
///           forward the message onto ROS.
class GazeboWindBetaPlugin : public ModelPlugin {
 public:
  GazeboWindBetaPlugin()
      : ModelPlugin(),
        namespace_(kDefaultNamespace),
        node_handle_(nullptr),
        pubs_and_subs_created_(false) {}

  virtual ~GazeboWindBetaPlugin();

 protected:

  /// \brief Load the plugin.
  /// \param[in] _model Pointer to the model that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Called when the world is updated.
  /// \param[in] _info Update timing information.
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:

  /// \brief    Flag that is set to true once CreatePubsAndSubs() is called, used
  ///           to prevent CreatePubsAndSubs() from be called on every OnUpdate().
  bool pubs_and_subs_created_;

  /// \brief    Creates all required publishers and subscribers, incl. routing of messages to/from ROS if required.
  /// \details  Call this once the first time OnUpdate() is called (can't
  ///           be called from Load() because there is no guarantee GazeboRosInterfacePlugin has
  ///           has loaded and listening to ConnectGazeboToRosTopic and ConnectRosToGazeboTopic messages).
  void CreatePubsAndSubs();

  /// \brief    Pointer to the update event connection.
  event::ConnectionPtr update_connection_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;

  std::string namespace_;

  struct WindQuery {
     physics::LinkPtr link;
     std::string frame_id;
     std::string wind_topic;
     gazebo::transport::PublisherPtr wind_pub;
     gz_mav_msgs::WindSpeedBeta wind_msg;

     ignition::math::Vector3d wind_ned;
     ignition::math::Vector3d pos_ned;
     ignition::math::Matrix3d wind_grad_ned;
     bool sdf_valid = true;
  };

  WindQuery * wind_queries_;
  int n_query_ = 0;

  struct WindLayer {
     double altitude = 0.0;   // lower altitude of wind layer, [m]
     //bool rel_alt = false;    // altitude over ground (true) or sea-level (false)
     //double blend_alt = 0.0;  // blending interval to smoothly mix between different layers [m]

     V3D wind_mean = V3D(0,0,0);             // spatiotemporal mean wind vector in NED [m/s]

     /*
     V3D trb_sigma_nrm = V3D(0.1,0.1,0.1);   // sigma/|wind_mean| of lon/lat/vert turbulence components [-]
     V3D trb_length = V3D(762,762,762);      // turbulence length scale of lon/lat/vert components [m]

     int n_omega = 100;   // number of spatial frequencies to be used
     double * omega;      // array of wavenumbers: omega_i = 2*pi/lambda_i [rad/m]
     double d_omega;
     double * phi_u;      // longitudinal turbulence psd,  [m³/s²]
     double * phi_v;      // lateral turbulence psd,  [m³/s²]
     double * phi_w;      // vertical turbulence psd,  [m³/s²]

     int n_theta = 21;    // number of planar spreading directions in [-pi/2,pi/2[
     double * theta;      // array of planar wave-directions, e[-pi/2,pi/2[ [rad]
     double d_theta;

     double gust_speed_mean = 0.0;
     double gust_speed_variance = 0.0;
     V3D gust_ned_direction = V3D(1,0,0);

     common::Time wind_gust_period_mean = 0.0;
     common::Time wind_gust_period_variance = 0.0;
     common::Time wind_gust_duration_mean = 0.0;
     common::Time wind_gust_duration_variance = 0.0;

     void InitializeLayer(){
        double omega_low =  2*M_PI/1e3;   // wavelength <= 1000m
        double omega_high = 2*M_PI/0.5;   // wavelength >= 0.5m (!aliasing! check sim-freq vs max. air-speed)

        omega = new double [n_omega];
        phi_u = new double [n_omega];
        phi_v = new double [n_omega];
        phi_w = new double [n_omega];

        d_omega = (omega_high-omega_low)/(n_omega-1);

        for (int i = 0; i < n_omega; i++) {
            omega[i] = i*d_omega;
        }

        //GazeboWindBetaPlugin::LonPsdKarman(omega, phi_u, n_omega, trb_length[0], trb_sigma_nrm[0]*wind_mean.Length());
        //GazeboWindBetaPlugin::LatVertPsdKarman(omega, phi_v, n_omega, trb_length[1], trb_sigma_nrm[1]*wind_mean.Length());
        //GazeboWindBetaPlugin::LatVertPsdKarman(omega, phi_w, n_omega, trb_length[2], trb_sigma_nrm[2]*wind_mean.Length());

        theta = new double [n_theta];
        d_theta = M_PI/(n_theta);

        for (int i = 0; i < n_theta; i++) {
            theta[i] = i*d_theta;
        }
     }
     */
  };

  WindLayer * wind_layers_;
  int * layer_index_alt_incr_;
  int n_layer_ = 0;

  common::Time wind_gust_end_;
  common::Time wind_gust_start_;

  // Turbulence: Fourier Simulation
  int counter=0;

  double lambda_min_ = 2.5;
  double L_ = 725;
  double sigma_ = 1.5;

  V3D trnsp_vel_turb_ = V3D(0,0,0);

  int nk_x_ = 15;
  int nk_y_ = 15;
  int nk_z_ = 15;

  double dk_x_;// = 2*M_PI/lambda_min_/nk_x_;
  double dk_y_;// = 2*M_PI/lambda_min_/nk_y_;
  double dk_z_;// = 2*M_PI/lambda_min_/nk_z_;

  M3C *** C_ij_; //M3C C_ij_[nk_x_][nk_y_][nk_z_];
  V3C *** rand_phase_; //V3C rand_phase_[nk_x_][nk_y_][nk_z_];

  void KarmanEnergySpectrum(double &E, V3D k, double L, double sigma);

  void SpectralTensorIsoInc(M3D &Phi, V3D k, double L, double sigma);

  void SpectralTensorIsoIncDec(M3C &A, V3D k, double L, double sigma);

  static void PrintMatrix(M3C M);

  static bool SortByAltitude(const WindLayer &L1, const WindLayer &L2) {return L1.altitude<L2.altitude;}

  /*
  /// \brief Returns power spectral density phi(omega) [m³/s²] of the
  /// longitudinal Karman turbulence model. n=length of passed arrays.
  /// l = turbulence length scale [m], s = turbulence rms [m²/s²]
  void LonPsdKarman(double * omega, double * phi, double n, double l, double s);

  /// \brief Returns power spectral density phi(omega) of the
  /// lateral/vertical Karman turbulence model. n=length of passed arrays.
  /// l = turbulence length scale [m], s = turbulence rms [m²/s²]
  void LatVertPsdKarman(double * omega, double * phi, double n, double l, double s);
 */

  //  Variables for custom wind field generation.
  bool wind_is_valid_ = false;
  double min_x_ = 0;
  double min_y_ = 0;
  double min_z_ = 0;
  int n_x_ = 0;
  int n_y_ = 0;
  int n_z_ = 0;
  double res_x_ = 1;
  double res_y_ = 1;
  double res_z_ = 1;
  V3D wf_offset = V3D(0,0,0);
  std::vector<V3D> wf_;
  std::vector<V3D> dwfdx_;
  std::vector<V3D> dwfdy_;
  std::vector<V3D> dwfdz_;
  std::vector<double> trb_kin_nrg_;

  /// \brief  Reads wind data from a text file and saves it.
  /// \param[in] custom_wind_field_path Path to the wind field from ~/.ros.
  void ReadCustomWindFieldCSV(std::string& custom_wind_field_path);

  /// \brief  Calculates gradients of wind data.
  void ProcessCustomWindField();

  // \brief  Reads wind data from a text file and saves it.
  // \param[in] custom_wind_field_path Path to the wind field from ~/.ros.
  //void ReadCustomWindField(std::string& custom_wind_field_path);

  /// \brief  Functions for trilinear interpolation of wind field at aircraft position.
  
  /// \brief  Linear interpolation
  /// \param[in]  position y-coordinate of the target point.
  ///             values Pointer to an array of size 2 containing the wind values
  ///                    of the two points to interpolate from (12 and 13).
  ///             points Pointer to an array of size 2 containing the y-coordinate 
  ///                    of the two points to interpolate from.
  ignition::math::Vector3d LinearInterpolation(double position, ignition::math::Vector3d * values, double* points) const;
  
  /// \brief  Bilinear interpolation
  /// \param[in]  position Pointer to an array of size 2 containing the x- and 
  ///                      y-coordinates of the target point.
  ///             values Pointer to an array of size 4 containing the wind values 
  ///                    of the four points to interpolate from (8, 9, 10 and 11).
  ///             points Pointer to an array of size 14 containing the z-coordinate
  ///                    of the eight points to interpolate from, the x-coordinate 
  ///                    of the four intermediate points (8, 9, 10 and 11), and the 
  ///                    y-coordinate of the last two intermediate points (12 and 13).
  ignition::math::Vector3d BilinearInterpolation(double* position, ignition::math::Vector3d * values, double* points) const;
  
  /// \brief  Trilinear interpolation
  /// \param[in]  link_position Vector3 containing the x, y and z-coordinates
  ///                           of the target point.
  ///             values Pointer to an array of size 8 containing the wind values of the 
  ///                    eight points to interpolate from (0, 1, 2, 3, 4, 5, 6 and 7).
  ///             points Pointer to an array of size 14 containing the z-coordinate          
  ///                    of the eight points to interpolate from, the x-coordinate 
  ///                    of the four intermediate points (8, 9, 10 and 11), and the 
  ///                    y-coordinate of the last two intermediate points (12 and 13).
  ignition::math::Vector3d TrilinearInterpolation(ignition::math::Vector3d link_position, ignition::math::Vector3d * values, double* points) const;
  
  gazebo::transport::PublisherPtr wind_force_pub_;
  gazebo::transport::PublisherPtr wind_speed_pub_;

  gazebo::transport::NodePtr node_handle_;

  /// \brief    Gazebo message for sending wind data.
  /// \details  This is defined at the class scope so that it is re-created
  ///           everytime a wind message needs to be sent, increasing performance.
  gz_geometry_msgs::WrenchStamped wrench_stamped_msg_;

  /// \brief    Gazebo message for sending wind speed data.
  /// \details  This is defined at the class scope so that it is re-created
  ///           everytime a wind speed message needs to be sent, increasing performance.
  gz_mav_msgs::WindSpeed wind_speed_msg_;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_BETA_PLUGIN_H

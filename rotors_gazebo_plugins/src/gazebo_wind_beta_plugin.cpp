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

#include "rotors_gazebo_plugins/gazebo_wind_beta_plugin.h"

#include <fstream>
#include <math.h>

#include "ConnectGazeboToRosTopic.pb.h"

namespace gazebo {

GazeboWindBetaPlugin::~GazeboWindBetaPlugin() {
    delete[] wind_queries_;
    delete[] wind_layers_;
    delete[] layer_index_alt_incr_;

    for (int iix=0; iix<nk_x_; iix++) {
        for (int iiy=0; iiy<nk_y_; iiy++) {
            delete[] C_ij_[iix][iiy];
            delete[] rand_phase_[iix][iiy];
        }
        delete[] C_ij_[iix];
        delete[] rand_phase_[iix];
    }
    delete[] rand_phase_;
    delete[] C_ij_;
}

void GazeboWindBetaPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    if (kPrintOnPluginLoad) {
        gzdbg << __FUNCTION__ << "() called." << std::endl;
    }

    // Store the pointer to the model.
    model_ = _model;
    world_ = model_->GetWorld();

    //==============================================//
    //========== READ IN PARAMS FROM SDF ===========//
    //==============================================//

    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzerr << "[gazebo_wind_beta_plugin] Please specify a robotNamespace.\n";

    // Create Gazebo Node.
    node_handle_ = gazebo::transport::NodePtr(new transport::Node());

    // Initialize with default namespace (typically /gazebo/default/).
    node_handle_->Init();

    if (_sdf->HasElement("findWind")) {

        sdf::ElementPtr _sdf_findWind = _sdf->GetElement("findWind");
        sdf::ElementPtr _sdf_query = _sdf_findWind->GetElement("query");

        while (_sdf_query) {
            _sdf_query = _sdf_query->GetNextElement("query");
            ++n_query_;
        }

        gzdbg<<"Found "<<n_query_<<" points to calculate wind \n";
        wind_queries_ = new WindQuery [n_query_];

        _sdf_query = _sdf_findWind->GetElement("query");

        for (int i=0; i<n_query_; i++) {
            if (_sdf_query->HasElement("linkName")) {
                std::string link_name = _sdf_query->Get<std::string>("linkName");
                wind_queries_[i].link = model_->GetLink(link_name);

                if (!wind_queries_[i].link) {
                    gzerr<<"Couldn't find specified link \"" << link_name << "\", won't publish.\n";
                    wind_queries_[i].sdf_valid = false;
                }

            } else if (_sdf_query->HasElement("worldPos")) {
                wind_queries_[i].pos_ned = _sdf_query->Get<ignition::math::Vector3d>("worldPos");

            } else {
                gzwarn<<"Don't know where to query wind, won't publish. \n";
                wind_queries_[i].sdf_valid = false;
            }

            if (_sdf_query->HasElement("windTopic")) {
                wind_queries_[i].wind_topic = _sdf_query->Get<std::string>("windTopic");

            } else {
                gzwarn<<"Please specify windTopic, won't publish wind data. \n";
                wind_queries_[i].sdf_valid = false;
            }

            getSdfParam<std::string>(_sdf_query, "frameId", wind_queries_[i].frame_id, wind_queries_[i].frame_id);

            _sdf_query = _sdf_query->GetNextElement("query");
        }
    }

    if (_sdf->HasElement("windLayer")) {

        sdf::ElementPtr _sdf_windLayer = _sdf->GetElement("windLayer");
        sdf::ElementPtr _sdf_layer = _sdf_windLayer->GetElement("layer");

        while (_sdf_layer) {
            _sdf_layer = _sdf_layer->GetNextElement("layer");
            ++n_layer_;
        }

        gzdbg<<"Found "<<n_layer_<<" wind layers \n";
        wind_layers_ = new WindLayer [n_layer_];
        layer_index_alt_incr_ = new int [n_layer_];
        _sdf_layer = _sdf_windLayer->GetElement("layer");

        for (int i=0; i<n_layer_; i++) {
            getSdfParam<double>(_sdf_layer, "altitude", wind_layers_[i].altitude,
                                wind_layers_[i].altitude);
            /*
               getSdfParam<double>(_sdf_layer, "altUpper", wind_layers_[i].alt_upper,
                                   wind_layers_[i].alt_upper);
               getSdfParam<bool>(_sdf_layer, "useRelAlt", wind_layers_[i].rel_alt,
                                 wind_layers_[i].rel_alt);
               */

            getSdfParam<V3D>(_sdf_layer, "windMeanNED", wind_layers_[i].wind_mean,
                             wind_layers_[i].wind_mean);
            /*
               getSdfParam<double>(_sdf_layer, "windSpeedVariance", wind_layers_[i].wind_speed_variance,
                                   wind_layers_[i].wind_speed_variance);
               getSdfParam<bool>(_sdf_layer, "gustSpeedMean", wind_layers_[i].rel_alt,
                                 wind_layers_[i].rel_alt);
               getSdfParam<bool>(_sdf_layer, "gustSpeedVariance", wind_layers_[i].rel_alt,
                                 wind_layers_[i].rel_alt);
               getSdfParam<bool>(_sdf_layer, "windNedDir", wind_layers_[i].rel_alt,
                                 wind_layers_[i].rel_alt);
               getSdfParam<bool>(_sdf_layer, "gustNedDir", wind_layers_[i].rel_alt,
                                 wind_layers_[i].rel_alt);
               getSdfParam<bool>(_sdf_layer, "gust", wind_layers_[i].rel_alt,
                                 wind_layers_[i].rel_alt);
               */

            _sdf_layer = _sdf_layer->GetNextElement("layer");
        }

        // sort layers, ascending altitude
        std::sort(wind_layers_,wind_layers_+n_layer_,GazeboWindBetaPlugin::SortByAltitude);
    }

    if (_sdf->HasElement("lambdaMin"))
        lambda_min_ = _sdf->Get<double>("lambdaMin");

    if (_sdf->HasElement("lengthScale"))
        L_ = _sdf->Get<double>("lengthScale");

    if (_sdf->HasElement("fourierNNN")) {
        ignition::math::Vector3i fourierNNN = _sdf->Get<ignition::math::Vector3i>("fourierNNN");
        nk_x_ = std::max(1,fourierNNN.X());
        nk_y_ = std::max(1,fourierNNN.Y());
        nk_z_ = std::max(1,fourierNNN.Z());

    } else {
        nk_x_ = 15;
        nk_y_ = 15;
        nk_z_ = 15;
    }

    // ensure odd number of components
    if(nk_x_%2==0)
        nk_x_+=1;

    if(nk_y_%2==0)
        nk_y_+=1;

    if(nk_z_%2==0)
        nk_z_+=1;

    if (_sdf->HasElement("sigma"))
        sigma_ = _sdf->Get<double>("sigma");

    if (_sdf->HasElement("trnspVelTurbNED"))
        trnsp_vel_turb_ = _sdf->Get<V3D>("trnspVelTurbNED");

    if (_sdf->HasElement("customWindFieldPath")) {
        gzdbg << "[gazebo_wind_plugin] Using custom wind field from text file.\n";
        // Get the wind field text file path, read it and save data.
        std::string custom_wind_field_path;
        getSdfParam<std::string>(_sdf, "customWindFieldPath", custom_wind_field_path,
                                 custom_wind_field_path);

        getSdfParam<double>(_sdf, "windFieldXOffset", wf_offset.X(), wf_offset.X());
        getSdfParam<double>(_sdf, "windFieldYOffset", wf_offset.Y(), wf_offset.Y());
        getSdfParam<double>(_sdf, "windFieldZOffset", wf_offset.Z(), wf_offset.Z());

        ReadCustomWindFieldCSV(custom_wind_field_path);
        ProcessCustomWindField();
    }

    // Set up Fourier Simulation
    std::random_device rd;
    std::mt19937 gen(rd());
    //std::uniform_real_distribution<> r_phase(0.0, 2*M_PI);
    std::normal_distribution<> re(0,1.0/sqrt(2));   // complex variance = var(re)+var(im) needs= 1
    std::normal_distribution<> im(0,1.0/sqrt(2));

    C_ij_ = new M3C** [nk_x_];
    rand_phase_ = new V3C** [nk_x_];

    for (int iix=0; iix<nk_x_; iix++) {
        C_ij_[iix] = new M3C* [nk_y_];
        rand_phase_[iix] = new V3C* [nk_y_];

        for (int iiy=0; iiy<nk_y_; iiy++) {
            C_ij_[iix][iiy] = new M3C [nk_z_];
            rand_phase_[iix][iiy] = new V3C [nk_z_];
        }
    }

    dk_x_ = 2*M_PI/lambda_min_/nk_x_;
    dk_y_ = 2*M_PI/lambda_min_/nk_y_;
    dk_z_ = 2*M_PI/lambda_min_/nk_z_;

    for (int iix=0; iix<nk_x_; iix++) {
        for (int iiy=0; iiy<nk_y_; iiy++) {
            for (int iiz=0; iiz<nk_z_; iiz++) {
                V3D k((iix-(nk_x_-1)/2)*dk_x_, (iiy-(nk_y_-1)/2)*dk_y_, (iiz-(nk_z_-1)/2)*dk_z_);

                /*
                rand_phase_[iix][iiy][iiz].Set(std::exp((std::complex<double>)1i*r_phase(gen)),
                                               std::exp((std::complex<double>)1i*r_phase(gen)),
                                               std::exp((std::complex<double>)1i*r_phase(gen)));
                */
                rand_phase_[iix][iiy][iiz].Set((std::complex<double>)re(gen) + (std::complex<double>)1i*im(gen),
                                               (std::complex<double>)re(gen) + (std::complex<double>)1i*im(gen),
                                               (std::complex<double>)re(gen) + (std::complex<double>)1i*im(gen));

                // only use k's inside spherical volume, otherwise set contribution to zero
                if (k.Length()>0 && k.Length()<M_PI/lambda_min_) {
                    GazeboWindBetaPlugin::SpectralTensorIsoIncDec(C_ij_[iix][iiy][iiz], k, L_, sigma_);
                    C_ij_[iix][iiy][iiz] = C_ij_[iix][iiy][iiz]*sqrt(dk_x_*dk_y_*dk_z_);

                } else {
                    C_ij_[iix][iiy][iiz].Set(0,0,0,0,0,0,0,0,0);
                }
            }
        }
    }

    /*
     gzdbg<<"C_ij_[2][3][4]: \n"; PrintMatrix(C_ij_[2][3][4]); gzdbg<<"\n";
     gzdbg<<"C_ij_[8][7][13]: \n"; PrintMatrix(C_ij_[8][7][13]); gzdbg<<"\n";
     gzdbg<<"C_ij_[0][1][13]: \n"; PrintMatrix(C_ij_[0][1][13]); gzdbg<<"\n";
     gzdbg<<"dk_x_: "<<dk_x_<<" dk_y_: "<<dk_y_<<" dk_z_: "<<dk_z_<<"\n";
     gzdbg<<"lambdaMin: "<<lambda_min_<<" lengthScale: "<<L_<<" sigma: "<<sigma_<<"\n";
     */

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboWindBetaPlugin::OnUpdate, this, _1));
}

void GazeboWindBetaPlugin::PrintMatrix(M3C M) {

    gzdbg<<std::real(M(0,0))<<" "<<std::real(M(0,1))<<" "<<std::real(M(0,2))<<"\n";
    gzdbg<<std::real(M(1,0))<<" "<<std::real(M(1,1))<<" "<<std::real(M(1,2))<<"\n";
    gzdbg<<std::real(M(2,0))<<" "<<std::real(M(2,1))<<" "<<std::real(M(2,2))<<"\n";
}

// This gets called by the world update start event.
void GazeboWindBetaPlugin::OnUpdate(const common::UpdateInfo& _info) {

    if (!pubs_and_subs_created_) {
        CreatePubsAndSubs();
        pubs_and_subs_created_ = true;
    }

    for (int iiq=0; iiq<n_query_; iiq++) {

        ignition::math::Pose3d link_pose = wind_queries_[iiq].link->WorldPose();
        V3D pos = link_pose.Pos();

        // wind component from layered model (from xacro)
        V3D vel_lyr = V3D(0,0,0);

        // wind component from turbulent field
        V3C vel_turb = V3C(0,0,0);
        V3C dvel_x_turb = V3C(0,0,0);
        V3C dvel_y_turb = V3C(0,0,0);
        V3C dvel_z_turb = V3C(0,0,0);

        // wind component from custom field (from file)
        V3D vel_cust_field = V3D(0,0,0);
        V3D dvdx_cust_field = V3D(0,0,0);
        V3D dvdy_cust_field = V3D(0,0,0);
        V3D dvdz_cust_field = V3D(0,0,0);

        // Layered mean velocity field from xacro (uniform in x-y)
        if (n_layer_>=2) {
            for (int iil=0; iil<n_layer_-1; iil++) {
                // below lowest specified layer
                if (pos.Z()<wind_layers_[iil].altitude) {
                    if (pos.Z()<=0)
                        vel_lyr = V3D(0,0,0);
                    else
                        vel_lyr = wind_layers_[iil].wind_mean*pos.Z()/wind_layers_[iil].altitude;
                    break;
                }

                // in range of specified layers
                if (pos.Z()>=wind_layers_[iil].altitude && pos.Z()<wind_layers_[iil+1].altitude) {
                    vel_lyr = wind_layers_[iil].wind_mean +
                            (wind_layers_[iil+1].wind_mean-wind_layers_[iil].wind_mean)*
                            (pos.Z()-wind_layers_[iil].altitude)/(wind_layers_[iil+1].altitude-wind_layers_[iil].altitude);
                    break;
                }

                // on or above top layer
                if (iil==n_layer_-2)
                    vel_lyr = wind_layers_[iil+1].wind_mean;
            }

        } else if (n_layer_==1) {
            if (pos.Z()<=0)
                vel_lyr = V3D(0,0,0);
            else if (pos.Z()<=wind_layers_[0].altitude)
                vel_lyr = wind_layers_[0].wind_mean*pos.Z()/wind_layers_[0].altitude;
            else
                vel_lyr = wind_layers_[0].wind_mean;

        } else {
            vel_lyr = V3D(0,0,0);
        }

        // ToDo: more robust trafo NED to world
        vel_lyr.Y() = -vel_lyr.Y();
        vel_lyr.Z() = -vel_lyr.Z();

        // Turbulence field (Fourier simulation)
#if GAZEBO_MAJOR_VERSION >= 9
        V3D pos_off = -trnsp_vel_turb_*world_->SimTime().Double();
#else
        V3D pos_off = -trnsp_vel_turb_*world_->GetSimTime().Double();
#endif

        V3D pos_q = pos + pos_off;
        V3D pos_q_dx = pos + pos_off + V3D(1,0,0);
        V3D pos_q_dy = pos + pos_off + V3D(0,1,0);
        V3D pos_q_dz = pos + pos_off + V3D(0,0,1);

        for (int iix=0; iix<nk_x_; iix++) {
            for (int iiy=0; iiy<nk_y_; iiy++) {
                for (int iiz=0; iiz<nk_z_; iiz++) {
                    V3D k((iix-(nk_x_-1)/2)*dk_x_, (iiy-(nk_y_-1)/2)*dk_y_, (iiz-(nk_z_-1)/2)*dk_z_);
                    V3C arg = C_ij_[iix][iiy][iiz]*rand_phase_[iix][iiy][iiz];
                    vel_turb = vel_turb + std::exp((std::complex<double>)-1i*k.Dot(pos_q))*arg;
                    dvel_x_turb = dvel_x_turb + std::exp((std::complex<double>)-1i*k.Dot(pos_q_dx))*arg;
                    dvel_y_turb = dvel_y_turb + std::exp((std::complex<double>)-1i*k.Dot(pos_q_dy))*arg;
                    dvel_z_turb = dvel_z_turb + std::exp((std::complex<double>)-1i*k.Dot(pos_q_dz))*arg;
                }
            }
        }

        // ToDo: check again own derivations...
        // ToDo: modify turbulence according to local turbulent kinetic energy trb_kin_nrg_...
        double sr2 = sqrt(2);
        vel_turb *= sr2;
        dvel_x_turb *= sr2;
        dvel_y_turb *= sr2;
        dvel_z_turb *= sr2;

        // scale turbulence in proximity of ground (z=0)
        double red_hrz_z = sqrt(ignition::math::clamp(pos.Z()/10.0,0.0,1.0));
        double red_vrt_z = sqrt(ignition::math::clamp(pos.Z()/20.0,0.0,1.0));

        // Custom mean velocity field
        if (wind_is_valid_) {

            // Calculate the x, y index of the grid points with x, y-coordinate
            // just smaller than or equal to aircraft x, y position.
            std::size_t x_inf = floor((pos.X() - wf_offset.X() - min_x_) / res_x_);
            std::size_t y_inf = floor((pos.Y() - wf_offset.Y() - min_y_) / res_y_);
            std::size_t z_inf = floor((pos.Z() - wf_offset.Z() - min_z_) / res_z_);

            // Calculate the x, y index of the grid points with x, y-coordinate just
            // greater than the aircraft x, y position.
            std::size_t x_sup = x_inf + 1u;
            std::size_t y_sup = y_inf + 1u;
            std::size_t z_sup = z_inf + 1u;

            // Save in an array the x, y index of each of the eight grid points
            // enclosing the aircraft.
            constexpr unsigned int n_vertices = 8;
            std::size_t idx_x[n_vertices] = {x_inf,x_inf,x_inf,x_inf,x_sup,x_sup,x_sup,x_sup}; // {x_inf, x_inf, x_sup, x_sup, x_inf, x_inf, x_sup, x_sup};
            std::size_t idx_y[n_vertices] = {y_inf,y_inf,y_sup,y_sup,y_inf,y_inf,y_sup,y_sup}; // {y_inf, y_inf, y_inf, y_inf, y_sup, y_sup, y_sup, y_sup};
            std::size_t idx_z[n_vertices] = {z_inf,z_sup,z_inf,z_sup,z_inf,z_sup,z_inf,z_sup}; // {z_inf, z_sup, z_sup, z_inf, z_inf, z_sup, z_sup, z_inf};

            // Check if aircraft is out of wind field or not, and act accordingly.
            if (x_inf >= 0u && y_inf >= 0u && z_inf >= 0u &&
                    x_sup <= (n_x_ - 2u) && y_sup <= (n_y_ - 2u) && z_sup <= (n_z_ - 2u)) {

                // Extract the wind velocities corresponding to each vertex.
                V3D wind_at_vertices[n_vertices];
                V3D dwfdx_at_vertices[n_vertices];
                V3D dwfdy_at_vertices[n_vertices];
                V3D dwfdz_at_vertices[n_vertices];

                for (std::size_t i = 0u; i < n_vertices; ++i) {
                    wind_at_vertices[i].X() = wf_[idx_x[i] + idx_y[i] * n_x_ + idx_z[i] * n_x_ * n_y_].X();
                    wind_at_vertices[i].Y() = wf_[idx_x[i] + idx_y[i] * n_x_ + idx_z[i] * n_x_ * n_y_].Y();
                    wind_at_vertices[i].Z() = wf_[idx_x[i] + idx_y[i] * n_x_ + idx_z[i] * n_x_ * n_y_].Z();

                    dwfdx_at_vertices[i] = dwfdx_[idx_x[i] + idx_y[i] * n_x_ + idx_z[i] * n_x_ * n_y_];
                    dwfdy_at_vertices[i] = dwfdy_[idx_x[i] + idx_y[i] * n_x_ + idx_z[i] * n_x_ * n_y_];
                    dwfdz_at_vertices[i] = dwfdz_[idx_x[i] + idx_y[i] * n_x_ + idx_z[i] * n_x_ * n_y_];
                }

                // Extract the relevant coordinate of every point needed for trilinear
                double interpolation_points[14] = { min_z_+idx_z[0]*res_z_,
                                                    min_z_+idx_z[1]*res_z_,
                                                    min_z_+idx_z[2]*res_z_,
                                                    min_z_+idx_z[3]*res_z_,
                                                    min_z_+idx_z[4]*res_z_,
                                                    min_z_+idx_z[5]*res_z_,
                                                    min_z_+idx_z[6]*res_z_,
                                                    min_z_+idx_z[7]*res_z_,
                                                    min_y_+idx_y[0]*res_y_,
                                                    min_y_+idx_y[2]*res_y_,
                                                    min_y_+idx_y[4]*res_y_,
                                                    min_y_+idx_y[6]*res_y_,
                                                    min_x_+idx_x[0]*res_x_,
                                                    min_x_+idx_x[4]*res_x_ };

                // Interpolate wind velocity and and gradients at aircraft position.

                vel_cust_field  = TrilinearInterpolation(pos-wf_offset, wind_at_vertices, interpolation_points);
                dvdx_cust_field = TrilinearInterpolation(pos-wf_offset, dwfdx_at_vertices, interpolation_points);
                dvdy_cust_field = TrilinearInterpolation(pos-wf_offset, dwfdy_at_vertices, interpolation_points);
                //dvdz_cust_field = TrilinearInterpolation(pos-wf_offset, dwfdz_at_vertices, interpolation_points);
            }
        }

        // Superpose everything
        wind_queries_[iiq].wind_msg.mutable_pos_ned()->set_x(std::real(pos.X()));
        wind_queries_[iiq].wind_msg.mutable_pos_ned()->set_y(std::real(pos.Y()));
        wind_queries_[iiq].wind_msg.mutable_pos_ned()->set_z(std::real(pos.Z()));

        wind_queries_[iiq].wind_msg.mutable_wind_ned()->set_x(std::real(vel_turb.X())*red_hrz_z+vel_lyr.X()+vel_cust_field.X());
        wind_queries_[iiq].wind_msg.mutable_wind_ned()->set_y(std::real(vel_turb.Y())*red_hrz_z+vel_lyr.Y()+vel_cust_field.Y());
        wind_queries_[iiq].wind_msg.mutable_wind_ned()->set_z(std::real(vel_turb.Z())*red_vrt_z+vel_lyr.Z()+vel_cust_field.Z());

        wind_queries_[iiq].wind_msg.mutable_wind_grad_ned()->set_xx(std::real(dvel_x_turb.X()-vel_turb.X())*red_hrz_z+dvdx_cust_field.X());
        wind_queries_[iiq].wind_msg.mutable_wind_grad_ned()->set_xy(std::real(dvel_y_turb.X()-vel_turb.X())*red_hrz_z+dvdy_cust_field.X());
        wind_queries_[iiq].wind_msg.mutable_wind_grad_ned()->set_xz(std::real(dvel_z_turb.X()-vel_turb.X())*red_hrz_z+dvdz_cust_field.X());
        wind_queries_[iiq].wind_msg.mutable_wind_grad_ned()->set_yx(std::real(dvel_x_turb.Y()-vel_turb.Y())*red_hrz_z+dvdx_cust_field.Y());
        wind_queries_[iiq].wind_msg.mutable_wind_grad_ned()->set_yy(std::real(dvel_y_turb.Y()-vel_turb.Y())*red_hrz_z+dvdy_cust_field.Y());
        wind_queries_[iiq].wind_msg.mutable_wind_grad_ned()->set_yz(std::real(dvel_z_turb.Y()-vel_turb.Y())*red_hrz_z+dvdz_cust_field.Y());
        wind_queries_[iiq].wind_msg.mutable_wind_grad_ned()->set_zx(std::real(dvel_x_turb.Z()-vel_turb.Z())*red_vrt_z+dvdx_cust_field.Z());
        wind_queries_[iiq].wind_msg.mutable_wind_grad_ned()->set_zy(std::real(dvel_y_turb.Z()-vel_turb.Z())*red_vrt_z+dvdy_cust_field.Z());
        wind_queries_[iiq].wind_msg.mutable_wind_grad_ned()->set_zz(std::real(dvel_z_turb.Z()-vel_turb.Z())*red_vrt_z+dvdz_cust_field.Z());

        wind_queries_[iiq].wind_pub->Publish(wind_queries_[iiq].wind_msg);

        if (counter%100==0) {
            gzdbg<<"turb: "<<std::real(vel_turb.X())<<" "<<std::real(vel_turb.Y())<<" "<<std::real(vel_turb.Z())<<"\n";
            gzdbg<<"mean: "<<vel_lyr.X()<<" "<<vel_lyr.Y()<<" "<<vel_lyr.Z()<<"\n";
            gzdbg<<"alt: "<<pos.Z()<<"\n";
            counter = 0;
        }
    }

    counter++;

}

void GazeboWindBetaPlugin::CreatePubsAndSubs() {
    // Create temporary "ConnectGazeboToRosTopic" publisher and message.
    gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
            node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
                "~/" + kConnectGazeboToRosSubtopic, 1);

    gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;

    // ============================================ //
    // ========= WRENCH STAMPED MSG SETUP ========= //
    // ============================================ //
    /*
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
  */
    // ============================================ //
    // ========== WIND SPEED MSG SETUP ============ //
    // ============================================ //

    for (int i=0; i<n_query_; i++) {
        wind_queries_[i].wind_pub = node_handle_->Advertise<gz_mav_msgs::WindSpeedBeta>(
                    "~/" + namespace_ + "/" + wind_queries_[i].wind_topic, 1);

        connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                         wind_queries_[i].wind_topic);
        connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                      wind_queries_[i].wind_topic);
        // connect_gazebo_to_ros_topic_msg.set_msgtype(
        //     gz_std_msgs::ConnectGazeboToRosTopic::WIND_SPEED);
        // connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
        //                                          true);
    }
}

void  GazeboWindBetaPlugin::KarmanEnergySpectrum(double &E, V3D k, double L, double sigma) {

    // Evaluates Karman energy spectrum at spatial frequency k
    //
    // Ref: J.Mann, "The spatial structure of neutral atmospheric surface-layer turbulence",
    //      J. Fluid. Mech. (1994), vol. 273, p.146-p.147
    //
    // 1.4528 = 55/9*Gamma(5/6)/Gamma(1/3)/sqrt(pi)

    double LK2 = L*L*k.Length()*k.Length();
    E=1.4528*L*sigma*sigma*LK2*LK2/pow(1+LK2,17.0/6.0);

}

void  GazeboWindBetaPlugin::SpectralTensorIsoInc(M3D &Phi, V3D k, double L, double sigma) {

    // Returns spectral tensor Phi (Fourier transform of covariance tensor) at spatial frequency k,
    // using the Karman spectral energy.
    //
    // Ref: J.Mann, "The spatial structure of neutral atmospheric surface-layer turbulence",
    //      J. Fluid. Mech. (1994), vol. 273, p.146-p.147

    double E;
    GazeboWindBetaPlugin::KarmanEnergySpectrum(E, k, L, sigma);

    Phi.Set( k[1]*k[1]+k[2]*k[2], -k[0]*k[1], -k[0]*k[2],
            -k[1]*k[0], k[0]*k[0]+k[2]*k[2], -k[1]*k[2],
            -k[2]*k[0], -k[2]*k[1], k[0]*k[0]+k[1]*k[1]);

    Phi = E/(4*M_PI*pow(k.Length(),4))*Phi;
}

void  GazeboWindBetaPlugin::SpectralTensorIsoIncDec(M3C &A, V3D k, double L, double sigma) {

    // Returns decomposition A of spectral tensor Phi (Fourier transform of covariance tensor)
    // at spatial frequency k, using the Karman spectral energy. A'A=Phi
    //
    // Ref: J.Mann, "Wind field simulation", Prop. Engng. Mech. (1998), vol. 13, p.280

    double E;
    GazeboWindBetaPlugin::KarmanEnergySpectrum(E, k, L, sigma);

    A.Set(0, k[2], -k[1],
            -k[2], 0, k[0],
            k[1], -k[0], 0);

    A = sqrt(E/(4*M_PI))/(k.Length()*k.Length())*A;
}

/*
void GazeboWindBetaPlugin::LonPsdKarman(double * omega, double * phi, double n, double l, double s){

     double A = 2*s*s*l/M_PI;
     for (int i = 0; i < n; i++) {
          double B = 1.339*l*omega[i];
          phi[i] = A * 1/(pow(1+B*B,5/6));
     }
}

void GazeboWindBetaPlugin::LatVertPsdKarman(double * omega, double * phi, double n, double l, double s){

     double A = s*s*l/M_PI;
     for (int i = 0; i < n; i++) {
          double B = 1.339*l*omega[i];
          phi[i] = A * (1+8/3*B*B)/(pow(1+B*B,11/6));
     }
}
*/

void GazeboWindBetaPlugin::ProcessCustomWindField() {

    if (wind_is_valid_) {

        for (int iix=0; iix<n_x_-1; iix++) {
            for (int iiy=0; iiy<n_y_-1; iiy++) {
                for (int iiz=0; iiz<n_z_-1; iiz++) {

                    int i0  = iix+iiy*n_x_+iiz*n_x_*n_y_;
                    int idx = (iix+1)+iiy*n_x_+iiz*n_x_*n_y_;
                    int idy = iix+(iiy+1)*n_x_+iiz*n_x_*n_y_;
                    int idz = iix+iiy*n_x_+(iiz+1)*n_x_*n_y_;

                    dwfdx_.push_back((wf_[idx]-wf_[i0])/res_x_);
                    dwfdy_.push_back((wf_[idy]-wf_[i0])/res_y_);
                    dwfdz_.push_back((wf_[idz]-wf_[i0])/res_z_);
                }
            }
        }
    }
}

void GazeboWindBetaPlugin::ReadCustomWindFieldCSV(std::string& custom_wind_field_path) {

    std::ifstream fin;
    fin.open(custom_wind_field_path);

    if (fin.is_open()) {

        std::vector<float> x_coord;
        std::vector<float> y_coord;
        std::vector<float> z_coord;
        int n_grid = 0;
        double data[11];
        char comma;
        bool stop = false;
        bool start = true;
        bool got_n_x = false;
        bool got_n_y = false;
        bool got_n_z = false;

        // Ignore header
        fin.ignore(128, '\n');

        for (;;) {

            // read 11 numbers in current line and break on error
            for (int iid=0; iid<10; iid++) {
                fin >> data[iid] >> comma;
                if (fin.fail() || fin.eof())
                    stop = true;
            }

            fin >> data[10];

            if (fin.fail() || fin.eof())
                stop = true;

            if (stop)
                break;

            ++n_grid;

            wf_.push_back(V3D(data[1],data[2],data[3]));
            trb_kin_nrg_.push_back(data[5]);

            if (start) {
                min_x_ = data[8];
                min_y_ = data[9];
                min_z_ = data[10];
                n_x_ = 1;
                n_y_ = 1;
                n_z_ = 1;
                start = false;

            } else {
                if (!got_n_x && data[8]!=min_x_) {
                    ++n_x_;
                    x_coord.push_back(data[8]);

                } else if (data[8]==min_x_) {
                    got_n_x=true;
                    if (!got_n_y && data[9]!=min_y_) {
                        ++n_y_;
                        y_coord.push_back(data[9]);

                    } else if (data[9]==min_y_) {
                        got_n_y=true;
                        if (!got_n_z && data[10]!=min_z_) {
                            ++n_z_;
                            z_coord.push_back(data[10]);

                        } else {
                            got_n_z=true;
                        }
                    }
                }
            }

            // ignore rest of line
            fin.ignore(128, '\n');
        }

        gzerr<<"Custom wind field: nx="<<n_x_<<" | ny="<<n_y_<<" | nz="<<n_z_<<"\n";
        gzerr<<"Custom wind field: min_x="<<min_x_<<" | min_y="<<min_y_<<" | min_z="<<min_z_<<"\n";

        if (n_x_>1 && n_y_>1 && n_z_>1) {
            wind_is_valid_ = true;
            res_x_ = x_coord[1]-x_coord[0];
            res_y_ = y_coord[1]-y_coord[0];
            res_z_ = z_coord[1]-z_coord[0];

        } else {
            gzerr<<"Custom wind field not valid, too few grid points (min 2x2x2).\n";
        }

    } else {
        gzerr<<"Could not open custom wind field *.csv file. Check file permissions.\n";
    }

}

/*
void GazeboWindBetaPlugin::ReadCustomWindField(std::string& custom_wind_field_path) {
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

        gzdbg << "[gazebo_wind_plugin] Successfully read custom wind field from text file.\n";



    } else {
        gzerr << "[gazebo_wind_plugin] Could not open custom wind field text file.\n";
    }



}
*/

ignition::math::Vector3d GazeboWindBetaPlugin::LinearInterpolation(
        double position, ignition::math::Vector3d * values, double* points) const {
    ignition::math::Vector3d value = values[0] + (values[1] - values[0]) /
            (points[1] - points[0]) * (position - points[0]);
    return value;
}

ignition::math::Vector3d GazeboWindBetaPlugin::BilinearInterpolation(
        double* position, ignition::math::Vector3d * values, double* points) const {
    ignition::math::Vector3d intermediate_values[2] = { LinearInterpolation(
                                                        position[1], &(values[0]), &(points[0])),
                                                        LinearInterpolation(
                                                        position[1], &(values[2]), &(points[2])) };
    ignition::math::Vector3d value = LinearInterpolation(
                position[0], intermediate_values, &(points[4]));
    return value;
}

ignition::math::Vector3d GazeboWindBetaPlugin::TrilinearInterpolation(
        ignition::math::Vector3d link_position, ignition::math::Vector3d * values, double* points) const {
    double position[3] = {link_position.X(),link_position.Y(),link_position.Z()};

    /*
    gzerr<<"Trilipol:   Pos: "<<position[0]<<","<<position[1]<<","<<position[2]<<"\n";
    std::stringstream ss;
    ss<<"         Points: ";
    for(int i=0; i<14; i++){
        ss<<points[i]<<", ";
    }
    gzerr<<ss.str()<<"\n";
    gzerr<<"           Wind: "<<values[0].X()<<", "<<values[1].X()<<", "<<values[2].X()<<", "<<values[3].X()<<", "<<values[4].X()<<", "<<values[5].X()<<", "<<values[6].X()<<", "<<values[7].X()<<"\n";
    gzerr<<"                 "<<values[0].Y()<<", "<<values[1].Y()<<", "<<values[2].Y()<<", "<<values[3].Y()<<", "<<values[4].Y()<<", "<<values[5].Y()<<", "<<values[6].Y()<<", "<<values[7].Y()<<"\n";
    gzerr<<"                 "<<values[0].Z()<<", "<<values[1].Z()<<", "<<values[2].Z()<<", "<<values[3].Z()<<", "<<values[4].Z()<<", "<<values[5].Z()<<", "<<values[6].Z()<<", "<<values[7].Z()<<"\n";

    gzerr<<"                 nx="<<n_x_<<" | ny="<<n_y_<<" | nz="<<n_z_<<"\n";
    gzerr<<"                 min_x="<<min_x_<<" | min_y="<<min_y_<<" | min_z="<<min_z_<<"\n";
    gzerr<<"                 res_x="<<res_x_<<" | res_y="<<res_y_<<" | res_z="<<res_z_<<"\n";
    */

    ignition::math::Vector3d intermediate_values[4] = { LinearInterpolation(
                                                        position[2], &(values[0]), &(points[0])),
                                                        LinearInterpolation(
                                                        position[2], &(values[2]), &(points[2])),
                                                        LinearInterpolation(
                                                        position[2], &(values[4]), &(points[4])),
                                                        LinearInterpolation(
                                                        position[2], &(values[6]), &(points[6])) };
    ignition::math::Vector3d value = BilinearInterpolation(
                &(position[0]), intermediate_values, &(points[8]));
    /*
    gzerr<<"                 vx="<<value.X()<<" | vy="<<value.Y()<<" | vz="<<value.Z()<<"\n";
    */

    return value;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboWindBetaPlugin);

}  // namespace gazebo

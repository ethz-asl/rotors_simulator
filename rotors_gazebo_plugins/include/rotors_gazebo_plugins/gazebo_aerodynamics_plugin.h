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

#ifndef _GAZEBO_LIFT_DRAG_PLUGIN_HH_
#define _GAZEBO_LIFT_DRAG_PLUGIN_HH_

#include <mutex>

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include <ignition/math.hh>

#include "common.h"
#include "uav_parameters.h"

#include <math.h>
#include <Eigen/Eigen>
#include "gazebo/msgs/msgs.hh"
#include <stdio.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include "PropulsionSlipstream.pb.h"
#include "VisVectorArray.pb.h"
#include "WindSpeedBeta.pb.h"
#include "ConnectGazeboToRosTopic.pb.h"
#include <Float32.pb.h>
#include <iostream>
#include <fstream>

#include <algorithm>

#include "gazebo/common/Assert.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include <iomanip>

#define AIRFOIL_N_VIS_VEC 6
#define BODY_N_VIS_VEC 2

namespace gazebo
{

typedef ignition::math::Vector3d V3D;
typedef ignition::math::Matrix3<double> M3D;

typedef const boost::shared_ptr<const gz_visualization_msgs::VisVectorArray> GzVisVectorArrayMsgPtr;
typedef const boost::shared_ptr<const gz_mav_msgs::PropulsionSlipstream> PropulsionSlipstreamPtr;
typedef const boost::shared_ptr<const gz_mav_msgs::WindSpeedBeta> WindPtr;
typedef const boost::shared_ptr<const gz_std_msgs::Float32> Float32Ptr;
static const std::string kDefaultPropulsionSlipstreamSubTopic = "/propulsion_slipstream";
static constexpr double kDefaultRhoAir = 1.2041; // air density 1.2041 kg/m³ (dry, @20 °C, 101.325 kPa)


class GazeboAerodynamics : public ModelPlugin
{
    /// \brief Constructor.
public:
    GazeboAerodynamics():
        ModelPlugin(){gzdbg<<"gazebo_aerodynamics constructed"<<std::endl;}
    ~GazeboAerodynamics();

protected:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void OnUpdate();

private:
    std::string namespace_;
    transport::NodePtr node_handle_;
    physics::WorldPtr world;
    physics::ModelPtr model;
    physics::LinkPtr link;
    event::ConnectionPtr updateConnection;

    double rho_ = kDefaultRhoAir; // air density 1.2041 kg/m³ (dry, @20 °C, 101.325 kPa)

    struct ControlSurface {
        ControlSurface():
            cs_ref(0.0){}

        physics::JointPtr control_joint = nullptr;
        transport::SubscriberPtr control_ref_sub = nullptr;
        std::string cs_ref_topic;
        bool from_topic = false;
        std::atomic<double> cs_ref;
        V3D control_joint_rad_to_cl = V3D(0,0,0);   // dC_L/dCS slope, [1/rad]
        V3D control_joint_rad_to_cd = V3D(0,0,0);   // dC_D/dCS slope, [1/rad]
        V3D control_joint_rad_to_cm = V3D(0,0,0);   // dC_M/dCS slope, [1/rad]
        double d_aoa_b_d_delta_cs = 0;                // dAlpha_break/dCs slope [rad/rad]
        void Callback(Float32Ptr& reference){
            cs_ref = (double)reference->data();
        }
    };

    struct Wind {
        Wind(){}

        transport::SubscriberPtr wind_sub_ = nullptr;
        std::mutex wind_lock;
        std::string wind_topic;

        V3D pos_ned = V3D(0,0,0);
        V3D wind_ned = V3D(0,0,0);
        M3D wind_grad_ned = M3D(0,0,0,0,0,0,0,0,0);

        void Callback(WindPtr& wind){
            std::unique_lock<std::mutex> lock(wind_lock);

            pos_ned = V3D(wind->pos_ned().x(),
                          wind->pos_ned().y(),
                          wind->pos_ned().z());

            wind_ned = V3D(wind->wind_ned().x(),
                           wind->wind_ned().y(),
                           wind->wind_ned().z());

            wind_grad_ned = M3D(wind->wind_grad_ned().xx(),
                                wind->wind_grad_ned().xy(),
                                wind->wind_grad_ned().xz(),
                                wind->wind_grad_ned().yx(),
                                wind->wind_grad_ned().yy(),
                                wind->wind_grad_ned().yz(),
                                wind->wind_grad_ned().zx(),
                                wind->wind_grad_ned().zy(),
                                wind->wind_grad_ned().zz());
            lock.unlock();
        }

        V3D GetWind(V3D p_cp){
            std::unique_lock<std::mutex> lock(wind_lock);   //necessary? atomic V3D?
            V3D wind_local = wind_ned + wind_grad_ned*(p_cp-pos_ned);
            //V3D wind_local = wind_grad_ned*(p_cp-pos_ned);
            lock.unlock();
            return wind_local;
        }
    };

    struct Slipstream {
        Slipstream(){}

        transport::SubscriberPtr propulsion_slipstream_sub_ = nullptr;
        std::mutex slipstream_lock;
        std::string slpstr_topic;

        V3D p_rot = V3D(0,0,0);
        V3D d_wake = V3D(0,0,0);
        V3D d_wake_e = V3D(0,0,0);
        V3D v_ind_d = V3D(0,0,0);
        V3D v_ind_e = V3D(0,0,0);

        double r_rot = 0;

        void Callback(PropulsionSlipstreamPtr& propulsion_slipstream){
            std::unique_lock<std::mutex> lock(slipstream_lock);
            // position of rotordisk center wrt world, expressed in world frame [m]
            p_rot = V3D(propulsion_slipstream->rotor_pos().x(),
                        propulsion_slipstream->rotor_pos().y(),
                        propulsion_slipstream->rotor_pos().z());

            // wake direction expressed in world frame [-]
            d_wake = V3D(propulsion_slipstream->wake_dir().x(),
                         propulsion_slipstream->wake_dir().y(),
                         propulsion_slipstream->wake_dir().z());
            d_wake_e = d_wake.Normalized();

            // induced velocity at rotordisk, expressed in world frame [m/s]
            v_ind_d = V3D(propulsion_slipstream->ind_vel_disk().x(),
                          propulsion_slipstream->ind_vel_disk().y(),
                          propulsion_slipstream->ind_vel_disk().z());

            // induced velocity at end of wake (Note: not necessarily 0!)
            v_ind_e = V3D(propulsion_slipstream->ind_vel_end().x(),
                          propulsion_slipstream->ind_vel_end().y(),
                          propulsion_slipstream->ind_vel_end().z());

            // propeller/wake diameter
            r_rot = propulsion_slipstream->prop_diam()/2;
            lock.unlock();
        }

        V3D GetIndVel(V3D p_cp){

            V3D v_ind; // induced velocity at cp (e.g. due to slipstream)
            std::unique_lock<std::mutex> lock(slipstream_lock);
            V3D p_r2cp_ = p_cp - p_rot;
            double off_a_ = d_wake_e.Dot(p_r2cp_);                 // axial distance in wake (d1 in report)
            double off_p_ = (off_a_*d_wake_e-p_r2cp_).Length();    // radial distance to wake centerline (d2 in report)


            if(off_a_>0 && d_wake.Length()>off_a_){
                // if in zone of slipstream influence
                double k_a_ = (d_wake.Length()-off_a_)/d_wake.Length();    // axial direction interpolation weight
                double r_rot_exp = (2-1*k_a_)*r_rot;
                double k_p_ = 1-pow((off_p_/r_rot_exp),4);
                k_p_ = ignition::math::clamp(k_p_,0.0,1.0);                // radial distance downscaling
                v_ind = k_p_*(k_a_*v_ind_d+(1-k_a_)*v_ind_e);   // induced velocity at airfoil segment cp
            }

            lock.unlock();
            return v_ind;
        }
    };

    struct Segment {
        Segment(){}

        physics::LinkPtr ref_link = nullptr;  // reference link for force/torque calculation
        physics::LinkPtr act_link = nullptr;  // link to apply force/torque to

        // To include hyteresis in future implementation, currently not used
        double alpha_prev = 0;
        double alpha_dot = 0;
        bool cl_hist_up = true;

        AerodynamicParameters aero_params;

        V3D cp = V3D(0,0,0);    // Center of pressure wrt link frame, expressed in link frame
        V3D fwd = V3D(1,0,0);
        V3D upwd = V3D(0,0,1);
        double seg_area;

        double seg_chord;
        double ellp_mult;

        ControlSurface * cs;
        int n_cs = 0;

        /// \brief Quantities to model propeller/rotor wake/slipstream

        Slipstream * slpstr;
        V3D v_ind_cp; // induced velocity at cp (e.g. due to slipstream)
        int n_slpstr = 0;

        Wind * wind;
        V3D wind_cp;
        int n_wind = 0;

        /// \brief For visualization in rviz
        std::string vector_vis_array_topic;
        gazebo::transport::PublisherPtr vector_vis_array_pub;
        gz_visualization_msgs::VisVectorArray vector_vis_array_msg;
        std::array<gz_visualization_msgs::ArrowMarker*, AIRFOIL_N_VIS_VEC> vec_vis;

        void UpdateIndVel(V3D w_cp) {
            v_ind_cp = V3D(0,0,0);
            for(int j=0; j<n_slpstr; j++){
                v_ind_cp += slpstr[j].GetIndVel(w_cp);
            }
        }

        void UpdateWind(V3D w_cp) {
            wind_cp = V3D(0,0,0);
            for(int j=0; j<n_wind; j++){
                wind_cp += wind[j].GetWind(w_cp);
            }
        }

    };

    Segment * segments_;
    int n_seg_ = 0;

    struct Body {

        Body():
            a_fus_xx(0),
            a_fus_yy(0),
            a_fus_zz(0),
            cd_cyl_ax(0.82),
            cd_cyl_lat(1.17),
            cp(0,0,0),
            fwd(1,0,0),
            upwd(0,0,1){}

        physics::LinkPtr ref_link = nullptr;  // reference link for force/torque calculation
        physics::LinkPtr act_link = nullptr;  // link to apply force/torque to

        double a_fus_xx;                      // forward-projected area of fuselage, [m^2]
        double a_fus_yy;                      // side-projected area of fuselage, [m^2]
        double a_fus_zz;                      // down-projected area of fuselage, [m^2]
        double cd_cyl_ax;                     // drag coefficient of long cylinder in axial flow, [-]
        double cd_cyl_lat;                    // drag coefficient of cylinder in lateral flow, [-]

        V3D cp;
        V3D fwd;
        V3D upwd;

        Wind * wind;
        V3D wind_cp;
        int n_wind = 0;

        /// \brief For visualization in rviz
        std::string vector_vis_array_topic;
        gazebo::transport::PublisherPtr vector_vis_array_pub;
        gz_visualization_msgs::VisVectorArray vector_vis_array_msg;
        std::array<gz_visualization_msgs::ArrowMarker*, BODY_N_VIS_VEC> vec_vis;

        void UpdateWind(V3D w_cp) {
            wind_cp = V3D(0,0,0);
            for(int j=0; j<n_wind; j++){
                wind_cp += wind[j].GetWind(w_cp);
            }
        }
    };

    Body * bodies_;
    int n_bdy_ = 0;

    std::string vector_vis_array_topic_;
    gz_visualization_msgs::VisVectorArray vector_vis_array_;
    gazebo::transport::PublisherPtr vector_vis_array_pub_;

    common::Time last_time_;

    bool pubs_and_subs_created_ = false;

};

}
#endif

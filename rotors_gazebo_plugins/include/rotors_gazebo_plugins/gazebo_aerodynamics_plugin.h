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
//#include "VisVector.pb.h"
#include "VisVectorArray.pb.h"
#include "ConnectGazeboToRosTopic.pb.h"
//#include <Int32.pb.h>
#include <iostream>
#include <fstream>


#include <algorithm>

#include "gazebo/common/Assert.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include <iomanip>


namespace gazebo
{

typedef ignition::math::Vector3d V3D;
typedef ignition::math::Matrix3<double> M3D;

/// \brief Stuff for propeller slipstream message subscription
typedef const boost::shared_ptr<const gz_mav_msgs::PropulsionSlipstream> PropulsionSlipstreamPtr;
static const std::string kDefaultPropulsionSlipstreamSubTopic = "/propulsion_slipstream";

//typedef const boost::shared_ptr<const gz_visualization_msgs::VisVector> GzVisVectorMsgPtr;
typedef const boost::shared_ptr<const gz_visualization_msgs::VisVectorArray> GzVisVectorArrayMsgPtr;

/// \brief Stuff for log request message subscription
//typedef const boost::shared_ptr<const std_msgs::msgs::Int32> Int32Ptr;
//static const std::string kDefaultDoLogSubTopic = "/do_log";

//void PropulsionSlipstreamCallback(PropulsionSlipstreamPtr& propulsion_slipstream, int seg_index, int slpstr_index, GazeboAerodynamics obj);

/// \brief A plugin that simulates lift and drag.
class GAZEBO_VISIBLE GazeboAerodynamics : public ModelPlugin
{
    /// \brief Constructor.
public: GazeboAerodynamics();

    /// \brief Destructor.
public: ~GazeboAerodynamics();

    // Documentation Inherited.
public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
protected: virtual void OnUpdate();

    /// \brief Connection to World Update events.
protected: event::ConnectionPtr updateConnection;

    /// \brief Pointer to world.
protected: physics::WorldPtr world;

    /// \brief Pointer to physics engine.
protected: physics::PhysicsEnginePtr physics;

    /// \brief Pointer to model containing plugin.
protected: physics::ModelPtr model;

    /// \brief Pointer to link currently targeted by mud joint.
protected: physics::LinkPtr link;

    /// \brief SDF for this plugin;
protected: sdf::ElementPtr sdf;

private: std::string namespace_;
private: transport::NodePtr node_handle_;

    /// \brief air density
    /// at 25 deg C it's about 1.1839 kg/m^3
    /// At 20 °C and 101.325 kPa, dry air has a density of 1.2041 kg/m3.
protected: double rho;

    struct control_surface {

        control_surface():
            controlJoint(nullptr),
            controlJointRadToCL(0.0),
            controlJointRadToCD(0.0),
            controlJointRadToCM(0.0){}

        physics::JointPtr controlJoint;
        double controlJointRadToCL;   // dC_L/dCS slope, [1/rad]
        double controlJointRadToCD;   // dC_D/dCS slope, [1/rad]
        double controlJointRadToCM;   // dC_M/dCS slope, [1/rad]
    };

    struct slipstream {

        slipstream():
            propulsion_slipstream_sub_(nullptr){}

        transport::SubscriberPtr propulsion_slipstream_sub_;
        std::mutex slipstream_lock;
        std::string slpstr_topic;

        V3D p_rot;
        V3D d_wake;
        V3D d_wake_e;
        V3D v_ind_d;
        V3D v_ind_e;

        double r_rot;

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

    struct segment {

        segment():
            alpha_prev(0),
            alpha_dot(0),
            cp(0,0,0),
            fwd(1,0,0),
            upwd(0,0,1),
            ellpRed(1){}

        int index;
        double alpha;

        // To include hyteresis in future implementation, currently not used
        double alpha_prev;
        double alpha_dot;
        bool cl_hist_up;

        AerodynamicParameters aero_params_;

        double cs_c_lift;
        double cs_c_drag;
        double cs_c_pitch_moment;

        V3D cp;    // Center of pressure wrt link frame, expressed in link frame
        V3D fwd;
        V3D upwd;
        double segArea;
        double segChord;
        double ellpRed;
        
        control_surface * cs;
        int n_cs = 0;

        /// \brief Quantities to model propeller/rotor wake/slipstream

        slipstream * slpstr;
        V3D v_ind_cp_; // induced velocity at cp (e.g. due to slipstream)
        int n_slpstr = 0;

        /// \brief Force and torque visualization in rviz

        gz_visualization_msgs::ArrowMarker* lift_vis;
        gz_visualization_msgs::ArrowMarker* slpstr_vis;

        void UpdateIndVel(V3D w_cp){
            v_ind_cp_ = V3D(0,0,0);
            for(int j=0; j<n_slpstr; j++){
                v_ind_cp_ += slpstr[j].GetIndVel(w_cp);;
            }
        }

    };

    segment * segments;
    int n_seg = 0;

    std::string vector_vis_array_topic;
    gz_visualization_msgs::VisVectorArray vector_vis_array;
    gazebo::transport::PublisherPtr vector_vis_array_pub;

    struct body {

        body():
            index(0),
            A_fus_xx(0),
            A_fus_yy(0),
            A_fus_zz(0),
            cd_cyl_ax(0.82),
            cd_cyl_lat(1.17),
            cp(0,0,0),
            fwd(1,0,0),
            upwd(0,0,1){}

        int index;

        double A_fus_xx;                      // forward-projected area of fuselage, [m^2]
        double A_fus_yy;                      // side-projected area of fuselage, [m^2]
        double A_fus_zz;                      // down-projected area of fuselage, [m^2]
        double cd_cyl_ax;                     // drag coefficient of long cylinder in axial flow, [-]
        double cd_cyl_lat;                    // drag coefficient of cylinder in lateral flow, [-]

        V3D cp;
        V3D fwd;
        V3D upwd;

        gz_visualization_msgs::ArrowMarker* force_vis;

    };

    body * bodies;
    int n_bdy = 0;

    common::Time last_time;

    /// \brief Debugging/Logging
protected:
    bool pubs_and_subs_created_;
    bool dbgOut;
    int printItv;
    int logItv;
    int updateCounter;
    std::ofstream logfile;
    std::string logName;
    bool logFlag;
    bool logStarted;
    bool logEnable;
    bool headerFlag;
    bool segLog [4] = {0,0,0,0};

    /// \brief Utilities
    int sgn(double val) {
        return (int)(0.0 < val) - (int)(val < 0.0);
    }
};

}
#endif

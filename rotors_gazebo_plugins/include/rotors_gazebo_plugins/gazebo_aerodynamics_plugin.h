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
#ifndef _GAZEBO_LIFT_DRAG_PLUGIN_HH_    //header guard
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
#include <PropulsionSlipstream.pb.h>
//#include <Int32.pb.h>
#include <iostream>
#include <fstream>

namespace gazebo
{

/// \brief Stuff for propeller slipstream message subscription
typedef const boost::shared_ptr<const gz_mav_msgs::PropulsionSlipstream> PropulsionSlipstreamPtr;
static const std::string kDefaultPropulsionSlipstreamSubTopic = "/propulsion_slipstream";

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

    /// \brief Lifting body type
    /// "airfoil" or "fuselage"
    //std::string bodyType;

    /// \brief Center of pressure wrt link frame, expressed in link frame
    //protected: ignition::math::Vector3d cp;

    /// \brief Normally, this is taken as a direction parallel to the chord
    /// of the airfoil in zero angle of attack forward flight (trailing to leading edge).
    //protected: ignition::math::Vector3d forward;

    /// \brief A vector in the lift/drag plane, perpendicular to the forward
    /// vector. Inflow velocity orthogonal to forward and upward vectors
    /// is considered flow in the wing sweep direction.
    //protected: ignition::math::Vector3d upward;

    /// \brief Quantities for full +/-180° AoA range, per segment
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
            v_ind_cp_(ignition::math::Vector3d(0,0,0)),
            propulsion_slipstream_sub_(nullptr){}

        transport::SubscriberPtr propulsion_slipstream_sub_;
        std::mutex writingVelInd;

        ignition::math::Vector3d cp_wrld;   // current world position of center of pressure
        ignition::math::Vector3d v_ind_cp_; // induced velocity at cp (e.g. due to slipstream)

        ignition::math::Vector3d p_rot;
        ignition::math::Vector3d d_wake;
        ignition::math::Vector3d d_wake_e;
        ignition::math::Vector3d v_ind_d;
        ignition::math::Vector3d v_ind_e;

        double d_rot;

        void Callback(PropulsionSlipstreamPtr& propulsion_slipstream){
            std::unique_lock<std::mutex> lock(writingVelInd);
            // position of rotordisk center wrt world, expressed in world frame [m]
            p_rot = ignition::math::Vector3d(propulsion_slipstream->rotor_pos().x(),
                                             propulsion_slipstream->rotor_pos().y(),
                                             propulsion_slipstream->rotor_pos().z());

            // wake direction expressed in world frame [-]
            d_wake = ignition::math::Vector3d(propulsion_slipstream->wake_dir().x(),
                                              propulsion_slipstream->wake_dir().y(),
                                              propulsion_slipstream->wake_dir().z());
            d_wake_e = d_wake.Normalized();

            // induced velocity at rotordisk, expressed in world frame [m/s]
            v_ind_d = ignition::math::Vector3d(propulsion_slipstream->ind_vel_disk().x(),
                                               propulsion_slipstream->ind_vel_disk().y(),
                                               propulsion_slipstream->ind_vel_disk().z());

            // induced velocity at end of wake (Note: not necessarily 0!)
            v_ind_e = ignition::math::Vector3d(propulsion_slipstream->ind_vel_end().x(),
                                               propulsion_slipstream->ind_vel_end().y(),
                                               propulsion_slipstream->ind_vel_end().z());

            // propeller/wake diameter
            d_rot = propulsion_slipstream->prop_diam();
            lock.unlock();
        }

        void GetIndVel(){
            std::unique_lock<std::mutex> lock(writingVelInd);
            ignition::math::Vector3d p_r2cp_ = cp_wrld - p_rot;
            double off_a_ = d_wake_e.Dot(p_r2cp_);                 // axial distance in wake (d1 in report)
            double off_p_ = (off_a_*d_wake_e-p_r2cp_).Length();    // radial distance to wake centerline (d2 in report)

            if(off_a_>0 && d_wake.Length()>off_a_){
                // if in zone of slipstream influence
                double k_a_ = (d_wake.Length()-off_a_)/d_wake.Length();    // axial direction interpolation weight
                double k_p_ = 1-pow((off_p_/d_rot),4);
                k_p_ = ignition::math::clamp(k_p_,0.0,1.0);                // radial distance downscaling
                v_ind_cp_ = k_p_*(k_a_*v_ind_d+(1-k_a_)*v_ind_e);   // induced velocity at airfoil segment cp
            }
            lock.unlock();
        }
    };

    struct segment {

        segment():
            alpha_prev(0){}

        double alpha;
        double alpha_prev;
        double alpha_max_ns;
        double alpha_min_ns;

        Eigen::Vector3d c_lift_alpha;
        Eigen::Vector3d c_drag_alpha;
        Eigen::Vector2d c_pitch_moment_alpha;

        double alpha_blend;
        double fp_c_lift_max;
        double fp_c_drag_max;
        double fp_c_pitch_moment_max;

        double cs_c_lift;
        double cs_c_drag;
        double cs_c_pitch_moment;

        /*
      // double alpha_zlift;           // zero-lift AoA, [rad]
      // double cla;                   // dC_L/DAoA, [1/rad]
      // double alpha_dmin;            // AoA where drag minimal,  [rad]
      // double cd_af_min;             // Minimum drag coefficient if not stalled, [-]
      // double cd_af_stall;           // Drag coefficient when stalled, [-]
      // double alphaStall;            // AoA where C_d = cd_af_stall, [rad]
      // double cm_af_0;               // Moment coefficient at zero AoA, [-]
      // double cl_fp_max;             // Flat-plate maximum lift coefficient, [-]
      // double cd_fp_max;             // Flat-plate maximum drag coefficient, [-]
      // double cm_fp_max;             // Flat-plate maximum moment coefficient, [-]
      // double cla_lin [2];           // AoA range where lift ~linear in AoA, [rad]
      // double d_a;                   // Blending range around stall AoA Airfoil vs Flat-plate model, [rad]

      //AerodynamicParameters aero_params_;

      // bool segUse [4] = {0,0,0,0};          // Indicate which segments to use (0: dont use, 1: use), [-]
      // bool segSlps [4] = {0,0,0,0};         // Airflow type (0: free stream, 1: slipstream), [-]
      // bool segCS [4] = {0,0,0,0};           // Control surface presence (0: without control surface, 1: with control surface), [-]
      // double segYOffset [4] = {0,0,0,0};    // Offset in y-axis of the 4 wing segments, [m]
      // double segChord [4] = {0,0,0,0};      // Segment chord lengths, [m]
      // double segArea [4] = {0,0,0,0};       // Effective planeform surface areas of the 4 wing segments, [m^2]
      */

        ignition::math::Vector3d cp;
        ignition::math::Vector3d fwd;
        ignition::math::Vector3d upwd;
        double segArea;
        double segChord;
        
        control_surface * cs;
        int n_cs = 0;

        /// \brief Quantities to model propeller/rotor wake/slipstream

        slipstream * slpstr;
        //ignition::math::Vector3d cp_wrld; // current world position of center of pressure
        ignition::math::Vector3d v_ind_cp_; // induced velocity at cp (e.g. due to slipstream)
        int n_slpstr = 0;

    };

    segment * segments;
    int n_seg = 0;

    /// \brief Fuselage lift/drag modeling

    struct body {

        body():
            A_fus_xx(0),
            A_fus_yy(0),
            A_fus_zz(0),
            cd_cyl_ax(0),
            cd_cyl_lat(0){}

        double A_fus_xx;                      // forward-projected area of fuselage, [m^2]
        double A_fus_yy;                      // side-projected area of fuselage, [m^2]
        double A_fus_zz;                      // down-projected area of fuselage, [m^2]
        double cd_cyl_ax;                     // drag coefficient of long cylinder in axial flow, [-]
        double cd_cyl_lat;                    // drag coefficient of cylinder in lateral flow, [-]

        ignition::math::Vector3d cp;
        ignition::math::Vector3d fwd;
        ignition::math::Vector3d upwd;

    };

    body * bodies;
    int n_bdy = 0;

    /// \brief Debugging/Logging
protected:
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

    /// \brief Stuff for propeller slipstream and log request message subscription


    //std::string propulsion_slipstream_sub_topic_;
    //transport::SubscriberPtr propulsion_slipstream_sub_;
    //void PropulsionSlipstreamCallback(PropulsionSlipstreamPtr& propulsion_slipstream);

    std::string do_log_sub_topic_;
    transport::SubscriberPtr do_log_sub_;
    //void DoLogCallback(Int32Ptr& do_log);

    /// \brief Utilities
    int sgn(double val) {
        return (int)(0.0 < val) - (int)(val < 0.0);
    }
};

// callback wrapper

/*
  struct cb_wrapper
  {
      cb_wrapper() {}
      void PropulsionSlipstreamCallback(PropulsionSlipstreamPtr& propulsion_slipstream, int seg_index, int slpstr_index, GazeboAerodynamics *obj){
          obj->segments[seg_index].GetIndVel(propulsion_slipstream, slpstr_index);
      }

  };
  */
}
#endif

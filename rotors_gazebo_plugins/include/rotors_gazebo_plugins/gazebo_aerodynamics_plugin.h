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
      
    /// \brief Pointer to a joint that actuates a control surface for
    /// this lifting body
    protected: physics::JointPtr controlJoint;
      
    /// \brief SDF for this plugin;
    protected: sdf::ElementPtr sdf;
      
    /// \brief air density
    /// at 25 deg C it's about 1.1839 kg/m^3
    /// At 20 °C and 101.325 kPa, dry air has a density of 1.2041 kg/m3.
    protected: double rho;

    /// \brief Lifting body type
    /// "airfoil" or "fuselage"
    std::string bodyType;
      
    /// \brief Center of pressure wrt link frame, expressed in link frame
    protected: ignition::math::Vector3d cp;

    /// \brief Normally, this is taken as a direction parallel to the chord
    /// of the airfoil in zero angle of attack forward flight (trailing to leading edge).
    protected: ignition::math::Vector3d forward;

    /// \brief A vector in the lift/drag plane, perpendicular to the forward
    /// vector. Inflow velocity orthogonal to forward and upward vectors
    /// is considered flow in the wing sweep direction.
    protected: ignition::math::Vector3d upward;

    /// \brief Angle of attack
    protected: double alpha;
      
    /// \brief Quantities for full +/-180° AoA range
    protected:
      double alpha_zlift;           // zero-lift AoA, [rad]
      double cla;                   // dC_L/DAoA, [1/rad]
      double alpha_dmin;            // AoA where drag minimal,  [rad]
      double cd_af_min;             // Minimum drag coefficient if not stalled, [-]
      double cd_af_stall;           // Drag coefficient when stalled, [-]
      double alphaStall;            // AoA where C_d = cd_af_stall, [rad]
      double cm_af_0;               // Moment coefficient at zero AoA, [-]
      double cl_fp_max;             // Flat-plate maximum lift coefficient, [-]
      double cd_fp_max;             // Flat-plate maximum drag coefficient, [-]
      double cm_fp_max;             // Flat-plate maximum moment coefficient, [-]
      double cla_lin [2];           // AoA range where lift ~linear in AoA, [rad]
      double d_a;                   // Blending range around stall AoA Airfoil vs Flat-plate model, [rad]

      double controlJointRadToCL;   // dC_L/dCS slope, [1/rad]
      double controlJointRadToCD;   // dC_D/dCS slope, [1/rad]
      double controlJointRadToCM;   // dC_M/dCS slope, [1/rad]
      
    /// \brief Quantities for full +/-180° AoA range
    AerodynamicParameters aero_params_;

    /// \brief Multi-segment airfoil
    protected:
      bool segUse [4] = {0,0,0,0};          // Indicate which segments to use (0: dont use, 1: use), [-]
      bool segSlps [4] = {0,0,0,0};         // Airflow type (0: free stream, 1: slipstream), [-]
      bool segCS [4] = {0,0,0,0};           // Control surface presence (0: without control surface, 1: with control surface), [-]
      double segYOffset [4] = {0,0,0,0};    // Offset in y-axis of the 4 wing segments, [m]
      double segChord [4] = {0,0,0,0};      // Segment chord lengths, [m]
      double segArea [4] = {0,0,0,0};       // Effective planeform surface areas of the 4 wing segments, [m^2]
    
    /// \brief Fuselage lift/drag modeling
    protected:
      double A_fus_xx;                      // forward-projected area of fuselage, [m^2]
      double A_fus_yy;                      // side-projected area of fuselage, [m^2]
      double A_fus_zz;                      // down-projected area of fuselage, [m^2]
      double cd_cyl_ax;                     // drag coefficient of long cylinder in axial flow, [-]
      double cd_cyl_lat;                    // drag coefficient of cylinder in lateral flow, [-]
      
    /// \brief Quantities to model propeller/rotor wake/slipstream
    protected:
      ignition::math::Vector3d p_rot;   // position of rotordisk center wrt world, expressed in world frame [m]
      ignition::math::Vector3d d_wake;  // wake direction expressed in world frame [-]
      ignition::math::Vector3d v_ind_d; // induced velocity at rotordisk, expressed in world frame [m/s]
      ignition::math::Vector3d v_ind_e; // induced velocity at end of wake (Note: not necessarily 0!)
      double d_rot;                     // propeller diameter [m]
      
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
    private:
      std::string namespace_;
      transport::NodePtr node_handle_;
      
      std::string propulsion_slipstream_sub_topic_;
      transport::SubscriberPtr propulsion_slipstream_sub_;
      void PropulsionSlipstreamCallback(PropulsionSlipstreamPtr& propulsion_slipstream);
      
      std::string do_log_sub_topic_;
      transport::SubscriberPtr do_log_sub_;
      //void DoLogCallback(Int32Ptr& do_log);
      
    /// \brief Utilities
    int sgn(double val) {
        return (int)(0.0 < val) - (int)(val < 0.0);
    }
  };
}
#endif

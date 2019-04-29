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

#include <algorithm>
#include <string>

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "rotors_gazebo_plugins/gazebo_aerodynamics_plugin.h"

#include <iomanip>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(GazeboAerodynamics)

/////////////////////////////////////////////////
GazeboAerodynamics::GazeboAerodynamics()
{
    /// Initialize to defaults
    /// comments on the different variables can be found in the
    /// header fileown_plugins/liftdrag_plugin_dr.h
    
    this->rho = 1.2041;
    this->bodyType = "airfoil";
    //this->cp = ignition::math::Vector3d(0, 0, 0);
    //this->forward = ignition::math::Vector3d(1, 0, 0);
    //this->upward = ignition::math::Vector3d(0, 0, 1);
    //this->alpha = 0.0;
    
    /// Quantities for full 360° AoA range
    //this->alpha_zlift = 0.0;
    //this->cla = 2.0*M_PI;
    //this->alpha_dmin = 0;
    //this->cd_af_min = 0.01;
    //this->cd_af_stall = 0.04;
    //this->cm_af_0 = -0.1;
    //this->alphaStall = 0.5*M_PI;
    //this->cl_fp_max = 0.65;
    //this->cd_fp_max = 1.2;
    //this->cm_fp_max = 0.4;
    //this->cla_lin[0] = -0.0873;
    //this->cla_lin[1] = 0.2269;
    //this->d_a = 0.1;
    
    /// How much to change coefficents per every radian of the control joint value
    //this->controlJointRadToCL = 0.0;
    //this->controlJointRadToCM = 0.0;
    //this->controlJointRadToCD = 0.0;
    
    /// Fuselage lift/drag
    this->A_fus_xx = 0.0;
    this->A_fus_yy = 0.0;
    this->A_fus_zz = 0.0;
    this->cd_cyl_ax = 0.82;
    this->cd_cyl_lat = 1.17;

    /// Propeller slipstream modeling
    //this->p_rot = ignition::math::Vector3d(0,0,0);
    //this->d_wake = ignition::math::Vector3d(0,0,0);
    //this->v_ind_d = ignition::math::Vector3d(0,0,0);
    //this->v_ind_e = ignition::math::Vector3d(0,0,0);
    //this->d_rot = 0.0;
    
    /// Debugging/Logging
    this->dbgOut = false;
    this->printItv = 300;
    this->logItv = 100;
    this->updateCounter = 0;
    this->logName = "example.txt";
    this->logFlag = false;
    this->logStarted = false;
    this->logEnable = false;
    this->headerFlag = true;
    
    /// Topics to subscribe to
    //this->propulsion_slipstream_sub_topic_ = kDefaultPropulsionSlipstreamSubTopic;
    //this->do_log_sub_topic_ = kDefaultDoLogSubTopic;
    
    //std::cout<<"liftdrag constructed"<<std::endl;
    gzdbg<<"gazebo_aerodynamics constructed"<<std::endl;
}

/////////////////////////////////////////////////
GazeboAerodynamics::~GazeboAerodynamics()
{
    this->logfile.close();

    for(int i=0; i<n_seg; i++){
        delete[] segments[i].slpstr;
        delete[] segments[i].cs;
        //delete[] segments[i].v_ind_cp_;
    }
    delete[] segments;
    
    gzdbg<<"liftdrag destructed"<<std::endl;
}

/////////////////////////////////////////////////
void GazeboAerodynamics::Load(physics::ModelPtr _model,
                          sdf::ElementPtr _sdf)
{
    gzdbg<<"gazebo_aerodynamics load called"<<std::endl;

    GZ_ASSERT(_model, "GazeboAerodynamics _model pointer is NULL");
    GZ_ASSERT(_sdf, "GazeboAerodynamics _sdf pointer is NULL");
    this->model = _model;
    this->sdf = _sdf;
    
    this->world = this->model->GetWorld();
    GZ_ASSERT(this->world, "GazeboAerodynamics world pointer is NULL");
    
#if GAZEBO_MAJOR_VERSION >= 9
    this->physics = this->world->Physics();
#else
    this->physics = this->world->GetPhysicsEngine();
#endif
    GZ_ASSERT(this->physics, "GazeboAerodynamics physics pointer is NULL");
    
    GZ_ASSERT(_sdf, "GazeboAerodynamics _sdf pointer is NULL");
    
    namespace_.clear();
    
    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";
    
    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init(namespace_);
    
    if (_sdf->HasElement("link_name")) {
        sdf::ElementPtr elem = _sdf->GetElement("link_name");
        // GZ_ASSERT(elem, "Element link_name doesn't exist!");
        std::string linkName = elem->Get<std::string>();
        this->link = this->model->GetLink(linkName);
        // GZ_ASSERT(this->link, "Link was NULL");
        
        if (!this->link) {
            gzerr << "Link with name[" << linkName << "] not found. "
            << "The GazeboAerodynamics will not generate forces\n";
            std::cout<<"updateConnection not called"<<std::endl;
        
        } else {
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboAerodynamics::OnUpdate, this));
            std::cout<<"updateConnection called"<<std::endl;
        }
    }
    
    if (_sdf->HasElement("airfoil")) {

        sdf::ElementPtr _sdf_airfoil = _sdf->GetElement("airfoil");
        sdf::ElementPtr _sdf_segment = _sdf_airfoil->GetElement("segment");
        
        while (_sdf_segment) {
            _sdf_segment = _sdf_segment->GetNextElement("segment");
            ++n_seg;
        }
        
        gzdbg<<"found "<<n_seg<<"airfoil segments for this link. \n";
        segments = new segment [n_seg];
        
        _sdf_segment = _sdf_airfoil->GetElement("segment");
        
        for(int i=0; i<n_seg; i++){
            
            if (_sdf_segment->HasElement("forward"))
                segments[i].fwd = _sdf_segment->Get<ignition::math::Vector3d>("forward");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'forward' element \n";

            segments[i].fwd = segments[i].fwd.Normalize();
            
            if (_sdf_segment->HasElement("upward"))
                segments[i].upwd = _sdf_segment->Get<ignition::math::Vector3d>("upward");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'upward' element \n";

            segments[i].upwd = segments[i].upwd.Normalize();
            
            if (_sdf_segment->HasElement("cp"))
                segments[i].cp = _sdf_segment->Get<ignition::math::Vector3d>("cp");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'cp' element \n";

            if (_sdf_segment->HasElement("seg_area"))
                segments[i].segArea = _sdf_segment->Get<double>("seg_area");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'seg_area' element \n";

            if (_sdf_segment->HasElement("seg_chord"))
                segments[i].segChord = _sdf_segment->Get<double>("seg_chord");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'seg_chord' element \n";

            AerodynamicParameters aero_params_;

            if (_sdf_segment->HasElement("aeroParamsYAML")) {
                std::string aero_params_yaml =
                _sdf_segment->GetElement("aeroParamsYAML")->Get<std::string>();
                aero_params_.LoadAeroParamsYAML(aero_params_yaml);

            } else {
                gzwarn<<"segment ["<<i<<"] is missing aerodynamic paramaters YAML file, "
                      <<"using default parameters.\n";
            }

            segments[i].alpha_max_ns = aero_params_.alpha_max_ns;
            segments[i].alpha_min_ns = aero_params_.alpha_min_ns;

            segments[i].c_lift_alpha = aero_params_.c_lift_alpha;
            segments[i].c_drag_alpha = aero_params_.c_drag_alpha;
            segments[i].c_pitch_moment_alpha = aero_params_.c_pitch_moment_alpha;

            segments[i].alpha_blend = aero_params_.alpha_blend;
            segments[i].fp_c_lift_max = aero_params_.fp_c_lift_max;
            segments[i].fp_c_drag_max = aero_params_.fp_c_drag_max;
            segments[i].fp_c_pitch_moment_max = aero_params_.fp_c_pitch_moment_max;

            // get control joints
            if (_sdf_segment->HasElement("control")) {
                 sdf::ElementPtr _sdf_control = _sdf_segment->GetElement("control");
                 sdf::ElementPtr _sdf_cs = _sdf_control->GetElement("cs");

                 // get number of control inputs on this particular segment
                 while (_sdf_cs) {
                     _sdf_cs = _sdf_cs->GetNextElement("cs");
                     ++segments[i].n_cs;
                 }

                 gzdbg<<"found "<<segments[i].n_cs<<" control surface segments for segment ["<<i<<"]. \n";
                 _sdf_cs = _sdf_control->GetElement("cs");
                 segments[i].cs = new control_surface [segments[i].n_cs];

                 for(int j=0; j<segments[i].n_cs; j++){

                   if (_sdf_cs->HasElement("controlJoint")){
                       std::string joint_name = _sdf_cs->Get<std::string>("controlJoint");
                       segments[i].cs[j].controlJoint = model->GetJoint(joint_name);

                       if (model->GetJoint(joint_name) == nullptr)
                           gzwarn << "joint [" << joint_name << "] not found \n";

                   } else {
                       gzwarn<<"control surface ["<<j<<"] of segment ["<<i<<"] is missing 'controlJoint' element \n";
                   }

                   if (_sdf_cs->HasElement("radToCLift"))
                        segments[i].cs[j].controlJointRadToCL = _sdf_cs->Get<double>("radToCLift");
                   else
                        gzwarn<<"control surface ["<<j<<"] of segment ["<<i<<"] is missing 'radToCLift' element \n";

                   if (_sdf_cs->HasElement("radToCDrag"))
                        segments[i].cs[j].controlJointRadToCD = _sdf_cs->Get<double>("radToCDrag");
                   else
                        gzwarn<<"control surface ["<<j<<"] of segment ["<<i<<"] is missing 'radToCDrag' element \n";

                   if (_sdf_cs->HasElement("radToCPitch"))
                        segments[i].cs[j].controlJointRadToCM = _sdf_cs->Get<double>("radToCPitch");
                   else
                        gzwarn<<"control surface ["<<j<<"] of segment ["<<i<<"] is missing 'radToCPitch' element \n";

                   _sdf_cs = _sdf_cs->GetNextElement("cs");
                 }
            }

            // get slipstreams
            if (_sdf_segment->HasElement("indVel")) {
                 sdf::ElementPtr _sdf_ind_vel= _sdf_segment->GetElement("indVel");
                 sdf::ElementPtr _sdf_slpstr = _sdf_ind_vel->GetElement("slpstr");

                 // get number of slipstreams for this particular segment
                 while (_sdf_slpstr) {
                     _sdf_slpstr = _sdf_slpstr->GetNextElement("slpstr");
                     ++segments[i].n_slpstr;
                 }

                 gzdbg<<"found "<<segments[i].n_slpstr<<" slipstreams for segment ["<<i<<"]. \n";
                 _sdf_slpstr = _sdf_ind_vel->GetElement("slpstr");

                 segments[i].slpstr = new slipstream [segments[i].n_slpstr];
                 //segments[i].propulsion_slipstream_sub_ = new transport::SubscriberPtr [segments[i].n_slpstr];
                 //segments[i].v_ind_cp_ = new ignition::math::Vector3d [segments[i].n_slpstr];
                 //segments[i].f = new boost::function<void(PropulsionSlipstreamPtr& msg_ptr)> [segments[i].n_slpstr];

                 for(int j=0; j<segments[i].n_slpstr; j++){

                    if(_sdf_slpstr->HasElement("topic")){
                        std::string slpstr_topic = _sdf_slpstr->Get<std::string>("topic");

                        segments[i].slpstr[j].propulsion_slipstream_sub_ = node_handle_->Subscribe("~/" + model->GetName() + slpstr_topic, &GazeboAerodynamics::slipstream::GetIndVel, &segments[i].slpstr[j]);

                        //boost::function<void(PropulsionSlipstreamPtr& msg_ptr)> f = boost::bind(PropulsionSlipstreamCallback,_1,i,j,this);

                        //segments[i].f[j] = boost::bind(PropulsionSlipstreamCallback,_1,i,j,this);

                        //segments[i].propulsion_slipstream_sub_[j] = node_handle_->Subscribe("~/" + model->GetName() + slpstr_topic, segments[i].f[j]);


                        //segments[i].propulsion_slipstream_sub_[j] = node_handle_->Subscribe("~/" + model->GetName() + slpstr_topic, &boost::bind(PropulsionSlipstreamCallback,_1,i,j,this));

/*
                        segments[i].propulsion_slipstream_sub_[j] = node_handle_->Subscribe("~/" + model->GetName() + slpstr_topic,
                                                                                         &GazeboAerodynamics::PropulsionSlipstreamCallback,this);
*/
                    } else {
                        gzwarn<<"slipstream ["<<j<<"] of segment ["<<i<<"] is missing 'radToCPitch' element \n";
                    }
                 }
            }

            _sdf_segment = _sdf_segment->GetNextElement("segment");
        }
    }

    /*
    if (_sdf->HasElement("air_density"))
        this->rho = _sdf->Get<double>("air_density");
    
    if (_sdf->HasElement("forward"))
        this->forward = _sdf->Get<ignition::math::Vector3d>("forward");
    this->forward.Normalize();
    
    if (_sdf->HasElement("upward"))
        this->upward = _sdf->Get<ignition::math::Vector3d>("upward");
    this->upward.Normalize();
    
    if (_sdf->HasElement("body_type")) {
        this->bodyType =_sdf->Get<std::string>("body_type");
        std::string cmp1 = "airfoil";
        std::string cmp2 = "fuselage";
        
        std::cout<<"bodyType comparison cmp1: "<<cmp1.compare(this->bodyType)<<std::endl;
        std::cout<<"bodyType comparison cmp2: "<<cmp2.compare(this->bodyType)<<std::endl;
        
        if(cmp1.compare(this->bodyType)==0)
            std::cout<<"bodyType is airfoil"<<std::endl;
        
        if(cmp2.compare(this->bodyType)==0)
            std::cout<<"bodyType is fuselage"<<std::endl;
        
        if(cmp1.compare(this->bodyType)!=0 && cmp2.compare(this->bodyType)!=0){
            gzerr << "body type needs to be either 'airfoil' or 'fuselage', setting it to 'airfoil'";
            this->bodyType = "airfoil";
        }
    
    } else {
        gzerr << "missing <body_type> element\n";
    }
    
    if (_sdf->HasElement("cp"))
        this->cp = _sdf->Get<ignition::math::Vector3d>("cp");
    
    */
    /*
    if (this->bodyType.compare("airfoil")==0) {
        
        std::cout<<"Airfoil body type found"<<std::endl;
        
        if (_sdf->HasElement("a0"))
            this->alpha_zlift = _sdf->Get<double>("a0")/180.0*M_PI;
        
        if (_sdf->HasElement("cla"))
            this->cla = _sdf->Get<double>("cla");
        
        if (_sdf->HasElement("a_min_drag"))
            this->alpha_dmin = _sdf->Get<double>("a_min_drag")/180.0*M_PI;
        
        if (_sdf->HasElement("cd_af_min"))
            this->cd_af_min = _sdf->Get<double>("cd_af_min");
        
        if (_sdf->HasElement("cd_af_stall"))
            this->cd_af_stall = _sdf->Get<double>("cd_af_stall");
        
        if (_sdf->HasElement("alpha_stall"))
            this->alphaStall = _sdf->Get<double>("alpha_stall")/180.0*M_PI;
        
        if (_sdf->HasElement("cm_af_0"))
            this->cm_af_0 = _sdf->Get<double>("cm_af_0");
        
        if (_sdf->HasElement("cl_fp_max"))
            this->cl_fp_max = _sdf->Get<double>("cl_fp_max");
        
        if (_sdf->HasElement("cd_fp_max"))
            this->cd_fp_max = _sdf->Get<double>("cd_fp_max");
        
        if (_sdf->HasElement("cm_fp_max"))
            this->cm_fp_max = _sdf->Get<double>("cm_fp_max");
        
        if (_sdf->HasElement("cla_lin_intrvl")) {
            ignition::math::Vector2d claLinIntrv = _sdf->Get<ignition::math::Vector2d>("cla_lin_intrvl");
            this->cla_lin[0] = claLinIntrv.X()/180.0*M_PI;
            this->cla_lin[1] = claLinIntrv.Y()/180.0*M_PI;
            
        } else {
            gzerr << "missing <cla_lin_intrvl> element\n";
        }
        
        if (_sdf->HasElement("blend_range"))
            this->d_a = _sdf->Get<double>("blend_range")/180.0*M_PI;
        
        if (_sdf->HasElement("control_joint_name"))
        {
            std::string controlJointName = _sdf->Get<std::string>("control_joint_name");
            this->controlJoint = this->model->GetJoint(controlJointName);
            if (!this->controlJoint)
            {
                gzerr << "Joint with name[" << controlJointName << "] does not exist.\n";
            }
        }
        
        if (_sdf->HasElement("control_joint_deg_to_cl"))
            this->controlJointRadToCL = _sdf->Get<double>("control_joint_deg_to_cl")*180.0/M_PI;
        
        if (_sdf->HasElement("control_joint_deg_to_cd"))
            this->controlJointRadToCD = _sdf->Get<double>("control_joint_deg_to_cd")*180.0/M_PI;
        
        if (_sdf->HasElement("control_joint_deg_to_cm"))
            this->controlJointRadToCM  = _sdf->Get<double>("control_joint_deg_to_cm")*180.0/M_PI;
        
        if (_sdf->HasElement("seg_use")) {
            ignition::math::Vector4i segUseTemp = _sdf->Get<ignition::math::Vector4i>("seg_use");
            this->segUse[0] = (bool)segUseTemp.X();
            this->segUse[1] = (bool)segUseTemp.Y();
            this->segUse[2] = (bool)segUseTemp.Z();
            this->segUse[3] = (bool)segUseTemp.W();
            //std::cout<<"segUse: "<<this->segUse[0]<<this->segUse[1]<<this->segUse[2]<<this->segUse[3]<<std::endl;
        
        } else {
            gzerr << "missing <seg_use> element\n";
        }
        
        if (_sdf->HasElement("seg_slps")){
            ignition::math::Vector4i segSlpsTemp = _sdf->Get<ignition::math::Vector4i>("seg_slps");
            this->segSlps[0] = segSlpsTemp.X();
            this->segSlps[1] = segSlpsTemp.Y();
            this->segSlps[2] = segSlpsTemp.Z();
            this->segSlps[3] = segSlpsTemp.W();
            
            if (this->segSlps[0]||this->segSlps[1]||this->segSlps[2]||this->segSlps[3]) {
                if (_sdf->HasElement("propulsionSlipstreamSubTopic")) {
                    this->propulsion_slipstream_sub_topic_ = _sdf->Get<std::string>("propulsionSlipstreamSubTopic");
                    propulsion_slipstream_sub_ = node_handle_->Subscribe("~/" + model->GetName() + propulsion_slipstream_sub_topic_, &GazeboAerodynamics::PropulsionSlipstreamCallback, this);
                    
                } else {
                    gzerr << "please specify a propulsionSlipstreamSubTopic that matches one published by a motor, even if it is not used\n";
                }
            }
            
        } else {
            gzerr << "missing <seg_type> element\n";
        }
        
        if (_sdf->HasElement("seg_ctrl_surf")) {
            ignition::math::Vector4i segCSTemp = _sdf->Get<ignition::math::Vector4i>("seg_ctrl_surf");
            this->segCS[0] = segCSTemp.X();
            this->segCS[1] = segCSTemp.Y();
            this->segCS[2] = segCSTemp.Z();
            this->segCS[3] = segCSTemp.W();
            
        } else {
            gzerr << "missing <seg_ctrl_surf> element\n";
        }
        
        if (_sdf->HasElement("seg_y_off")) {
            ignition::math::Vector4d segYOffsetTemp = _sdf->Get<ignition::math::Vector4d>("seg_y_off");
            this->segYOffset[0] = segYOffsetTemp.X();
            this->segYOffset[1] = segYOffsetTemp.Y();
            this->segYOffset[2] = segYOffsetTemp.Z();
            this->segYOffset[3] = segYOffsetTemp.W();
        
        } else {
            gzerr << "missing <seg_y_off> element\n";
        }
        
        if (_sdf->HasElement("seg_chord")) {
            ignition::math::Vector4d segChordTemp = _sdf->Get<ignition::math::Vector4d>("seg_chord");
            this->segChord[0] = segChordTemp.X();
            this->segChord[1] = segChordTemp.Y();
            this->segChord[2] = segChordTemp.Z();
            this->segChord[3] = segChordTemp.W();
        
        } else {
            gzerr << "missing <seg_chord> element\n";
        }
        
        if (_sdf->HasElement("seg_area")) {
            ignition::math::Vector4d segAreaTemp = _sdf->Get<ignition::math::Vector4d>("seg_area");
            this->segArea[0] = segAreaTemp.X()/10000.0;
            this->segArea[1] = segAreaTemp.Y()/10000.0;
            this->segArea[2] = segAreaTemp.Z()/10000.0;
            this->segArea[3] = segAreaTemp.W()/10000.0;
            //std::cout<<"segArea: "<<this->segArea[0]<<" "<<this->segArea[1]<<" "<<this->segArea[2]<<" "<<this->segArea[3]<<std::endl;
            
        } else {
            gzerr << "missing <seg_area> element\n";
        }
        
        if (_sdf->HasElement("seg_log")) {
            ignition::math::Vector4i segLogTemp = _sdf->Get<ignition::math::Vector4i>("seg_log");
            this->segLog[0] = (bool)segLogTemp.X();
            this->segLog[1] = (bool)segLogTemp.Y();
            this->segLog[2] = (bool)segLogTemp.Z();
            this->segLog[3] = (bool)segLogTemp.W();
        
        } else {
            gzerr << "missing <seg_log> element\n";}
    }
    */
    /*
    if (this->bodyType.compare("fuselage")==0) {
        
        if (_sdf->HasElement("A_fus_xx"))
            this->A_fus_xx = _sdf->Get<double>("A_fus_xx")/10000.0;
        
        if (_sdf->HasElement("A_fus_yy"))
            this->A_fus_yy = _sdf->Get<double>("A_fus_yy")/10000.0;
        
        if (_sdf->HasElement("A_fus_zz"))
            this->A_fus_zz = _sdf->Get<double>("A_fus_zz")/10000.0;
    }
    */

    if (_sdf->HasElement("dbg_out"))
        this->dbgOut = _sdf->Get<int>("dbg_out");
    
    if (_sdf->HasElement("print_itv"))
        this->printItv = _sdf->Get<int>("print_itv");
    
    if (_sdf->HasElement("log_itv"))
        this->logItv = _sdf->Get<int>("log_itv");
    
    if (_sdf->HasElement("log_name")) {
        this->logName = _sdf->Get<std::string>("log_name");
        this->logEnable = true;
        
    } else {
        this->logEnable = false;    // default
    }

    //do_log_sub_ = node_handle_->Subscribe("~/" + model->GetName() + do_log_sub_topic_, &GazeboAerodynamics::DoLogCallback, this);   // triggers log
}

/////////////////////////////////////////////////
/*
void GazeboAerodynamics::PropulsionSlipstreamCallback(PropulsionSlipstreamPtr& propulsion_slipstream){
    
    // world-position of rotor hub
    p_rot = ignition::math::Vector3d(propulsion_slipstream->rotor_pos().x(),
                                     propulsion_slipstream->rotor_pos().y(),
                                     propulsion_slipstream->rotor_pos().z());
    
    // propeller wake vector (hub to end of wake)
    d_wake = ignition::math::Vector3d(propulsion_slipstream->wake_dir().x(),
                                      propulsion_slipstream->wake_dir().y(),
                                      propulsion_slipstream->wake_dir().z());
    
    // induced velocity at propeller disk
    v_ind_d = ignition::math::Vector3d(propulsion_slipstream->ind_vel_disk().x(),
                                       propulsion_slipstream->ind_vel_disk().y(),
                                       propulsion_slipstream->ind_vel_disk().z());
    
    // induced velocity at end of wake (i.e. at p_rot+d_wake)
    v_ind_e = ignition::math::Vector3d(propulsion_slipstream->ind_vel_end().x(),
                                       propulsion_slipstream->ind_vel_end().y(),
                                       propulsion_slipstream->ind_vel_end().z());
    
    // propeller/wake diameter
    d_rot = propulsion_slipstream->prop_diam();
    
}
*/
                
/*
void GazeboAerodynamics::DoLogCallback(Int32Ptr& do_log){
    if (do_log->data() == 1 && logEnable) {
        logFlag = true; // starts log
    
    } else {
        logFlag = false; // stops log
    }
}
*/


void GazeboAerodynamics::OnUpdate()
{
#if false
    //ignition::math::Vector3d pos_ref = ignition::math::Vector3d(0, 0, 0); // used as reference position (relative to and expressed in fuselage frame) to calculate moments exerted by airfoils for comparison with PX4 aerodynamic model (debugging).
    
    if (this->bodyType.compare("airfoil") == 0) {
        
        // logging of interesting quantities for debugging purpose. Filepath hardcoded, improve
        // log is started/stopped via command in do_log_sub_topic_ if logEnable=true
        if (!logStarted && logFlag) {
            std::cout<<"starting log"<<std::endl;
            logStarted = true;
            this->logfile.open (std::string("/Users/davidrohr/Documents/ETH/Faecher/Master/Master_Thesis/Simulation/MATLAB/GazeboLogs/") + logName, std::ofstream::out | std::ofstream::app);
            if (this->headerFlag) {
                //generate header
                this->headerFlag = false;
                this->logfile << "t_sim, P_x, P_y, P_z, Q_w, Q_x, Q_y, Q_z";
                for (int i = 0; i<4; i++) {
                    if (segUse[i]==1 && segLog[i]==1) {
                        this->logfile << ", CPx_"<<i<< ", CPy_"<<i<< ", CPz_" <<i;
                        this->logfile << ", Vx_" <<i<< ", Vy_" <<i<< ", Vz_"  <<i;
                        this->logfile << ", Lx_" <<i<< ", Ly_" <<i<< ", Lz_"  <<i;
                        this->logfile << ", Dx_" <<i<< ", Dy_" <<i<< ", Dz_"  <<i;
                        this->logfile << ", Vindx_" <<i<< ", Vindy_" <<i<< ", Vindz_"  <<i;
                        this->logfile << ", a_"  <<i<< ", q_"  <<i<< ", w_af_"<<i;
                    }
                }
                this->logfile << "\n";
            }
        }
        
        if (this->logfile.is_open() && !logFlag) {
            std::cout<<"stopping log"<<std::endl;
            this->logfile.close();
            this->logStarted = false;
        }
        
        bool logNow = this->logfile.is_open() && this->updateCounter%this->logItv == 0;
        
        // find parent-link
        physics::Link_V parent_links = this->link->GetParentJointsLinks();
        physics::LinkPtr parent_link;
        
        if (parent_links.size() >= 1) {
            parent_link = parent_links.back();
        } else {
            parent_link = this->link;
        }
        
        // pose of body and parent
#if GAZEBO_MAJOR_VERSION >= 9
        ignition::math::Pose3d pose = this->link->WorldPose();
        ignition::math::Pose3d parent_pose = parent_link->WorldPose();
#else
        ignition::math::Pose3d pose = ignitionFromGazeboMath(this->link->GetWorldPose());
        ignition::math::Pose3d parent_pose = ignitionFromGazeboMath(parent_link->GetWorldPose());
#endif
        ignition::math::Vector3d cp_wrld = pose.Pos() + pose.Rot().RotateVector(this->cp);  // wing cp-reference position expressed in world-frame
        
        // log link specific data
        if (logNow) {
            this->logfile<<this->world->SimTime().Double()<<", "<<cp_wrld.X()<<", "<<cp_wrld.Y()<<", "<<cp_wrld.Z();
            this->logfile<<", "<<pose.Rot().W()<<", "<<pose.Rot().X()<<", "<<pose.Rot().Y()<<", "<<pose.Rot().Z();
        }
        
        // iterate over employed wing segments
        for (int i = 0; i<4; i++) {
            if (segUse[i] == 1) {
                
                // geometry...
                ignition::math::Vector3d cp_;
                cp_ = this->cp + ignition::math::Vector3d(0.0,this->segYOffset[i],0.0); // cp position of segment i wrt link-frame
                cp_wrld = pose.Pos() + pose.Rot().RotateVector(cp_);                    // cp position of segment i wrt to world-frame
                
                //ignition::math::Vector3d ref_wrld = parent_pose.Pos() + parent_pose.Rot().RotateVector(pos_ref);    // debugging: a body-fixed reference point, expressed in world frame
                //ignition::math::Vector3d cp_loc = parent_pose.Rot().RotateVectorReverse(cp_wrld - ref_wrld);        // debugging: reference point -> cp, expressed in body frame
                
                // calculate induced flow velocity at c/4 (i.e. cp_)
                ignition::math::Vector3d p_r2cp_;                       // vector: propeller->cp_
                ignition::math::Vector3d v_ind_cp_(0,0,0);              // zero induced (slipstream) velocity by default
                ignition::math::Vector3d d_wakeI = d_wake.Normalize();  // propeller wake direction
                double off_a_;
                double off_p_;
                double k_p_;
                double k_a_;
                
                // if we want to inlcude propeller slipstream
                if(segSlps[i]){
                    p_r2cp_ = cp_wrld - p_rot;
                    off_a_ = d_wakeI.Dot(p_r2cp_);                  // axial distance in wake (d1 in report)
                    off_p_ = (off_a_*d_wakeI-p_r2cp_).Length();     // radial distance to wake centerline (d2 in report)
                    
                    if(off_a_>0 && d_wake.Length()>off_a_){
                        // if in zone of slipstream influence
                        k_a_ = (d_wake.Length()-off_a_)/d_wake.Length();    // axial direction interpolation weight
                        k_p_ = 1-pow((off_p_/d_rot),4);
                        k_p_ = ignition::math::clamp(k_p_,0.0,1.0);         // radial distance downscaling
                        v_ind_cp_ = k_p_*(k_a_*v_ind_d+(1-k_a_)*v_ind_e);   // induced velocity at airfoil segment cp
                    }
                }
                
                
                // get local airspeed at cp_, expressed in world frame
                GZ_ASSERT(this->link, "Link was NULL");
#if GAZEBO_MAJOR_VERSION >= 9
                ignition::math::Vector3d vel = this->link->WorldLinearVel(cp_);
#else
                ignition::math::Vector3d vel = ignitionFromGazeboMath(this->link->GetWorldLinearVel(cp_));
#endif
                
                // account for induced velocity, expressed in world frame
                vel = vel - v_ind_cp_;
                
                // inflow direction at cp_, expressed in world frame
                ignition::math::Vector3d velI = vel;
                velI.Normalize();

                // forward, upward, spanwise direction, expressed in world frame
                ignition::math::Vector3d forwardI = pose.Rot().RotateVector(this->forward);
                ignition::math::Vector3d upwardI = pose.Rot().RotateVector(this->upward);
                ignition::math::Vector3d spanwiseI = forwardI.Cross(upwardI).Normalize();
                
                // velocity in lift-drag plane
                ignition::math::Vector3d velInLDPlane = vel - vel.Dot(spanwiseI)*spanwiseI;
                
                // get direction of drag
                ignition::math::Vector3d dragDirection = -velInLDPlane;
                dragDirection.Normalize();
                
                // get direction of lift
                ignition::math::Vector3d liftDirection = spanwiseI.Cross(velInLDPlane);
                liftDirection.Normalize();
                
                // get direction of moment
                ignition::math::Vector3d momentDirection = spanwiseI;

                // get sweep (angle between velI and lift-drag-plane) (aka sideslip)
                double cosSweepAngle = ignition::math::clamp(velI.Dot(-dragDirection), -1.0, 1.0);
                
                // compute angle of attack
                double alpha_ = acos(-dragDirection.Dot(forwardI)); // [0,pi]
                if (dragDirection.Dot(upwardI)<=0) {
                    alpha_*=-1;
                }
                
                // compute dynamic pressure
                double speedInLDPlane = velInLDPlane.Length();
                double q = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;
                
                // aerodynamic coefficient correction terms due to control surface deflection (only effective if not stalled)
#if GAZEBO_MAJOR_VERSION >= 9
                double controlAngle = this->controlJoint->Position(0);
#else
                double controlAngle = this->controlJoint->GetAngle(0).Radian();
#endif
                double d_cl = 0.0;
                double d_cd = 0.0;
                double d_cm = 0.0;
                
                // only apply if there is a control surface on this segment...
                if(segCS[i]){
                    d_cl = this->controlJointRadToCL * controlAngle;
                    d_cd = this->controlJointRadToCD * controlAngle;
                    d_cm = this->controlJointRadToCM * controlAngle;
                }
                
                // assembling aerodynamic coefficients for pre-stall (af) and post-stall (fp)
                double cl_af = (alpha_-alpha_zlift)*cla + d_cl;
                double cd_af = pow((alpha_-alpha_dmin)/(alphaStall-alpha_dmin),2)*(cd_af_stall-cd_af_min) + cd_af_min  + d_cd;
                double cm_af = cm_af_0 + d_cm;
                
                double cl_fp = cl_fp_max*sin(2*alpha_);
                double cd_fp = cd_af_min + (cd_fp_max-cd_af_min)*pow(sin(alpha_),2);
                double cm_fp = -cm_fp_max*sin(pow(alpha_,3)/(M_PI*M_PI));
                
                /* where:
                 *
                 * alpha_           Angle of attack (AoA)
                 * alpha_zlift           Zero-lift AoA
                 * alpha_dmin       AoA where drag minimal
                 * alphaStall       AoA where drag coeff = cd_af_stall
                 * cla              Lift coeff to AoA slope
                 * cd_af_min        Minimum drag coefficient if not stalled
                 * cd_af_stall      Drag coefficient at AoA = alphaStall
                 * cm_af_0          Moment coefficient at AoA=0
                 * cl_fp_max        Flat-plate maximum lift coefficient
                 * cd_fp_max        Flat-plate maximum drag coefficient
                 * cm_fp_max        Flat-plate maximum moment coefficient
                 *
                 */
                
                // form mixing weight to combine pre- and post-stall models
                double w_af;
                if(alpha_>cla_lin[1]+d_a)
                    w_af = 0.0;
                else if(alpha_>cla_lin[1])
                    w_af = 0.5+0.5*cos(M_PI*(alpha_-cla_lin[1])/d_a);
                else if(alpha_>cla_lin[0])
                    w_af = 1.0;
                else if(alpha_>cla_lin[0]-d_a)
                    w_af = 0.5+0.5*cos(M_PI*(cla_lin[0]-alpha_)/d_a);
                else
                    w_af = 0.0;
                
                // form weighted sum of pre-stall and post_stall (flat-plate) contributions to aerodynamic coefficients
                double cl = w_af*cl_af+(1-w_af)*cl_fp;
                double cd = w_af*cd_af+(1-w_af)*cd_fp;
                double cm = w_af*cm_af+(1-w_af)*cm_fp;
                
                // original plugin incorporates sweep (sideslip)
                //cl = cl*cosSweepAngle;
                //cd = cd*cosSweepAngle;
                //cm = cm*cosSweepAngle;
                
                // set to zero if desired...
                cm = 0.0;
                //cd = 0.0;
                //cl = 0.0;
                
                // assemble final forces and moments
                ignition::math::Vector3d lift   = cl * q * this->segArea[i] * liftDirection;
                ignition::math::Vector3d drag   = cd * q * this->segArea[i] * dragDirection;
                ignition::math::Vector3d moment = cm * q * this->segArea[i] * momentDirection * this->segChord[i];
                
                ignition::math::Vector3d force  = lift + drag;
                ignition::math::Vector3d torque = moment;
                
                /*
                // debugging quantities (moments wrt reference point in body frame)
                ignition::math::Vector3d lift_m = cp_loc.Cross(parent_pose.Rot().RotateVectorReverse(lift));
                ignition::math::Vector3d drag_m = cp_loc.Cross(parent_pose.Rot().RotateVectorReverse(drag));
                ignition::math::Vector3d moment_m = parent_pose.Rot().RotateVectorReverse(moment);
                ignition::math::Vector3d tot_m = lift_m+drag_m+moment_m;
                
                if(this->updateCounter%(this->printItv*5)==0){
                    gzdbg << std::setprecision(2) << std::fixed;
                    gzdbg << "plugin name: " << this->sdf->Get<std::string>("name") <<" [" << i << "] | cp_pos: (" <<cp_loc.X()<<","<<cp_loc.Y()<<","<<cp_loc.Z()<<")"<<" | tot_m: ("<<tot_m.X()<<","<<tot_m.Y()<<","<<tot_m.Z()<<")"<<"\n" ;
                }
                */
                
                // debug printing
                if (this->updateCounter%this->printItv==0 && this->dbgOut) {
                    
                    gzdbg << "=============================\n";
                    gzdbg << "link: [" << this->link->GetName() << "] segment: [" << i << "]\n";
                    //gzdbg << "dynamic pressure: [" << q << "]\n";
                    
                    //gzdbg << "forward (inertial): [" << forwardI << "]\n";
                    //gzdbg << "upward (inertial): [" << upwardI << "]\n";
                    //gzdbg << "lift dir (inertial): [" << liftDirection << "]\n";
                    //gzdbg << "drag dir (inertial): [" << dragDirection << "]\n";
                    //gzdbg << "Span direction (normal to LD plane): [" << spanwiseI << "]\n";
                    gzdbg << "vel: [" << vel << "]\n";
                    //gzdbg << "cos(alpha) " << -dragDirection.Dot(forwardI) << "\n";
                    //gzdbg << "acos(0)"<<acos(0.0)<<"acos(1)"<<acos(1.0)<< "\n";
                    
                    gzdbg << "off_a: ["<<off_a_<<"]\n";
                    gzdbg << "off_p: ["<<off_p_<<"]\n";
                    gzdbg << "k_a_: ["<<k_a_<<"]\n";
                    gzdbg << "k_p_: ["<<k_p_<<"]\n";
                    
                    //gzdbg << "p_rot: [" << p_rot<< "]\n";
                    //gzdbg << "d_wake: [" << d_wake<< "]\n";
                    //gzdbg << "vel_ind_d: [" << v_ind_d<< "]\n";
                    //gzdbg << "vel_ind_e: [" << v_ind_e<< "]\n";
                    //gzdbg << "vel_ind_cp: [" << v_ind_cp_<< "]\n";
                    
                    gzdbg << "alpha: " << alpha_ << "\n";
                    //gzdbg << "aStall:" << this->alphaStall << "\n";
                    //gzdbg << "cla_lin: " << cla_lin << "\n";
                    //gzdbg << "w_a: " << (double)w_af << "\n";
                    //gzdbg << "d_cl: " << d_cl << "\n";
                    //gzdbg << "d_alpha: " << d_alpha << "\n";
                    
                    //gzdbg << "cl: " << cl << "\n";
                    gzdbg << "vel: " << vel.Length() << "\n";
                    //gzdbg << "lift: " << lift << "\n";
                    //gzdbg << "drag: " << drag.Length() << "\n";
                    //gzdbg << "moment: " << moment.Length() << "\n";
                    //gzdbg << "dbg: " << this->dbgOut << "\n";
                    gzdbg << "Msg: " << this->propulsion_slipstream_sub_topic_ << "\n";
                    //gzdbg << "time: " << this->world->SimTime() << "\n";
                    //gzdbg << "LogFlag" << logFlag << "\n";
                    //std::cout<< this->propulsion_slipstream_sub_topic_ <<std::endl;
                    
                }
                
                // log segment specific data
                if (logNow && segLog[i]==1) {
                    this->logfile<<", "<<cp_wrld.X()<<", "<<cp_wrld.Y()<<", "<<cp_wrld.Z();
                    this->logfile<<", "<<vel.X()<<", "<<vel.Y()<<", "<<vel.Z();
                    this->logfile<<", "<<lift.X()<<", "<<lift.Y()<<", "<<lift.Z();
                    this->logfile<<", "<<drag.X()<<", "<<drag.Y()<<", "<<drag.Z();
                    this->logfile<<", "<<v_ind_cp_.X()<<", "<<v_ind_cp_.Y()<<", "<<v_ind_cp_.Z();
                    this->logfile<<", "<<alpha_<<", "<<q<<", "<<w_af;
                }
                
                // correct for nan or inf
                force.Correct();
                cp_wrld.Correct();
                torque.Correct();
                
                // apply forces cp
                this->link->AddForceAtWorldPosition(force, cp_wrld);
                this->link->AddTorque(torque);
                
            }
        }
        
        if (logNow)
            this->logfile<< "\n";
        
    }
    
    else if(this->bodyType.compare("fuselage") == 0){
        
        // logging of interesting quantities for debugging purpose. Filepath hardcoded, improve
        // log is started/stopped via command in do_log_sub_topic_ if logEnable=true
        if(!logStarted && logFlag){
            std::cout<<"starting log"<<std::endl;
            logStarted = true;
            this->logfile.open (std::string("/Users/davidrohr/Documents/ETH/Faecher/Master/Master_Thesis/Simulation/MATLAB/GazeboLogs/") + logName, std::ofstream::out | std::ofstream::app);
            if(this->headerFlag){
                this->headerFlag = false;
                this->logfile << "t_sim, P_x, P_y, P_z, Q_w, Q_x, Q_y, Q_z";
                this->logfile << ", Vx_" << ", Vy_" << ", Vz_";
                this->logfile << ", Dx_" << ", Dy_" << ", Dz_";
                this->logfile << "\n";
            }
        }
        if(this->logfile.is_open() && !logFlag){
            std::cout<<"stopping log"<<std::endl;
            this->logfile.close();
            this->logStarted = false;
        }
        bool logNow = this->logfile.is_open() && this->updateCounter%this->logItv==0;
        
        // get pose of body
        GZ_ASSERT(this->link, "Link was NULL");
#if GAZEBO_MAJOR_VERSION >= 9
        ignition::math::Pose3d pose = this->link->WorldPose();
#else
        ignition::math::Pose3d pose = ignitionFromGazeboMath(this->link->GetWorldPose());
#endif
        ignition::math::Vector3d cp_wrld = pose.Pos() + pose.Rot().RotateVector(this->cp);  // position of cp in world frame
        
        // Logging
        if(logNow){
            //this->logfile<<this->world->SimTime().Double()<<", "<<pose.Pos().X()<<", "<<pose.Pos().Y()<<", "<<pose.Pos().Z();
            this->logfile<<this->world->SimTime().Double()<<", "<<cp_wrld.X()<<", "<<cp_wrld.Y()<<", "<<cp_wrld.Z();
            this->logfile<<", "<<pose.Rot().W()<<", "<<pose.Rot().X()<<", "<<pose.Rot().Y()<<", "<<pose.Rot().Z();
        }
        
        // get velocity of cp, expressed in world frame
#if GAZEBO_MAJOR_VERSION >= 9
        ignition::math::Vector3d vel = this->link->WorldLinearVel(this->cp);
#else
        ignition::math::Vector3d vel = ignitionFromGazeboMath(this->link->GetWorldLinearVel(this->cp));
#endif
        // express forward, upward and spanwise vectors in world frame
        ignition::math::Vector3d forwardI = pose.Rot().RotateVector(this->forward);
        ignition::math::Vector3d upwardI = pose.Rot().RotateVector(this->upward);
        ignition::math::Vector3d spanwiseI = forwardI.Cross(upwardI);
        forwardI.Normalize();
        upwardI.Normalize();
        spanwiseI.Normalize();
        
        // 'signed-quadratic' flow components in forward, upward and spanwise direction
        double uu = vel.Dot(forwardI)*fabs(vel.Dot(forwardI));
        double vv = vel.Dot(upwardI)*fabs(vel.Dot(upwardI));
        double ww = vel.Dot(spanwiseI)*fabs(vel.Dot(spanwiseI));
        
        // 'directional' drag coefficiens
        double cd_x = A_fus_xx * cd_cyl_ax;
        double cd_y = A_fus_yy * cd_cyl_lat;
        double cd_z = A_fus_zz * cd_cyl_lat;
        
        // calculate and apply drag fuselage drag force
        ignition::math::Vector3d drag = -this->rho/2.0*(forwardI*uu*cd_x + upwardI*vv*cd_y + spanwiseI*ww*cd_z);
        this->link->AddForceAtWorldPosition(drag, cp_wrld);
        
        // logging
        if (logNow) {
            this->logfile<<", "<<vel.X()<<", "<<vel.Y()<<", "<<vel.Z();
            this->logfile<<", "<<drag.X()<<", "<<drag.Y()<<", "<<drag.Z();
            this->logfile<< "\n";
        }
    }
    
    this->updateCounter++;  // counter to time logging/debug printing
#endif
}



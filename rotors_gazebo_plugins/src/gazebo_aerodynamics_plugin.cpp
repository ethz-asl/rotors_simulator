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
    //this->bodyType = "airfoil";
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
    //this->A_fus_xx = 0.0;
    //this->A_fus_yy = 0.0;
    //this->A_fus_zz = 0.0;
    //this->cd_cyl_ax = 0.82;
    //this->cd_cyl_lat = 1.17;

    /// Propeller slipstream modeling
    //this->p_rot = ignition::math::Vector3d(0,0,0);
    //this->d_wake = ignition::math::Vector3d(0,0,0);
    //this->v_ind_d = ignition::math::Vector3d(0,0,0);
    //this->v_ind_e = ignition::math::Vector3d(0,0,0);
    //this->d_rot = 0.0;
    
    /// Debugging/Logging
    this->pubs_and_subs_created_ = false;
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
    
    gzdbg<<"model name: "<<model->GetName()<<"\n";
    model->Print("");
    gzdbg<<"model child count: "<<model->GetChildCount()<<"\n";

    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";
    
    node_handle_ = transport::NodePtr(new transport::Node());
    //node_handle_->Init(namespace_);
    node_handle_->Init();
    
    if (_sdf->HasElement("linkName")) {
        sdf::ElementPtr elem = _sdf->GetElement("linkName");
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

    last_time = world->SimTime(); // ini last time

    if (_sdf->HasElement("aeroForcesVis"))
        vector_vis_array_topic = _sdf->Get<std::string>("aeroForcesVis");
    else
        vector_vis_array_topic = "aero_forces_vis";

    vector_vis_array.mutable_header()->mutable_stamp()->set_sec(0.0);
    vector_vis_array.mutable_header()->mutable_stamp()->set_nsec(0.0);
    vector_vis_array.mutable_header()->set_frame_id(this->link->GetName());

    if (_sdf->HasElement("body")) {

        sdf::ElementPtr _sdf_body = _sdf->GetElement("body");
        sdf::ElementPtr _sdf_element = _sdf_body->GetElement("element");

        while (_sdf_element) {
            _sdf_element = _sdf_element->GetNextElement("element");
            ++n_bdy;
        }

        gzdbg<<"found "<<n_bdy<<" body element(s) for this link. \n";
        bodies = new body [n_bdy];

        _sdf_element = _sdf_body->GetElement("element");

        for(int i=0; i<n_bdy; i++){
            gzdbg<<"processing body-element nr: "<<i<<" \n";

            bodies[i].index = i;

            if (_sdf_element->HasElement("forward"))
                    bodies[i].fwd = _sdf_element->Get<ignition::math::Vector3d>("forward");
            else
            gzwarn<<"segment ["<<i<<"] is missing 'forward' element \n";

            bodies[i].fwd.Normalize();

            if (_sdf_element->HasElement("upward"))
                bodies[i].upwd = _sdf_element->Get<ignition::math::Vector3d>("upward");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'upward' elemenbodiest \n";

            bodies[i].upwd.Normalize();

            if (_sdf_element->HasElement("cp"))
                bodies[i].cp = _sdf_element->Get<ignition::math::Vector3d>("cp");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'cp' element \n";

            if (_sdf_element->HasElement("aBdyXX"))
                bodies[i].A_fus_xx = _sdf_element->Get<double>("aBdyXX");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'aBdyXX' element \n";

            if (_sdf_element->HasElement("aBdyYY"))
                bodies[i].A_fus_yy = _sdf_element->Get<double>("aBdyYY");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'aBdyYY' element \n";

            if (_sdf_element->HasElement("aBdyZZ"))
                bodies[i].A_fus_zz = _sdf_element->Get<double>("aBdyZZ");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'aBdyZZ' element \n";

            bodies[i].force_vis = vector_vis_array.add_vector();

            bodies[i].force_vis->set_ns(namespace_+"/bdy");
            bodies[i].force_vis->set_id(bodies[i].index);
            bodies[i].force_vis->mutable_scale()->set_x(0.025);
            bodies[i].force_vis->mutable_scale()->set_y(0.05);
            bodies[i].force_vis->mutable_scale()->set_z(0.05);
            bodies[i].force_vis->mutable_color()->set_x(1.0);
            bodies[i].force_vis->mutable_color()->set_y(0.0);
            bodies[i].force_vis->mutable_color()->set_z(0.0);
        }
    }

    if (_sdf->HasElement("airfoil")) {

        sdf::ElementPtr _sdf_airfoil = _sdf->GetElement("airfoil");
        sdf::ElementPtr _sdf_segment = _sdf_airfoil->GetElement("segment");
        
        while (_sdf_segment) {
            _sdf_segment = _sdf_segment->GetNextElement("segment");
            ++n_seg;
        }
        
        gzdbg<<"found "<<n_seg<<" airfoil segment(s) for this link. \n";
        segments = new segment [n_seg];
        
        _sdf_segment = _sdf_airfoil->GetElement("segment");
        
        for(int i=0; i<n_seg; i++){
            gzdbg<<"processing airfoil-segment nr: "<<i<<" \n";

            segments[i].index = i;

            if (_sdf_segment->HasElement("forward"))
                segments[i].fwd = _sdf_segment->Get<ignition::math::Vector3d>("forward");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'forward' element \n";

            segments[i].fwd.Normalize();
            
            if (_sdf_segment->HasElement("upward"))
                segments[i].upwd = _sdf_segment->Get<ignition::math::Vector3d>("upward");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'upward' element \n";

            segments[i].upwd.Normalize();
            
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

            if (_sdf_segment->HasElement("aeroParamsYAML")) {
                std::string aero_params_yaml =
                        _sdf_segment->GetElement("aeroParamsYAML")->Get<std::string>();
                segments[i].aero_params_.LoadAeroParamsYAML(aero_params_yaml);

            } else {
                gzwarn<<"segment ["<<i<<"] is missing aerodynamic paramaters YAML file, "
                     <<"using default parameters.\n";
            }

            /*
            segments[i].alpha_max_ns = aero_params_.alpha_max_ns;
            segments[i].alpha_min_ns = aero_params_.alpha_min_ns;

            segments[i].c_lift_alpha = aero_params_.c_lift_alpha;
            segments[i].c_drag_alpha = aero_params_.c_drag_alpha;
            segments[i].c_pitch_moment_alpha = aero_params_.c_pitch_moment_alpha;

            segments[i].alpha_blend = aero_params_.alpha_blend;
            segments[i].fp_c_lift_max = aero_params_.fp_c_lift_max;
            segments[i].fp_c_drag_max = aero_params_.fp_c_drag_max;
            segments[i].fp_c_pitch_moment_max = aero_params_.fp_c_pitch_moment_max;
            */

            // get control joints
            if (_sdf_segment->HasElement("control")) {
                sdf::ElementPtr _sdf_control = _sdf_segment->GetElement("control");
                sdf::ElementPtr _sdf_cs = _sdf_control->GetElement("cs");

                // get number of control inputs on this particular segment
                while (_sdf_cs) {
                    _sdf_cs = _sdf_cs->GetNextElement("cs");
                    ++segments[i].n_cs;
                }

                gzdbg<<"found "<<segments[i].n_cs<<" control surface segment(s) for segment ["<<i<<"]. \n";
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
                //sdf::ElementPtr _sdf_slpstr = _sdf_ind_vel->GetElement("slpstr");

                // get number of slipstreams for this particular segment
                while (_sdf_ind_vel) {
                    _sdf_ind_vel = _sdf_ind_vel->GetNextElement("indVel");
                    ++segments[i].n_slpstr;
                }

                gzdbg<<"found "<<segments[i].n_slpstr<<" slipstream(s) for segment ["<<i<<"]. \n";
                _sdf_ind_vel = _sdf_segment->GetElement("indVel");
                segments[i].slpstr = new slipstream [segments[i].n_slpstr];

                for(int j=0; j<segments[i].n_slpstr; j++){

                    if(_sdf_ind_vel->HasElement("topic")){
                        segments[i].slpstr[j].slpstr_topic = _sdf_ind_vel->Get<std::string>("topic");
                    } else {
                        gzwarn<<"slipstream ["<<j<<"] of segment ["<<i<<"] is missing 'radToCPitch' element \n";
                    }

                    _sdf_ind_vel = _sdf_ind_vel->GetNextElement("indVel");
                }

                if(segments[i].n_slpstr>0){
                    segments[i].slpstr_vis = vector_vis_array.add_vector();
                    segments[i].slpstr_vis->set_ns(namespace_+"/slpstr");
                    segments[i].slpstr_vis->set_id(segments[i].index);
                    segments[i].slpstr_vis->mutable_scale()->set_x(0.025);
                    segments[i].slpstr_vis->mutable_scale()->set_y(0.05);
                    segments[i].slpstr_vis->mutable_scale()->set_z(0.05);
                    segments[i].slpstr_vis->mutable_color()->set_x(1.0);
                    segments[i].slpstr_vis->mutable_color()->set_y(1.0);
                    segments[i].slpstr_vis->mutable_color()->set_z(1.0);
                }
            }

            segments[i].lift_vis = vector_vis_array.add_vector();
            segments[i].lift_vis->set_ns(namespace_+"/lift");
            segments[i].lift_vis->set_id(segments[i].index);
            segments[i].lift_vis->mutable_scale()->set_x(0.025);
            segments[i].lift_vis->mutable_scale()->set_y(0.05);
            segments[i].lift_vis->mutable_scale()->set_z(0.05);
            segments[i].lift_vis->mutable_color()->set_x(1.0);
            segments[i].lift_vis->mutable_color()->set_y(1.0);
            segments[i].lift_vis->mutable_color()->set_z(1.0);


            // message setup for visualization
            /*
            const std::string frame_id = this->link->GetName();
            segments[i].lift_f_vis.mutable_header()->mutable_stamp()->set_sec(0.0);
            segments[i].lift_f_vis.mutable_header()->mutable_stamp()->set_nsec(0.0);
            segments[i].lift_f_vis.mutable_header()->set_frame_id(frame_id);
            segments[i].lift_f_vis.set_ns(namespace_);
            segments[i].lift_f_vis.set_id(segments[i].index);
            segments[i].lift_f_vis.mutable_scale()->set_x(0.025);
            segments[i].lift_f_vis.mutable_scale()->set_y(0.05);
            segments[i].lift_f_vis.mutable_scale()->set_z(0.05);
            segments[i].lift_f_vis.mutable_color()->set_x(1.0);
            segments[i].lift_f_vis.mutable_color()->set_y(0.0);
            segments[i].lift_f_vis.mutable_color()->set_z(0.0);
            segments[i].lift_f_vis_topic = namespace_ + "_lift_" + std::to_string(i);
            */

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

    if (!pubs_and_subs_created_) {

        gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
                node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
                "~/" + kConnectGazeboToRosSubtopic, 1);
        gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;
        gzdbg<<"advertised ~/" + kConnectGazeboToRosSubtopic + "\n";

        for (int i = 0; i<n_seg; i++){
            for (int j=0; j<segments[i].n_slpstr; j++){
                segments[i].slpstr[j].propulsion_slipstream_sub_ = node_handle_->Subscribe("~/" + namespace_ + "/" + segments[i].slpstr[j].slpstr_topic,
                                                                                           &GazeboAerodynamics::slipstream::Callback,
                                                                                           &segments[i].slpstr[j]);
                gzdbg<<"subscribing to: "<<"~/" + namespace_ + "/" + segments[i].slpstr[j].slpstr_topic<<"\n";
            }
        }
        /*
            segments[i].lift_f_vis_pub = node_handle_->Advertise<gz_visualization_msgs::VisVector>("~/" + segments[i].lift_f_vis_topic, 1);

            connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + segments[i].lift_f_vis_topic);
            connect_gazebo_to_ros_topic_msg.set_ros_topic(segments[i].lift_f_vis_topic);
            connect_gazebo_to_ros_topic_msg.set_msgtype(gz_std_msgs::ConnectGazeboToRosTopic::VIS_VECTOR);
            connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg, true);
        }

        for (int i = 0; i<n_bdy; i++){

        }
        */

        vector_vis_array_pub = node_handle_->Advertise<gz_visualization_msgs::VisVectorArray>("~/" + namespace_ + "/" + vector_vis_array_topic, 1);
        connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" + vector_vis_array_topic);
        connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" + vector_vis_array_topic);
        connect_gazebo_to_ros_topic_msg.set_msgtype(gz_std_msgs::ConnectGazeboToRosTopic::VIS_VECTOR_ARRAY);
        connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg, true);

        pubs_and_subs_created_ = true;
    }

    common::Time current_time = world->SimTime();
    double dt = (current_time - last_time).Double();
    last_time = current_time;

    //ignition::math::Vector3d pos_ref = ignition::math::Vector3d(0, 0, 0); // used as reference position (relative to and expressed in fuselage frame) to calculate moments exerted by airfoils for comparison with PX4 aerodynamic model (debugging).

    if (n_seg>0) {
        
        /*
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
        */

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
        //ignition::math::Pose3d parent_pose = parent_link->WorldPose();
#else
        ignition::math::Pose3d pose = ignitionFromGazeboMath(this->link->GetWorldPose());
        //ignition::math::Pose3d parent_pose = ignitionFromGazeboMath(parent_link->GetWorldPose());
#endif

        // log link specific data
        /*
        if (logNow) {
            this->logfile<<this->world->SimTime().Double()<<", "<<cp_wrld.X()<<", "<<cp_wrld.Y()<<", "<<cp_wrld.Z();
            this->logfile<<", "<<pose.Rot().W()<<", "<<pose.Rot().X()<<", "<<pose.Rot().Y()<<", "<<pose.Rot().Z();
        }
        */

        // iterate over employed wing segments
        for (int i = 0; i<n_seg; i++) {

            ignition::math::Vector3d cp_ = pose.Pos() + pose.Rot().RotateVector(segments[i].cp);  // segments cp-reference position expressed in world-frame

            segments[i].v_ind_cp_ = ignition::math::Vector3d(0,0,0);

            for(int j=0; j<segments[i].n_slpstr; j++){
                segments[i].slpstr[j].cp_wrld = cp_;
                segments[i].slpstr[j].GetIndVel();
                segments[i].v_ind_cp_ += segments[i].slpstr[j].v_ind_cp_;
            }

            // geometry...
            //ignition::math::Vector3d cp_;
            //cp_ = this->cp + ignition::math::Vector3d(0.0,this->segYOffset[i],0.0); // cp position of segment i wrt link-frame
            //cp_wrld = pose.Pos() + pose.Rot().RotateVector(cp_);                    // cp position of segment i wrt to world-frame

            //ignition::math::Vector3d ref_wrld = parent_pose.Pos() + parent_pose.Rot().RotateVector(pos_ref);    // debugging: a body-fixed reference point, expressed in world frame
            //ignition::math::Vector3d cp_loc = parent_pose.Rot().RotateVectorReverse(cp_wrld - ref_wrld);        // debugging: reference point -> cp, expressed in body frame

            // calculate induced flow velocity at c/4 (i.e. cp_)
            /*
                ignition::math::Vector3d p_r2cp_;                       // vector: propeller->cp_
                ignition::math::Vector3d v_ind_cp_(0,0,0);              // zero induced (slipstream) velocity by default
                ignition::math::Vector3d d_wakeI = d_wake.Normalized();  // propeller wake direction
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
            */

            // get local airspeed at cp_, expressed in world frame
            GZ_ASSERT(this->link, "Link was NULL");
#if GAZEBO_MAJOR_VERSION >= 9
            ignition::math::Vector3d vel = this->link->WorldLinearVel(segments[i].cp);
#else
            ignition::math::Vector3d vel = ignitionFromGazeboMath(this->link->GetWorldLinearVel(segments[i].cp));
#endif

            // account for induced velocity, expressed in world frame
            vel = vel - segments[i].v_ind_cp_;

            // inflow direction at cp_, expressed in world frame
            ignition::math::Vector3d velI = vel.Normalized();

            // forward, upward, spanwise direction, expressed in world frame
            ignition::math::Vector3d forwardI = pose.Rot().RotateVector(segments[i].fwd);
            ignition::math::Vector3d upwardI = pose.Rot().RotateVector(segments[i].upwd);
            ignition::math::Vector3d spanwiseI = forwardI.Cross(upwardI).Normalized();

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

            //segments[i].alpha_dot = (alpha_-segments[i].alpha_prev)/ignition::math::clamp(dt,0.001,0.1);
            //segments[i].alpha_prev = alpha_;

            // compute dynamic pressure
            double speedInLDPlane = velInLDPlane.Length();
            double q = 0.5 * rho * speedInLDPlane * speedInLDPlane;

            // aerodynamic coefficient correction terms due to control surface deflection (only effective if not stalled)

            double d_cl = 0.0;
            double d_cd = 0.0;
            double d_cm = 0.0;

            for(int j=0; j<segments[i].n_cs; j++){
#if GAZEBO_MAJOR_VERSION >= 9
                double controlAngle = segments[i].cs[j].controlJoint->Position(0);
#else
                double controlAngle = segments[i].cs[j].controlJoint->GetAngle(0).Radian();
#endif
                d_cl += segments[i].cs[j].controlJointRadToCL*controlAngle;
                d_cd += segments[i].cs[j].controlJointRadToCD*controlAngle;
                d_cm += segments[i].cs[j].controlJointRadToCM*controlAngle;
            }

            // assembling aerodynamic coefficients for pre-stall (af) and post-stall (fp)
            /*
                double cl_af = (alpha_-alpha_zlift)*cla + d_cl;
                double cd_af = pow((alpha_-alpha_dmin)/(alphaStall-alpha_dmin),2)*(cd_af_stall-cd_af_min) + cd_af_min  + d_cd;
                double cm_af = cm_af_0 + d_cm;
                
                double cl_fp = cl_fp_max*sin(2*alpha_);
                double cd_fp = cd_af_min + (cd_fp_max-cd_af_min)*pow(sin(alpha_),2);
                double cm_fp = -cm_fp_max*sin(pow(alpha_,3)/(M_PI*M_PI));
            */

            Eigen::Vector3d alpha_poly_3(1.0,alpha_,alpha_*alpha_);
            Eigen::Vector2d alpha_poly_2(1.0,alpha_);

            double cl_af = segments[i].aero_params_.c_lift_alpha.dot(alpha_poly_3) + d_cl;
            double cd_af = segments[i].aero_params_.c_drag_alpha.dot(alpha_poly_3) + d_cd;
            double cm_af = segments[i].aero_params_.c_pitch_moment_alpha.dot(alpha_poly_2) + d_cm;

            double cl_fp = segments[i].aero_params_.fp_c_lift_max*sin(2*alpha_);
            double cd_fp = segments[i].aero_params_.fp_c_drag_max*pow(sin(alpha_),2);
            double cm_fp = -segments[i].aero_params_.fp_c_pitch_moment_max*sin(pow(alpha_,3)/(M_PI*M_PI));

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
            if(alpha_>segments[i].aero_params_.alpha_max_ns + segments[i].aero_params_.alpha_blend)
                w_af = 0.0;
            else if(alpha_>segments[i].aero_params_.alpha_max_ns)
                w_af = 0.5+0.5*cos(M_PI*(alpha_ - segments[i].aero_params_.alpha_max_ns)/segments[i].aero_params_.alpha_blend);
            else if(alpha_>segments[i].aero_params_.alpha_min_ns)
                w_af = 1.0;
            else if(alpha_>segments[i].aero_params_.alpha_min_ns - segments[i].aero_params_.alpha_blend)
                w_af = 0.5+0.5*cos(M_PI*(segments[i].aero_params_.alpha_min_ns-alpha_)/segments[i].aero_params_.alpha_blend);
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
            ignition::math::Vector3d lift   = cl * q * segments[i].segArea * liftDirection;
            ignition::math::Vector3d drag   = cd * q * segments[i].segArea * dragDirection;
            ignition::math::Vector3d moment = cm * q * segments[i].segArea * momentDirection * segments[i].segChord;

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
            /*
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
                } */

            // log segment specific data
            /*
                if (logNow && segLog[i]==1) {
                    this->logfile<<", "<<cp_wrld.X()<<", "<<cp_wrld.Y()<<", "<<cp_wrld.Z();
                    this->logfile<<", "<<vel.X()<<", "<<vel.Y()<<", "<<vel.Z();
                    this->logfile<<", "<<lift.X()<<", "<<lift.Y()<<", "<<lift.Z();
                    this->logfile<<", "<<drag.X()<<", "<<drag.Y()<<", "<<drag.Z();
                    this->logfile<<", "<<v_ind_cp_.X()<<", "<<v_ind_cp_.Y()<<", "<<v_ind_cp_.Z();
                    this->logfile<<", "<<alpha_<<", "<<q<<", "<<w_af;
                }
                */

            // correct for nan or inf
            force.Correct();
            cp_.Correct();
            torque.Correct();

            // apply forces cp
            this->link->AddForceAtWorldPosition(force, cp_);
            this->link->AddTorque(torque);

            // visualization

            // world to local frame
            ignition::math::Vector3d _B_force = pose.Rot().RotateVectorReverse(force);
            ignition::math::Vector3d _B_torque = pose.Rot().RotateVectorReverse(torque);
            ignition::math::Vector3d _B_indvel = pose.Rot().RotateVectorReverse(segments[i].v_ind_cp_);

            /*
            switch(i){
                case 0:
                    _B_force = pose.Rot().RotateVectorReverse(vel);
                    break;
                case 1:
                    _B_force = pose.Rot().RotateVectorReverse(velInLDPlane);
                    break;
                case 2:
                    _B_force = pose.Rot().RotateVectorReverse(dragDirection);
                    break;
                case 3:
                    _B_force = pose.Rot().RotateVectorReverse(liftDirection);
                    break;
                case 4:
                    _B_force = pose.Rot().RotateVectorReverse(momentDirection);
                    break;
                case 5:
                    _B_force = pose.Rot().RotateVectorReverse(spanwiseI);
                    break;
                default:
                   _B_force = ignition::math::Vector3d(0,0,0);
            }
            */
            /*
            segments[i].lift_f_vis.mutable_startpoint()->set_x(segments[i].cp.X());
            segments[i].lift_f_vis.mutable_startpoint()->set_y(segments[i].cp.Y());
            segments[i].lift_f_vis.mutable_startpoint()->set_z(segments[i].cp.Z());
            segments[i].lift_f_vis.mutable_vector()->set_x(_B_force.X()/10.0);
            segments[i].lift_f_vis.mutable_vector()->set_y(_B_force.Y()/10.0);
            segments[i].lift_f_vis.mutable_vector()->set_z(_B_force.Z()/10.0);
            segments[i].lift_f_vis_pub->Publish(segments[i].lift_f_vis);
            */

            segments[i].lift_vis->mutable_color()->set_x(1-w_af);
            segments[i].lift_vis->mutable_color()->set_y(w_af);
            segments[i].lift_vis->mutable_color()->set_z(0.0);
            segments[i].lift_vis->mutable_startpoint()->set_x(segments[i].cp.X());
            segments[i].lift_vis->mutable_startpoint()->set_y(segments[i].cp.Y());
            segments[i].lift_vis->mutable_startpoint()->set_z(segments[i].cp.Z());
            segments[i].lift_vis->mutable_vector()->set_x(_B_force.X()/300.0/segments[i].segArea);
            segments[i].lift_vis->mutable_vector()->set_y(_B_force.Y()/300.0/segments[i].segArea);
            segments[i].lift_vis->mutable_vector()->set_z(_B_force.Z()/300.0/segments[i].segArea);

            if(segments[i].n_slpstr>0){
                segments[i].slpstr_vis->mutable_color()->set_x(0.0);
                segments[i].slpstr_vis->mutable_color()->set_y(1.0);
                segments[i].slpstr_vis->mutable_color()->set_z(1.0);
                segments[i].slpstr_vis->mutable_startpoint()->set_x(segments[i].cp.X());
                segments[i].slpstr_vis->mutable_startpoint()->set_y(segments[i].cp.Y());
                segments[i].slpstr_vis->mutable_startpoint()->set_z(segments[i].cp.Z());
                segments[i].slpstr_vis->mutable_vector()->set_x(_B_indvel.X());
                segments[i].slpstr_vis->mutable_vector()->set_y(_B_indvel.Y());
                segments[i].slpstr_vis->mutable_vector()->set_z(_B_indvel.Z());
            }

            if(updateCounter%100==0){
                //gzdbg<<"w_af_"<<i<<": "<<w_af<<" | alpha:"<<alpha_<<"\n";
            }

        }
        
        /*
        if (logNow)
            this->logfile<< "\n";
        */
    }
    
    if (n_bdy>0) {
        
        // logging of interesting quantities for debugging purpose. Filepath hardcoded, improve
        // log is started/stopped via command in do_log_sub_topic_ if logEnable=true

        /*
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
        */


        for(int i=0; i<n_bdy; i++){
            // get pose of body
            GZ_ASSERT(this->link, "Link was NULL");
#if GAZEBO_MAJOR_VERSION >= 9
            ignition::math::Pose3d pose = this->link->WorldPose();
#else
            ignition::math::Pose3d pose = ignitionFromGazeboMath(this->link->GetWorldPose());
#endif
            ignition::math::Vector3d cp_ = pose.Pos() + pose.Rot().RotateVector(bodies[i].cp);  // position of cp in world frame

            // Logging

            /*
        if(logNow){
            //this->logfile<<this->world->SimTime().Double()<<", "<<pose.Pos().X()<<", "<<pose.Pos().Y()<<", "<<pose.Pos().Z();
            this->logfile<<this->world->SimTime().Double()<<", "<<cp_wrld.X()<<", "<<cp_wrld.Y()<<", "<<cp_wrld.Z();
            this->logfile<<", "<<pose.Rot().W()<<", "<<pose.Rot().X()<<", "<<pose.Rot().Y()<<", "<<pose.Rot().Z();
        }
        */

            // get velocity of cp, expressed in world frame
#if GAZEBO_MAJOR_VERSION >= 9
            ignition::math::Vector3d vel = this->link->WorldLinearVel(bodies[i].cp);
#else
            ignition::math::Vector3d vel = ignitionFromGazeboMath(this->link->GetWorldLinearVel(bodies[i].cp));
#endif
            // express forward, upward and spanwise vectors in world frame
            ignition::math::Vector3d forwardI = pose.Rot().RotateVector(bodies[i].fwd);
            ignition::math::Vector3d upwardI = pose.Rot().RotateVector(bodies[i].upwd);
            ignition::math::Vector3d spanwiseI = forwardI.Cross(upwardI);
            forwardI.Normalize();
            upwardI.Normalize();
            spanwiseI.Normalize();

            // 'signed-quadratic' flow components in forward, upward and spanwise direction
            double uu = vel.Dot(forwardI)*fabs(vel.Dot(forwardI));
            double vv = vel.Dot(upwardI)*fabs(vel.Dot(upwardI));
            double ww = vel.Dot(spanwiseI)*fabs(vel.Dot(spanwiseI));

            // 'directional' drag coefficiens
            double cd_x = bodies[i].A_fus_xx * bodies[i].cd_cyl_ax;
            double cd_y = bodies[i].A_fus_yy * bodies[i].cd_cyl_lat;
            double cd_z = bodies[i].A_fus_zz * bodies[i].cd_cyl_lat;

            // calculate and apply drag fuselage drag force
            ignition::math::Vector3d drag = -this->rho/2.0*(forwardI*uu*cd_x + upwardI*vv*cd_y + spanwiseI*ww*cd_z);
            this->link->AddForceAtWorldPosition(drag, cp_);

            // world to local frame
            ignition::math::Vector3d _B_drag = pose.Rot().RotateVectorReverse(drag);
            //ignition::math::Vector3d _B_torque = pose.Rot().RotateVectorReverse(torque);

            bodies[i].force_vis->mutable_startpoint()->set_x(bodies[i].cp.X());
            bodies[i].force_vis->mutable_startpoint()->set_y(bodies[i].cp.Y());
            bodies[i].force_vis->mutable_startpoint()->set_z(bodies[i].cp.Z());
            bodies[i].force_vis->mutable_vector()->set_x(_B_drag.X()/10);
            bodies[i].force_vis->mutable_vector()->set_y(_B_drag.Y()/10);
            bodies[i].force_vis->mutable_vector()->set_z(_B_drag.Z()/10);

            // logging

            /*
        if (logNow) {
            this->logfile<<", "<<vel.X()<<", "<<vel.Y()<<", "<<vel.Z();
            this->logfile<<", "<<drag.X()<<", "<<drag.Y()<<", "<<drag.Z();
            this->logfile<< "\n";
        }
        */
        }

    }
    
    vector_vis_array_pub->Publish(vector_vis_array);
    this->updateCounter++;  // counter to time logging/debug printing

}



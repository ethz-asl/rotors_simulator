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

#include "rotors_gazebo_plugins/gazebo_aerodynamics_plugin.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(GazeboAerodynamics)

/////////////////////////////////////////////////
GazeboAerodynamics::GazeboAerodynamics()
{
    /// Initialize to defaults
    /// comments on the different variables can be found in the
    /// header fileown_plugins/liftdrag_plugin_dr.h
    
    this->rho = 1.2041;
    
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
                    bodies[i].fwd = _sdf_element->Get<V3D>("forward");
            else
            gzwarn<<"segment ["<<i<<"] is missing 'forward' element \n";

            bodies[i].fwd.Normalize();

            if (_sdf_element->HasElement("upward"))
                bodies[i].upwd = _sdf_element->Get<V3D>("upward");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'upward' elemenbodiest \n";

            bodies[i].upwd.Normalize();

            if (_sdf_element->HasElement("cp"))
                bodies[i].cp = _sdf_element->Get<V3D>("cp");
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
                segments[i].fwd = _sdf_segment->Get<V3D>("forward");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'forward' element \n";

            segments[i].fwd.Normalize();
            
            if (_sdf_segment->HasElement("upward"))
                segments[i].upwd = _sdf_segment->Get<V3D>("upward");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'upward' element \n";

            segments[i].upwd.Normalize();
            
            if (_sdf_segment->HasElement("cp"))
                segments[i].cp = _sdf_segment->Get<V3D>("cp");
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

            _sdf_segment = _sdf_segment->GetNextElement("segment");
        }
    }

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

}

/////////////////////////////////////////////////

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

    if (n_seg>0) {
        
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
#else
        ignition::math::Pose3d pose = ignitionFromGazeboMath(this->link->GetWorldPose());
#endif

        // iterate over employed wing segments
        for (int i = 0; i<n_seg; i++) {

            V3D cp_ = pose.Pos() + pose.Rot().RotateVector(segments[i].cp);  // segments cp-reference position expressed in world-frame

            segments[i].UpdateIndVel(cp_);

            // get local airspeed at cp_, expressed in world frame
            GZ_ASSERT(this->link, "Link was NULL");
#if GAZEBO_MAJOR_VERSION >= 9
            V3D vel = this->link->WorldLinearVel(segments[i].cp);
#else
            V3D vel = ignitionFromGazeboMath(this->link->GetWorldLinearVel(segments[i].cp));
#endif

            // account for induced velocity, expressed in world frame
            vel = vel - segments[i].v_ind_cp_;

            // inflow direction at cp_, expressed in world frame
            V3D velI = vel.Normalized();

            // forward, upward, spanwise direction, expressed in world frame
            V3D forwardI = pose.Rot().RotateVector(segments[i].fwd);
            V3D upwardI = pose.Rot().RotateVector(segments[i].upwd);
            V3D spanwiseI = forwardI.Cross(upwardI).Normalized();

            // velocity in lift-drag plane
            V3D velInLDPlane = vel - vel.Dot(spanwiseI)*spanwiseI;

            // get direction of drag
            V3D dragDirection = -velInLDPlane;
            dragDirection.Normalize();

            // get direction of lift
            V3D liftDirection = spanwiseI.Cross(velInLDPlane);
            liftDirection.Normalize();

            // get direction of moment
            V3D momentDirection = spanwiseI;

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
            V3D lift   = cl * q * segments[i].segArea * liftDirection;
            V3D drag   = cd * q * segments[i].segArea * dragDirection;
            V3D moment = cm * q * segments[i].segArea * momentDirection * segments[i].segChord;

            V3D force  = lift + drag;
            V3D torque = moment;

            // correct for nan or inf
            force.Correct();
            cp_.Correct();
            torque.Correct();

            // apply forces cp
            this->link->AddForceAtWorldPosition(force, cp_);
            this->link->AddTorque(torque);

            // visualization

            // world to local frame
            V3D _B_force = pose.Rot().RotateVectorReverse(force);
            V3D _B_torque = pose.Rot().RotateVectorReverse(torque);
            V3D _B_indvel = pose.Rot().RotateVectorReverse(segments[i].v_ind_cp_);

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
    }
    
    if (n_bdy>0) {

        for(int i=0; i<n_bdy; i++){
            // get pose of body
            GZ_ASSERT(this->link, "Link was NULL");
#if GAZEBO_MAJOR_VERSION >= 9
            ignition::math::Pose3d pose = this->link->WorldPose();
#else
            ignition::math::Pose3d pose = ignitionFromGazeboMath(this->link->GetWorldPose());
#endif
            V3D cp_ = pose.Pos() + pose.Rot().RotateVector(bodies[i].cp);  // position of cp in world frame

            // get velocity of cp, expressed in world frame
#if GAZEBO_MAJOR_VERSION >= 9
            V3D vel = this->link->WorldLinearVel(bodies[i].cp);
#else
            V3D vel = ignitionFromGazeboMath(this->link->GetWorldLinearVel(bodies[i].cp));
#endif
            // express forward, upward and spanwise vectors in world frame
            V3D forwardI = pose.Rot().RotateVector(bodies[i].fwd);
            V3D upwardI = pose.Rot().RotateVector(bodies[i].upwd);
            V3D spanwiseI = forwardI.Cross(upwardI);
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
            V3D drag = -this->rho/2.0*(forwardI*uu*cd_x + upwardI*vv*cd_y + spanwiseI*ww*cd_z);
            this->link->AddForceAtWorldPosition(drag, cp_);

            // world to local frame
            V3D _B_drag = pose.Rot().RotateVectorReverse(drag);
            //V3D _B_torque = pose.Rot().RotateVectorReverse(torque);

            bodies[i].force_vis->mutable_startpoint()->set_x(bodies[i].cp.X());
            bodies[i].force_vis->mutable_startpoint()->set_y(bodies[i].cp.Y());
            bodies[i].force_vis->mutable_startpoint()->set_z(bodies[i].cp.Z());
            bodies[i].force_vis->mutable_vector()->set_x(_B_drag.X()/10);
            bodies[i].force_vis->mutable_vector()->set_y(_B_drag.Y()/10);
            bodies[i].force_vis->mutable_vector()->set_z(_B_drag.Z()/10);
        }
    }
    
    vector_vis_array_pub->Publish(vector_vis_array);
    this->updateCounter++;  // counter to time logging/debug printing

}



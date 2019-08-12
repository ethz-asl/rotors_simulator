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
GazeboAerodynamics::~GazeboAerodynamics()
{

    for(int i=0; i<n_seg_; i++){
        delete[] segments_[i].slpstr;
        delete[] segments_[i].cs;
    }
    delete[] segments_;
    delete[] bodies_;
    
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
    
    this->world = this->model->GetWorld();
    GZ_ASSERT(this->world, "GazeboAerodynamics world pointer is NULL");
    
    GZ_ASSERT(_sdf, "GazeboAerodynamics _sdf pointer is NULL");
    
    namespace_.clear();
    
    /*
    gzdbg<<"model name: "<<model->GetName()<<"\n";
    model->Print("");
    gzdbg<<"model child count: "<<model->GetChildCount()<<"\n";
    */

    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";
    
    node_handle_ = transport::NodePtr(new transport::Node());
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

    last_time_ = world->SimTime(); // ini last time

    if (_sdf->HasElement("aeroForcesVis"))
        vector_vis_array_topic_ = _sdf->Get<std::string>("aeroForcesVis");
    else
        vector_vis_array_topic_ = "aero_forces_vis";

    vector_vis_array_.mutable_header()->mutable_stamp()->set_sec(0.0);
    vector_vis_array_.mutable_header()->mutable_stamp()->set_nsec(0.0);
    vector_vis_array_.mutable_header()->set_frame_id(this->link->GetName());

    if (_sdf->HasElement("body")) {

        sdf::ElementPtr _sdf_body = _sdf->GetElement("body");
        sdf::ElementPtr _sdf_element = _sdf_body->GetElement("element");

        while (_sdf_element) {
            _sdf_element = _sdf_element->GetNextElement("element");
            ++n_bdy_;
        }

        gzdbg<<"found "<<n_bdy_<<" body element(s) for this link. \n";
        bodies_ = new Body [n_bdy_];

        _sdf_element = _sdf_body->GetElement("element");

        for(int i=0; i<n_bdy_; i++){
            gzdbg<<"processing body-element nr: "<<i<<" \n";

            bodies_[i].index = i;

            if (_sdf_element->HasElement("forward"))
                bodies_[i].fwd = _sdf_element->Get<V3D>("forward");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'forward' element \n";

            bodies_[i].fwd.Normalize();

            if (_sdf_element->HasElement("upward"))
                bodies_[i].upwd = _sdf_element->Get<V3D>("upward");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'upward' elemenbodiest \n";

            bodies_[i].upwd.Normalize();

            if (_sdf_element->HasElement("cp"))
                bodies_[i].cp = _sdf_element->Get<V3D>("cp");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'cp' element \n";

            if (_sdf_element->HasElement("aBdyXX"))
                bodies_[i].a_fus_xx = _sdf_element->Get<double>("aBdyXX");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'aBdyXX' element \n";

            if (_sdf_element->HasElement("aBdyYY"))
                bodies_[i].a_fus_yy = _sdf_element->Get<double>("aBdyYY");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'aBdyYY' element \n";

            if (_sdf_element->HasElement("aBdyZZ"))
                bodies_[i].a_fus_zz = _sdf_element->Get<double>("aBdyZZ");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'aBdyZZ' element \n";

            bodies_[i].force_vis = vector_vis_array_.add_vector();

            bodies_[i].force_vis->set_ns(namespace_+"/bdy");
            bodies_[i].force_vis->set_id(bodies_[i].index);
            bodies_[i].force_vis->mutable_scale()->set_x(0.025);
            bodies_[i].force_vis->mutable_scale()->set_y(0.05);
            bodies_[i].force_vis->mutable_scale()->set_z(0.05);
            bodies_[i].force_vis->mutable_color()->set_x(1.0);
            bodies_[i].force_vis->mutable_color()->set_y(0.0);
            bodies_[i].force_vis->mutable_color()->set_z(0.0);
        }
    }

    if (_sdf->HasElement("airfoil")) {

        sdf::ElementPtr _sdf_airfoil = _sdf->GetElement("airfoil");
        sdf::ElementPtr _sdf_segment = _sdf_airfoil->GetElement("segment");
        
        while (_sdf_segment) {
            _sdf_segment = _sdf_segment->GetNextElement("segment");
            ++n_seg_;
        }
        
        gzdbg<<"found "<<n_seg_<<" airfoil segment(s) for this link. \n";
        segments_ = new Segment [n_seg_];
        
        _sdf_segment = _sdf_airfoil->GetElement("segment");
        
        for(int i=0; i<n_seg_; i++){
            gzdbg<<"processing airfoil-segment nr: "<<i<<" \n";

            if (_sdf_segment->HasElement("forward"))
                segments_[i].fwd = _sdf_segment->Get<V3D>("forward");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'forward' element \n";

            segments_[i].fwd.Normalize();
            
            if (_sdf_segment->HasElement("upward"))
                segments_[i].upwd = _sdf_segment->Get<V3D>("upward");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'upward' element \n";

            segments_[i].upwd.Normalize();
            
            if (_sdf_segment->HasElement("cp"))
                segments_[i].cp = _sdf_segment->Get<V3D>("cp");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'cp' element \n";

            if (_sdf_segment->HasElement("seg_area"))
                segments_[i].seg_area = _sdf_segment->Get<double>("seg_area");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'seg_area' element \n";

            if (_sdf_segment->HasElement("seg_chord"))
                segments_[i].seg_chord = _sdf_segment->Get<double>("seg_chord");
            else
                gzwarn<<"segment ["<<i<<"] is missing 'seg_chord' element \n";

            if (_sdf_segment->HasElement("aeroParamsYAML")) {
                std::string aero_params_yaml =
                        _sdf_segment->GetElement("aeroParamsYAML")->Get<std::string>();
                segments_[i].aero_params.LoadAeroParamsYAML(aero_params_yaml);

            } else {
                gzwarn<<"segment ["<<i<<"] is missing aerodynamic paramaters YAML file, "
                     <<"using default parameters.\n";
            }

            // get control joints
            if (_sdf_segment->HasElement("control")) {
                sdf::ElementPtr _sdf_control = _sdf_segment->GetElement("control");
                sdf::ElementPtr _sdf_cs = _sdf_control->GetElement("cs");

                // get number of control inputs on this particular segment
                while (_sdf_cs) {
                    _sdf_cs = _sdf_cs->GetNextElement("cs");
                    ++segments_[i].n_cs;
                }

                gzdbg<<"found "<<segments_[i].n_cs<<" control surface segment(s) for segment ["<<i<<"]. \n";
                _sdf_cs = _sdf_control->GetElement("cs");
                segments_[i].cs = new ControlSurface [segments_[i].n_cs];

                for(int j=0; j<segments_[i].n_cs; j++){

                    if(_sdf_cs->HasElement("fromTopic"))
                        segments_[i].cs[j].from_topic = _sdf_cs->Get<bool>("fromTopic");

                    if (_sdf_cs->HasElement("controlJoint")){
                        std::string joint_name = _sdf_cs->Get<std::string>("controlJoint");
                        segments_[i].cs[j].control_joint = model->GetJoint(joint_name);

                        if (model->GetJoint(joint_name) == nullptr)
                            gzwarn << "joint [" << joint_name << "] not found \n";

                    } else if(!segments_[i].cs[j].from_topic) {
                        gzwarn<<"control surface ["<<j<<"] of segment ["<<i<<"] is missing 'controlJoint' element and won't be effective \n";
                    }

                    if(segments_[i].cs[j].from_topic){
                        if (_sdf_cs->HasElement("csRefTopic")) {
                            segments_[i].cs[j].cs_ref_topic = _sdf_cs->Get<std::string>("csRefTopic");
                        } else {
                            gzwarn<<"control surface ["<<j<<"] of segment ["<<i<<"] is missing 'csRefTopic' element and won't be effective \n";
                            segments_[i].cs[j].from_topic = false;
                        }
                    }

                    if (_sdf_cs->HasElement("radToCLift")) {
                        segments_[i].cs[j].control_joint_rad_to_cl = _sdf_cs->Get<V3D>("radToCLift");
                    } else {
                        gzwarn<<"control surface ["<<j<<"] of segment ["<<i<<"] is missing 'radToCLift' element \n";
                    }

                    if (_sdf_cs->HasElement("radToCDrag"))
                        segments_[i].cs[j].control_joint_rad_to_cd = _sdf_cs->Get<V3D>("radToCDrag");
                    else
                        gzwarn<<"control surface ["<<j<<"] of segment ["<<i<<"] is missing 'radToCDrag' element \n";

                    if (_sdf_cs->HasElement("radToCPitch"))
                        segments_[i].cs[j].control_joint_rad_to_cm = _sdf_cs->Get<V3D>("radToCPitch");
                    else
                        gzwarn<<"control surface ["<<j<<"] of segment ["<<i<<"] is missing 'radToCPitch' element \n";

                    if (_sdf_cs->HasElement("radToAoAB"))
                        segments_[i].cs[j].d_aoa_b_d_delta_cs = _sdf_cs->Get<double>("radToAoAB");
                    else
                        gzwarn<<"control surface ["<<j<<"] of segment ["<<i<<"] is missing 'radToAoAB' element \n";

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
                    ++segments_[i].n_slpstr;
                }

                gzdbg<<"found "<<segments_[i].n_slpstr<<" slipstream(s) for segment ["<<i<<"]. \n";
                _sdf_ind_vel = _sdf_segment->GetElement("indVel");
                segments_[i].slpstr = new Slipstream [segments_[i].n_slpstr];

                for(int j=0; j<segments_[i].n_slpstr; j++){

                    if(_sdf_ind_vel->HasElement("topic")){
                        segments_[i].slpstr[j].slpstr_topic = _sdf_ind_vel->Get<std::string>("topic");
                    } else {
                        gzwarn<<"slipstream ["<<j<<"] of segment ["<<i<<"] is missing 'radToCPitch' element \n";
                    }

                    _sdf_ind_vel = _sdf_ind_vel->GetNextElement("indVel");
                }

                if(segments_[i].n_slpstr>0){
                    segments_[i].slpstr_vis = vector_vis_array_.add_vector();
                    segments_[i].slpstr_vis->set_ns(namespace_+"/slpstr");
                    segments_[i].slpstr_vis->set_id(i);
                    segments_[i].slpstr_vis->mutable_scale()->set_x(0.025);
                    segments_[i].slpstr_vis->mutable_scale()->set_y(0.05);
                    segments_[i].slpstr_vis->mutable_scale()->set_z(0.05);
                    segments_[i].slpstr_vis->mutable_color()->set_x(1.0);
                    segments_[i].slpstr_vis->mutable_color()->set_y(1.0);
                    segments_[i].slpstr_vis->mutable_color()->set_z(1.0);
                }
            }

            segments_[i].lift_vis = vector_vis_array_.add_vector();
            segments_[i].lift_vis->set_ns(namespace_+"/lift");
            segments_[i].lift_vis->set_id(i);
            segments_[i].lift_vis->mutable_scale()->set_x(0.025);
            segments_[i].lift_vis->mutable_scale()->set_y(0.05);
            segments_[i].lift_vis->mutable_scale()->set_z(0.05);
            segments_[i].lift_vis->mutable_color()->set_x(1.0);
            segments_[i].lift_vis->mutable_color()->set_y(1.0);
            segments_[i].lift_vis->mutable_color()->set_z(1.0);

            _sdf_segment = _sdf_segment->GetNextElement("segment");
        }
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

        for (int i = 0; i<n_seg_; i++){
            for (int j=0; j<segments_[i].n_slpstr; j++){
                segments_[i].slpstr[j].propulsion_slipstream_sub_ = node_handle_->Subscribe("~/" + namespace_ + "/" + segments_[i].slpstr[j].slpstr_topic,
                                                                                            &GazeboAerodynamics::Slipstream::Callback,
                                                                                            &segments_[i].slpstr[j]);
                gzdbg<<"subscribing to: "<<"~/" + namespace_ + "/" + segments_[i].slpstr[j].slpstr_topic<<"\n";
            }
            for (int j=0; j<segments_[i].n_cs; j++){
                if(segments_[i].cs[j].from_topic){
                    segments_[i].cs[j].control_ref_sub = node_handle_->Subscribe("~/" + namespace_ + "/" + segments_[i].cs[j].cs_ref_topic,
                                                                                 &GazeboAerodynamics::ControlSurface::Callback,
                                                                                 &segments_[i].cs[j]);
                    gzdbg<<"sub to: ~/" + namespace_ + "/" + segments_[i].cs[j].cs_ref_topic + "\n";
                }
            }
        }

        vector_vis_array_pub_ = node_handle_->Advertise<gz_visualization_msgs::VisVectorArray>("~/" + namespace_ + "/" + vector_vis_array_topic_, 1);
        connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" + vector_vis_array_topic_);
        connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" + vector_vis_array_topic_);
        connect_gazebo_to_ros_topic_msg.set_msgtype(gz_std_msgs::ConnectGazeboToRosTopic::VIS_VECTOR_ARRAY);
        connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg, true);

        pubs_and_subs_created_ = true;
    }

    common::Time current_time = world->SimTime();
    double dt = (current_time - last_time_).Double();
    last_time_ = current_time;

    if (n_seg_>0) {
        
        // pose of body and parent
#if GAZEBO_MAJOR_VERSION >= 9
        ignition::math::Pose3d pose = this->link->WorldPose();
#else
        ignition::math::Pose3d pose = ignitionFromGazeboMath(this->link->GetWorldPose());
#endif

        // iterate over employed wing segments
        for (int i = 0; i<n_seg_; i++) {

            V3D cp = pose.Pos() + pose.Rot().RotateVector(segments_[i].cp);  // segments cp-reference position expressed in world-frame

            segments_[i].UpdateIndVel(cp);

            // get local airspeed at cp, expressed in world frame
            GZ_ASSERT(this->link, "Link was NULL");
#if GAZEBO_MAJOR_VERSION >= 9
            V3D vel = this->link->WorldLinearVel(segments_[i].cp);
#else
            V3D vel = ignitionFromGazeboMath(this->link->GetWorldLinearVel(segments_[i].cp));
#endif

            // account for induced velocity, expressed in world frame
            vel = vel - segments_[i].v_ind_cp;

            // inflow direction at cp, expressed in world frame
            V3D velI = vel.Normalized();

            // forward, upward, spanwise direction, expressed in world frame
            V3D forward_i = pose.Rot().RotateVector(segments_[i].fwd);
            V3D upward_i = pose.Rot().RotateVector(segments_[i].upwd);
            V3D spanwise_i = forward_i.Cross(upward_i).Normalized();

            // velocity in lift-drag plane
            V3D vel_in_ld_plane = vel - vel.Dot(spanwise_i)*spanwise_i;

            // get direction of drag
            V3D drag_dir = -vel_in_ld_plane;
            drag_dir.Normalize();

            // get direction of lift
            V3D lift_dir = spanwise_i.Cross(vel_in_ld_plane);
            lift_dir.Normalize();

            // get direction of moment
            V3D moment_dir = spanwise_i;

            // get sweep (angle between velI and lift-drag-plane) (aka sideslip)
            //double cos_sweep_angle = ignition::math::clamp(velI.Dot(-drag_dir), -1.0, 1.0);

            // compute angle of attack
            double alpha = acos(-drag_dir.Dot(forward_i)); // [0,pi]
            if (drag_dir.Dot(upward_i)<=0) {
                alpha*=-1;
            }

            //segments_[i].alpha_dot = (alpha-segments_[i].alpha_prev)/ignition::math::clamp(dt,0.001,0.1);
            //segments_[i].alpha_prev = alpha;

            // compute dynamic pressure
            double q = 0.5 * rho_ * vel_in_ld_plane.Dot(vel_in_ld_plane);

            // aerodynamic coefficient correction terms due to control surface deflection (only effective if not stalled)
            double d_cl = 0.0;
            double d_cd = 0.0;
            double d_cm = 0.0;

            // shift of stall angle due to control surface deflection
            double d_alpha_max_ns = 0.0;

            for(int j=0; j<segments_[i].n_cs; j++){
                double control_defl;
                if (segments_[i].cs[j].from_topic) {
                    control_defl = segments_[i].cs[j].cs_ref;

                } else if (segments_[i].cs[j].control_joint){
#if GAZEBO_MAJOR_VERSION >= 9
                    control_defl = segments_[i].cs[j].control_joint->Position(0);
#else
                    control_defl = segments_[i].cs[j].control_joint->GetAngle(0).Radian();
#endif
                } else {
                    control_defl = 0.0;
                }

                if (!std::isfinite(control_defl))
                    control_defl = 0.0;

                d_cl += segments_[i].cs[j].control_joint_rad_to_cl[2] +
                        segments_[i].cs[j].control_joint_rad_to_cl[0]*control_defl +
                        segments_[i].cs[j].control_joint_rad_to_cl[1]*control_defl*control_defl;

                d_cd += segments_[i].cs[j].control_joint_rad_to_cd[2] +
                        segments_[i].cs[j].control_joint_rad_to_cd[0]*control_defl +
                        segments_[i].cs[j].control_joint_rad_to_cd[1]*control_defl*control_defl;

                d_cm += segments_[i].cs[j].control_joint_rad_to_cm[2] +
                        segments_[i].cs[j].control_joint_rad_to_cm[0]*control_defl +
                        segments_[i].cs[j].control_joint_rad_to_cm[1]*control_defl*control_defl;

                d_alpha_max_ns += segments_[i].cs[j].d_aoa_b_d_delta_cs*control_defl;
            }

            // assembling aerodynamic coefficients for pre-stall (af) and post-stall (fp)
            Eigen::Vector3d alpha_poly_3(1.0,alpha,alpha*alpha);
            Eigen::Vector2d alpha_poly_2(1.0,alpha);

            double cl_af = segments_[i].aero_params.c_lift_alpha.dot(alpha_poly_3) + d_cl;
            double cd_af = segments_[i].aero_params.c_drag_alpha.dot(alpha_poly_3) + d_cd;
            double cm_af = segments_[i].aero_params.c_pitch_moment_alpha.dot(alpha_poly_2) + d_cm;

            double cl_fp = segments_[i].aero_params.fp_c_lift_max*sin(2*alpha);
            double cd_fp = segments_[i].aero_params.fp_c_drag_max*pow(sin(alpha),2);
            double cm_fp = -segments_[i].aero_params.fp_c_pitch_moment_max*sin(pow(alpha,3)/(M_PI*M_PI));

            // form mixing weight to combine pre- and post-stall models
            double w_af;
            if(alpha>segments_[i].aero_params.alpha_max_ns + segments_[i].aero_params.alpha_blend + d_alpha_max_ns)
                w_af = 0.0;
            else if(alpha>segments_[i].aero_params.alpha_max_ns + d_alpha_max_ns)
                w_af = 0.5+0.5*cos(M_PI*(alpha - segments_[i].aero_params.alpha_max_ns)/segments_[i].aero_params.alpha_blend);
            else if(alpha>segments_[i].aero_params.alpha_min_ns + d_alpha_max_ns)
                w_af = 1.0;
            else if(alpha>segments_[i].aero_params.alpha_min_ns - segments_[i].aero_params.alpha_blend + d_alpha_max_ns)
                w_af = 0.5+0.5*cos(M_PI*(segments_[i].aero_params.alpha_min_ns-alpha)/segments_[i].aero_params.alpha_blend);
            else
                w_af = 0.0;

            // form weighted sum of pre-stall and post_stall (flat-plate) contributions to aerodynamic coefficients
            double cl = w_af*cl_af+(1-w_af)*cl_fp;
            double cd = w_af*cd_af+(1-w_af)*cd_fp;
            double cm = w_af*cm_af+(1-w_af)*cm_fp;

            // original plugin incorporates sweep (sideslip)
            //cl = cl*cos_sweep_angle;
            //cd = cd*cos_sweep_angle;
            //cm = cm*cos_sweep_angle;

            // set to zero if desired...
            //cm = 0.0;
            //cd = 0.0;
            //cl = 0.0;

            // assemble final forces and moments
            V3D lift   = cl * q * segments_[i].seg_area * lift_dir;
            V3D drag   = cd * q * segments_[i].seg_area * drag_dir;
            V3D moment = cm * q * segments_[i].seg_area * moment_dir * segments_[i].seg_chord;

            V3D force  = lift + drag;
            V3D torque = moment;

            // correct for nan or inf
            force.Correct();
            cp.Correct();
            torque.Correct();

            // apply forces cp
            this->link->AddForceAtWorldPosition(force, cp);
            this->link->AddTorque(torque);

            // visualization

            // world to local frame
            V3D _B_force = pose.Rot().RotateVectorReverse(force);
            V3D _B_torque = pose.Rot().RotateVectorReverse(torque);
            V3D _B_indvel = pose.Rot().RotateVectorReverse(segments_[i].v_ind_cp);

            segments_[i].lift_vis->mutable_color()->set_x(1-w_af);
            segments_[i].lift_vis->mutable_color()->set_y(w_af);
            segments_[i].lift_vis->mutable_color()->set_z(0.0);
            segments_[i].lift_vis->mutable_startpoint()->set_x(segments_[i].cp.X());
            segments_[i].lift_vis->mutable_startpoint()->set_y(segments_[i].cp.Y());
            segments_[i].lift_vis->mutable_startpoint()->set_z(segments_[i].cp.Z());
            segments_[i].lift_vis->mutable_vector()->set_x(_B_force.X());
            segments_[i].lift_vis->mutable_vector()->set_y(_B_force.Y());
            segments_[i].lift_vis->mutable_vector()->set_z(_B_force.Z());

            if(segments_[i].n_slpstr>0){
                segments_[i].slpstr_vis->mutable_color()->set_x(0.0);
                segments_[i].slpstr_vis->mutable_color()->set_y(1.0);
                segments_[i].slpstr_vis->mutable_color()->set_z(1.0);
                segments_[i].slpstr_vis->mutable_startpoint()->set_x(segments_[i].cp.X());
                segments_[i].slpstr_vis->mutable_startpoint()->set_y(segments_[i].cp.Y());
                segments_[i].slpstr_vis->mutable_startpoint()->set_z(segments_[i].cp.Z());
                segments_[i].slpstr_vis->mutable_vector()->set_x(_B_indvel.X());
                segments_[i].slpstr_vis->mutable_vector()->set_y(_B_indvel.Y());
                segments_[i].slpstr_vis->mutable_vector()->set_z(_B_indvel.Z());
            }

            if(update_counter_%100==0){
                //gzdbg<<"w_af_"<<i<<": "<<w_af<<" | alpha:"<<alpha<<"\n";
            }
        }
    }
    
    if (n_bdy_>0) {

        for(int i=0; i<n_bdy_; i++){
            // get pose of body
            GZ_ASSERT(this->link, "Link was NULL");
#if GAZEBO_MAJOR_VERSION >= 9
            ignition::math::Pose3d pose = this->link->WorldPose();
#else
            ignition::math::Pose3d pose = ignitionFromGazeboMath(this->link->GetWorldPose());
#endif
            V3D cp = pose.Pos() + pose.Rot().RotateVector(bodies_[i].cp);  // position of cp in world frame

            // get velocity of cp, expressed in world frame
#if GAZEBO_MAJOR_VERSION >= 9
            V3D vel = this->link->WorldLinearVel(bodies_[i].cp);
#else
            V3D vel = ignitionFromGazeboMath(this->link->GetWorldLinearVel(bodies_[i].cp));
#endif
            // express forward, upward and spanwise vectors in world frame
            V3D forward_i = pose.Rot().RotateVector(bodies_[i].fwd);
            V3D upward_i = pose.Rot().RotateVector(bodies_[i].upwd);
            V3D spanwise_i = forward_i.Cross(upward_i);
            forward_i.Normalize();
            upward_i.Normalize();
            spanwise_i.Normalize();

            // 'signed-quadratic' flow components in forward, upward and spanwise direction
            double uu = vel.Dot(forward_i)*fabs(vel.Dot(forward_i));
            double vv = vel.Dot(upward_i)*fabs(vel.Dot(upward_i));
            double ww = vel.Dot(spanwise_i)*fabs(vel.Dot(spanwise_i));

            // 'directional' drag coefficiens
            double cd_x = bodies_[i].a_fus_xx * bodies_[i].cd_cyl_ax;
            double cd_y = bodies_[i].a_fus_yy * bodies_[i].cd_cyl_lat;
            double cd_z = bodies_[i].a_fus_zz * bodies_[i].cd_cyl_lat;

            // calculate and apply drag fuselage drag force
            V3D drag = -this->rho_/2.0*(forward_i*uu*cd_x + upward_i*vv*cd_y + spanwise_i*ww*cd_z);
            this->link->AddForceAtWorldPosition(drag, cp);

            // world to local frame
            V3D _B_drag = pose.Rot().RotateVectorReverse(drag);
            //V3D _B_torque = pose.Rot().RotateVectorReverse(torque);

            bodies_[i].force_vis->mutable_startpoint()->set_x(bodies_[i].cp.X());
            bodies_[i].force_vis->mutable_startpoint()->set_y(bodies_[i].cp.Y());
            bodies_[i].force_vis->mutable_startpoint()->set_z(bodies_[i].cp.Z());
            bodies_[i].force_vis->mutable_vector()->set_x(_B_drag.X()/10);
            bodies_[i].force_vis->mutable_vector()->set_y(_B_drag.Y()/10);
            bodies_[i].force_vis->mutable_vector()->set_z(_B_drag.Z()/10);
        }
    }
    
    vector_vis_array_pub_->Publish(vector_vis_array_);
    this->update_counter_++;  // counter to time logging/debug printing

}

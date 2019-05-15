/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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
 *
 * Modifications by David Rohr, drohr@student.ethz.ch
 *
 */

#include "rotors_gazebo_plugins/gazebo_propulsion_plugin.h"

namespace gazebo {

GazeboPropulsion::~GazeboPropulsion() {
    update_connection_->~Connection();
    delete[] propellers_;
    gzdbg<<"GazeboPropulsion plugin destructed\n";
}

void GazeboPropulsion::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    
    gzdbg<<"Load started\n";
    model_ = _model;
    this->world_ = this->model_->GetWorld();

    namespace_.clear();

    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzerr<<"[gazebo_propulsion] Please specify a robotNamespace.\n";

    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init();

    /*
    const YAML::Node nodeTest = YAML::Load("{name: Brewers, city: Milwaukee}");
    std::string id_string;
    gzdbg<<"yaml safe get: "<<SafeGet<std::string>(nodeTest, "name", &id_string)<<"\n";

    YAML::Node primes = YAML::Load("1");
    gzdbg<<"IsDefined: "<<primes.IsDefined()<<std::endl;
    gzdbg<<"IsMap: "<<primes.IsMap()<<std::endl;
    gzdbg<<"IsNull: "<<primes.IsNull()<<std::endl;
    gzdbg<<"IsScalar: "<<primes.IsScalar()<<std::endl;
    gzdbg<<"IsSequence: "<<primes.IsSequence()<<std::endl;
    */

    if (_sdf->HasElement("prop")){
        sdf::ElementPtr _sdf_propeller = _sdf->GetElement("prop");

        while (_sdf_propeller) {
            _sdf_propeller = _sdf_propeller->GetNextElement("prop");
            ++num_props_;
        }

        gzdbg<<"Found "<<num_props_<<" propellers for this plugin. \n";
        propellers_ = new Propeller [num_props_];
        _sdf_propeller = _sdf->GetElement("prop");

        for(int idx=0; idx<num_props_; idx++){
            gzdbg<<"Processing propeller Nr: "<<idx<<" \n";

            if (_sdf_propeller->HasElement("linkNameParent")) {
                propellers_[idx].parent_link =  model_->GetLink(_sdf_propeller->Get<std::string>("linkNameParent"));

                if (propellers_[idx].parent_link == NULL)
                    gzthrow("[gazebo_propulsion] Couldn't find specified parent link: "<<_sdf_propeller->Get<std::string>("linkNameParent")<<"\n");
            } else {
                gzerr<<"[gazebo_propulsion] Please specify propeller linkNameParent.\n";
            }

            if (_sdf_propeller->HasElement("cp"))
                propellers_[idx].p_cp = _sdf_propeller->Get<V3D>("cp");
            else
                gzwarn<<"_sdf_propeller ["<<idx<<"] is missing 'cp' element \n";

            if (_sdf_propeller->HasElement("axisPosRot"))
                propellers_[idx].p_joint = _sdf_propeller->Get<V3D>("axisPosRot");
            else
                gzwarn<<"_sdf_propeller ["<<idx<<"] is missing 'axisPosRot' element \n";

            propellers_[idx].p_joint.Normalize();

            if (_sdf_propeller->HasElement("turnDir")) {
                std::string turning_direction = _sdf_propeller->Get<std::string>("turnDir");

                if (turning_direction == "cw")
                    propellers_[idx].turning_direction = -1; // pos. thrust and axis of pos. rot. have opposite signs
                else if (turning_direction == "ccw")
                    propellers_[idx].turning_direction = 1;  // pos. thrust and axis of pos. rot. in same direction
                else
                    gzerr<<"[gazebo_propulsion] Please only use 'cw' or 'ccw' to specify turningDirection.\n";
            } else {
                gzwarn<<"Please specify a turning direction ('cw' or 'ccw'), defaults to ccw\n";
            }

            if (_sdf_propeller->HasElement("speedRefTopic")) {
                propellers_[idx].omega_ref_subtopic = _sdf_propeller->GetElement("speedRefTopic")->Get<std::string>();
                if(propellers_[idx].omega_ref_subtopic.empty())
                    gzwarn<<"No speedRefTopic specified, will not generate thrust\n";
            } else {
                gzwarn<<"No speedRefTopic specified, will not generate thrust\n";
            }

            if (_sdf_propeller->HasElement("propParamsYAML")) {
                std::string prop_paramsyaml = _sdf_propeller->GetElement("propParamsYAML")->Get<std::string>();
                propellers_[idx].prop_params.LoadPropParamsYAML(prop_paramsyaml);
            } else {
                gzwarn<<"propeller ["<<idx<<"] is missing propeller parameters YAML file, "
                     <<"using default parameters.\n";
            }

            // inertia tensor (assuming flat disk) expressed in propeller* frame
            // propeller* frame: parent-fixed and x-axis aligned with prop axis
            M3D inertia_prop = propellers_[idx].prop_params.mass *
                    pow(propellers_[idx].prop_params.diameter/2,2) *
                    M3D(0.5,0,0,0,0.25,0,0,0,0.25);

            V3D rot_axis = -propellers_[idx].p_joint.Cross(V3D(1,0,0));
            rot_axis.Normalize();
            double rot_angle = acos(propellers_[idx].p_joint.Dot(V3D(1,0,0)));
            M3D R_pa_pr; R_pa_pr.Axis(rot_axis,rot_angle);  // Maps from propeller* to parent frame

            // inertia tensor (assuming flat disk) expressed in parent frame
            propellers_[idx].inertia = R_pa_pr*(inertia_prop*(R_pa_pr.Inverse()));

            if (_sdf_propeller->HasElement("slpstrTopic")) {
                propellers_[idx].prop_slpstr_pubtopic = _sdf_propeller->GetElement("slpstrTopic")->Get<std::string>();
                if(propellers_[idx].prop_slpstr_pubtopic.empty())
                    gzwarn<<"No slisptream topic specified, won't publish.\n";
            } else {
                gzwarn<<"No slisptream topic specified, won't publish.\n";
            }

            if (_sdf_propeller->HasElement("visTopic")) {
                propellers_[idx].vector_vis_array_topic = _sdf_propeller->GetElement("visTopic")->Get<std::string>();
                if(propellers_[idx].vector_vis_array_topic.empty())
                    gzdbg<<"No visualization topic specified, won't publish.\n";
            } else {
                gzdbg<<"No visualization topic specified, won't publish.\n";
            }

            propellers_[idx].vector_vis_array_msg.mutable_header()->mutable_stamp()->set_sec(0.0);
            propellers_[idx].vector_vis_array_msg.mutable_header()->mutable_stamp()->set_nsec(0.0);
            propellers_[idx].vector_vis_array_msg.mutable_header()->set_frame_id(propellers_[idx].parent_link->GetName());

            // setup visualization message
            for(int idx_vis=0; idx_vis<propellers_[idx].vec_vis.size(); idx_vis++){
                propellers_[idx].vec_vis[idx_vis] = propellers_[idx].vector_vis_array_msg.add_vector();
                propellers_[idx].vec_vis[idx_vis]->set_ns(namespace_+std::to_string(idx));
                propellers_[idx].vec_vis[idx_vis]->set_id(idx_vis);
                propellers_[idx].vec_vis[idx_vis]->mutable_scale()->set_x(0.025);
                propellers_[idx].vec_vis[idx_vis]->mutable_scale()->set_y(0.05);
                propellers_[idx].vec_vis[idx_vis]->mutable_scale()->set_z(0.05);
                propellers_[idx].vec_vis[idx_vis]->mutable_color()->set_x(1.0);
                propellers_[idx].vec_vis[idx_vis]->mutable_color()->set_y(0.0);
                propellers_[idx].vec_vis[idx_vis]->mutable_color()->set_z(0.0);
            }

            _sdf_propeller = _sdf_propeller->GetNextElement("prop");
        }

    } else {
        gzerr<<"[gazebo_propulsion] Please specify propellers\n";
    }

    if (_sdf->HasElement("rhoAir"))
        rho_air_ = _sdf->Get<double>("rhoAir");
    else
        gzwarn<<" missing rhoAir element, using default of 1.255 kg/m^3 \n";

    // Listen to the update event. This event is broadcast everysimulation iteration.
    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboPropulsion::OnUpdate, this));

    gzdbg<<"Load completed\n";

}

// This gets called by the world update start event.
void GazeboPropulsion::OnUpdate() {

    sampling_time_ = world_->SimTime().Double() - prev_sim_time_;
    prev_sim_time_ = world_->SimTime().Double();

    if (!pubs_and_subs_created_) {
        gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
                node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
                    "~/" + kConnectGazeboToRosSubtopic, 1);
        gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;
        gzdbg<<"advertised ~/" + kConnectGazeboToRosSubtopic + "\n";

        for (int idx = 0; idx<num_props_; idx++) {

            if(!propellers_[idx].prop_slpstr_pubtopic.empty()){
                propellers_[idx].prop_slpstr_pub = node_handle_->Advertise<gz_mav_msgs::PropulsionSlipstream>
                        ("~/" + namespace_ + "/" + propellers_[idx].prop_slpstr_pubtopic, 1);
                gzdbg<<"advertising: "<<"~/" + namespace_ + "/" + propellers_[idx].prop_slpstr_pubtopic<<"\n";
            }

            if(!propellers_[idx].vector_vis_array_topic.empty()){
                propellers_[idx].vector_vis_array_pub = node_handle_->Advertise<gz_visualization_msgs::VisVectorArray>
                        ("~/" + namespace_ + "/" + propellers_[idx].vector_vis_array_topic, 1);

                connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" + propellers_[idx].vector_vis_array_topic);
                connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" + propellers_[idx].vector_vis_array_topic);
                connect_gazebo_to_ros_topic_msg.set_msgtype(gz_std_msgs::ConnectGazeboToRosTopic::VIS_VECTOR_ARRAY);
                connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg, true);
            }

            if(!propellers_[idx].omega_ref_subtopic.empty()){
                propellers_[idx].omega_ref_sub = node_handle_->Subscribe("~/" + namespace_ + "/" + propellers_[idx].omega_ref_subtopic,
                                                                         &GazeboPropulsion::Propeller::PropSpeedCallback,
                                                                         &propellers_[idx]);
            }
        }

        pubs_and_subs_created_ = true;
    }

    for (int idx = 0; idx<num_props_; idx++) {
        propellers_[idx].dt = sampling_time_;

        // pose of parent frame
#if GAZEBO_MAJOR_VERSION >= 9
        ignition::math::Pose3d pose_parent = propellers_[idx].parent_link->WorldPose();
#else
        ignition::math::Pose3d pose_parent = ignitionFromGazeboMath(propellers_[idx].parent_link->GetWorldPose());
#endif

        // Moments due to change in angular momentum of propeller, expressed in parent frame
        // Fast spinning link is problematic due to aliasing effects in the physics engine
        V3D H_Omega = propellers_[idx].parent_link->RelativeAngularVel();
        V3D H_Omega_dot = propellers_[idx].parent_link->RelativeAngularAccel();

        V3D H_moment_inertial = propellers_[idx].inertia*H_Omega_dot + H_Omega.Cross(propellers_[idx].inertia*H_Omega)
                                + propellers_[idx].inertia*propellers_[idx].p_joint*propellers_[idx].omega_dot
                                + H_Omega.Cross(propellers_[idx].inertia*propellers_[idx].p_joint*propellers_[idx].omega);
        /*
        V3D H_moment_inertial_1 = propellers_[idx].inertia*H_Omega_dot + H_Omega.Cross(propellers_[idx].inertia*H_Omega);
        V3D H_moment_inertial_2 = propellers_[idx].inertia*propellers_[idx].p_joint*propellers_[idx].omega_dot;
        V3D H_moment_inertial_3 = H_Omega.Cross(propellers_[idx].inertia*propellers_[idx].p_joint*propellers_[idx].omega);
        V3D H_moment_inertial = H_moment_inertial_1 + H_moment_inertial_2 + H_moment_inertial_3;
        */

        H_moment_inertial.Correct();
        propellers_[idx].parent_link->AddRelativeTorque(-H_moment_inertial);

        // velocity of propeller hub
#if GAZEBO_MAJOR_VERSION >= 9
        V3D body_velocity = propellers_[idx].parent_link->WorldLinearVel(propellers_[idx].p_cp);
#else
        V3D body_velocity = ignitionFromGazeboMath(propellers_[idx].parent_link->GetWorldLinearVel(propellers_[idx].p_cp));
#endif

        // propeller axis expressed in world frame, in positive rot direction
        V3D joint_axis = pose_parent.Rot().RotateVector(propellers_[idx].p_joint);
        joint_axis.Normalize();

        double rps = propellers_[idx].omega/(2*M_PI);

        V3D forward = joint_axis*propellers_[idx].turning_direction*(rps>=0 ? 1.0 : -1.0);   // positive thrust direction (for symmetric propellers, we want enable two-way operation)

        // resolve local airspeed into axial (V_inf_a) and radial (V_inf_r) component
        V3D body_velocity_radial = body_velocity - body_velocity.Dot(forward) * forward;
        V3D body_velocity_axial = body_velocity.Dot(forward) * forward;
        V3D rotor_pos = propellers_[idx].parent_link->WorldPose().Pos()
                        + pose_parent.Rot().RotateVector(propellers_[idx].p_cp);  // link position expressed in world frame

        double V_inf_a = body_velocity.Dot(forward);    // axial component of relative flow wrt propeller disk (no wind assumed)
        double V_inf_a_clmpd = std::max(V_inf_a,0.0);   // treat reverse flow as static case
        double V_inf_r = body_velocity_radial.Length(); // radial component of relative flow wrt propeller disk (no wind assumed)

        double J   = V_inf_a_clmpd/(std::max(std::abs(rps),0.1) * propellers_[idx].prop_params.diameter);    // Advance ratio (set minimum rps to prevent division by zero...)

        // ---- Thrust ----

        //double C_T = propellers_[idx].prop_params.k_t*J + propellers_[idx].prop_params.k_t0;
        //double thrust = rho_air_*pow(rps,2)*pow(propellers_[idx].prop_params.diameter,4)*C_T;
        double thrust = rho_air_*std::abs(rps)*pow(propellers_[idx].prop_params.diameter,3)*
                (propellers_[idx].prop_params.k_t*V_inf_a_clmpd +
                 std::abs(rps)*propellers_[idx].prop_params.diameter*propellers_[idx].prop_params.k_t0);

        propellers_[idx].parent_link->AddForceAtRelativePosition(thrust*forward, propellers_[idx].p_cp);

        // ---- Normal force ----

        V3D hub_force_ = -std::abs(rps*2*M_PI) * propellers_[idx].prop_params.rotor_drag_coefficient * body_velocity_radial;
        propellers_[idx].parent_link->AddForceAtRelativePosition(hub_force_, propellers_[idx].p_cp);

        // ---- Drag torque ----

        double drag_torque = -rho_air_*std::abs(rps)*pow(propellers_[idx].prop_params.diameter,4)*
                (V_inf_a_clmpd*propellers_[idx].prop_params.k_q +
                 std::abs(rps)*propellers_[idx].prop_params.diameter*propellers_[idx].prop_params.k_q0)*
                (rps>=0 ? 1.0 : -1.0);
        V3D drag_torque_ = joint_axis * drag_torque;
        propellers_[idx].parent_link->AddTorque(drag_torque_);

        // ---- Rolling moment ----

        V3D rolling_moment = -propellers_[idx].turning_direction*std::abs(rps*2*M_PI) *
                propellers_[idx].prop_params.rolling_moment_coefficient *
                body_velocity_radial;
        propellers_[idx].parent_link->AddTorque(rolling_moment);

        // ---- Propeller slipstream ----

        thrust = std::max(thrust,0.0);  //Currently, breaking (neg. thrust can result from above calc) is ignored for wake modelling since this flight regime is assumed to be entered rarely and handlig breaking would require a separate case (i.e. jf>=1, k_w would need to be set to a value smaller than one -> in breaking state, the streamlines are diverging opposite to the direction of travel, hence the induced velocity behind the prop decreases monotonically (in contrast to the non-braking case, where the induced velocity first increases behind the prop before exhibiting loss-driven decay further downstream).
        double disk_area = pow(propellers_[idx].prop_params.diameter,2)/4*M_PI;
        double w = 0.5*(-V_inf_a_clmpd + sqrt(pow(V_inf_a_clmpd,2)+2*thrust/(rho_air_*disk_area)));  // induced velocity at disk, w>=0
        double V_disk_a = V_inf_a+w;    //airflow velocity at propeller disk, w>=0

        V3D w_ds_a_;                // axial induced velocity
        V3D w_ds_r_;                // radial induced velocity
        V3D w_dir(0.0,0.0,0.0);     // propeller hub -> wake end
        V3D w_end(0.0,0.0,0.0);     // induced velocity at end of wake
        V3D w_disk(0.0,0.0,0.0);    // induced velocity at disk

        double k_w;
        double m_dot_clmpd = disk_area*(V_inf_a_clmpd + w)*rho_air_; //>=0

        if (V_disk_a>0) {
            // i.e. wake extends beyond prop...

            double jf = V_inf_a/V_disk_a;     // jet flow parameter (cf 'm' in selig paper)

            if (jf<=0)                        // hover & descent(V_inf_a<=0)
                k_w = 1.0;
            else if (jf>=0.75)                // full cruise
                k_w = 1.8;
            else
                k_w = 1.0 + jf/0.75*0.8;

            // induced velocity at disk in axial direction, accounting for losses (k_w)
            w_ds_a_ = -k_w*w*forward;

            // induced velocity at disk in radial direction
            // w_ds_r_ = -hub_force_/hub_force_.Length() * ignition::math::clamp(hub_force_.Length()/m_dot_clmpd*pow(std::abs(rps*2*M_PI)/max_rot_velocity_,0.25), 0.0, 0.8*body_velocity_radial.Length());
            w_ds_r_ = V3D(0,0,0);
            // double hub_vel_rel = w_ds_r_.Length()/V_inf_r; // debug: so far no check if hub_vel_rel < 1 ... should it be <1?

            w_disk = w_ds_a_+w_ds_r_; // downstream values already at disk for simplification

            // get wake
            if (V_inf_a >= 0) {
                // case I: No reverse free-stream, declare wake as terminated if induced velocity decayed to 1/8 of its initial value
                double t_end = log(8.0)/propellers_[idx].prop_params.d_flow;
                w_dir = -t_end*body_velocity + (7.0/8.0)*(w_ds_a_+w_ds_r_)/propellers_[idx].prop_params.d_flow;
                w_end = (w_ds_a_+w_ds_r_)/8.0;

            } else {
                // case II: Flow in wake changes direction (relative to propeller) after some time due to reverse free-stream
                double t_end = log(-w_ds_a_.Length()/V_inf_a)/propellers_[idx].prop_params.d_flow;
                w_dir = -t_end*body_velocity + (1.0-exp(-propellers_[idx].prop_params.d_flow*t_end))*(w_ds_a_+w_ds_r_)/propellers_[idx].prop_params.d_flow;
                w_end = (w_ds_a_+w_ds_r_)*exp(-propellers_[idx].prop_params.d_flow*t_end);
            }

        } else {
            // case III: wake does not extend beyond prop, reverse flow through prop disk, no wake modelled
            w_dir = V3D(0.0,0.0,0.0);
            w_end = V3D(0.0,0.0,0.0);
        }

        // ---- Fill propeller slipstream message ----

        if(propellers_[idx].prop_slpstr_pub){

            propulsion_slipstream_msg_.mutable_rotor_pos()->set_x(rotor_pos.X());
            propulsion_slipstream_msg_.mutable_rotor_pos()->set_y(rotor_pos.Y());
            propulsion_slipstream_msg_.mutable_rotor_pos()->set_z(rotor_pos.Z());

            propulsion_slipstream_msg_.mutable_ind_vel_disk()->set_x(w_disk.X());
            propulsion_slipstream_msg_.mutable_ind_vel_disk()->set_y(w_disk.Y());
            propulsion_slipstream_msg_.mutable_ind_vel_disk()->set_z(w_disk.Z());

            propulsion_slipstream_msg_.mutable_ind_vel_end()->set_x(w_end.X());
            propulsion_slipstream_msg_.mutable_ind_vel_end()->set_y(w_end.Y());
            propulsion_slipstream_msg_.mutable_ind_vel_end()->set_z(w_end.Z());

            propulsion_slipstream_msg_.mutable_wake_dir()->set_x(w_dir.X());
            propulsion_slipstream_msg_.mutable_wake_dir()->set_y(w_dir.Y());
            propulsion_slipstream_msg_.mutable_wake_dir()->set_z(w_dir.Z());

            propulsion_slipstream_msg_.mutable_timestamp()->set_sec((int)0);
            propulsion_slipstream_msg_.mutable_timestamp()->set_nsec((int)0);

            propulsion_slipstream_msg_.set_k_w(k_w);
            propulsion_slipstream_msg_.set_l_a(0.0);
            propulsion_slipstream_msg_.set_l_p(0.0);
            propulsion_slipstream_msg_.set_prop_diam(propellers_[idx].prop_params.diameter);

            propellers_[idx].prop_slpstr_pub->Publish(propulsion_slipstream_msg_);
        }

        if(propellers_[idx].vector_vis_array_pub){

            float r,g,b;
            V3D P_start = propellers_[idx].p_cp;//V3D(-0.025,0,0.15); //
            V3D P_vec;

            for(int idx_vis=0; idx_vis<propellers_[idx].vec_vis.size(); idx_vis++){

                switch (idx_vis) {
                case 0:
                    r=1;g=0;b=0;
                    P_vec = pose_parent.Rot().RotateVectorReverse(thrust*forward/10);
                    //P_vec = H_moment_inertial_1;
                    break;
                case 1:
                    r=0;g=1;b=0;
                    P_vec = pose_parent.Rot().RotateVectorReverse(hub_force_/10);
                    //P_vec = H_moment_inertial_2;
                    break;
                case 2:
                    r=0;g=0;b=1;
                    P_vec = pose_parent.Rot().RotateVectorReverse(drag_torque_);
                    //P_vec = propellers_[idx].p_joint*propellers_[idx].omega_dot;
                    //P_vec = H_moment_inertial_3;
                    break;
                case 3:
                    r=1;g=1;b=0;
                    P_vec = pose_parent.Rot().RotateVectorReverse(rolling_moment);
                    //P_vec = propellers_[idx].p_joint*propellers_[idx].omega_dot;
                    //P_vec = H_moment_inertial;
                    break;
                case 4:
                    r=1;g=0;b=1;
                    P_vec = pose_parent.Rot().RotateVectorReverse(w_dir);
                    //P_vec = 10*V3D(propellers_[idx].inertia(0,0),propellers_[idx].inertia(1,0),propellers_[idx].inertia(2,0));
                    //P_vec = 50*propellers_[idx].inertia*propellers_[idx].p_joint;
                    break;
                case 5:
                    r=0;g=1;b=1;
                    P_vec = pose_parent.Rot().RotateVectorReverse(w_disk);
                    //P_vec = 10*V3D(propellers_[idx].inertia(0,1),propellers_[idx].inertia(1,1),propellers_[idx].inertia(2,1));
                    //P_vec = 50*propellers_[idx].inertia*V3D(0,1,0);
                    break;
                case 6:
                    r=1;g=1;b=1;
                    P_vec = -H_moment_inertial;
                    //P_vec = 10*V3D(propellers_[idx].inertia(0,2),propellers_[idx].inertia(1,2),propellers_[idx].inertia(2,2));
                    //P_vec = 50*propellers_[idx].inertia*V3D(0.2,0.0,1);
                    break;
                }

                propellers_[idx].vec_vis[idx_vis]->mutable_color()->set_x(r);
                propellers_[idx].vec_vis[idx_vis]->mutable_color()->set_y(g);
                propellers_[idx].vec_vis[idx_vis]->mutable_color()->set_z(b);
                propellers_[idx].vec_vis[idx_vis]->mutable_startpoint()->set_x(P_start.X());
                propellers_[idx].vec_vis[idx_vis]->mutable_startpoint()->set_y(P_start.Y());
                propellers_[idx].vec_vis[idx_vis]->mutable_startpoint()->set_z(P_start.Z());
                propellers_[idx].vec_vis[idx_vis]->mutable_vector()->set_x(P_vec.X());
                propellers_[idx].vec_vis[idx_vis]->mutable_vector()->set_y(P_vec.Y());
                propellers_[idx].vec_vis[idx_vis]->mutable_vector()->set_z(P_vec.Z());
            }

            propellers_[idx].vector_vis_array_pub->Publish(propellers_[idx].vector_vis_array_msg);
        }
    }

    update_counter_++;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboPropulsion);
}

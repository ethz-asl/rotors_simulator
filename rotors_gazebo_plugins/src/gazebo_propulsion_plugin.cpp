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
//#include "gazebo_motor_model.h"
#include <ignition/math.hh>
#include <math.h>

namespace gazebo {

GazeboPropulsion::~GazeboPropulsion() {
    updateConnection_->~Connection();
    delete[] propellers;
    gzdbg<<"motor_model destructed\n";
}

void GazeboPropulsion::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    
    gzdbg << "GazeboPropulsion::Load started\n";
    model_ = _model;
    this->world_ = this->model_->GetWorld();

    namespace_.clear();

    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";

    node_handle_ = transport::NodePtr(new transport::Node());
    //node_handle_->Init(namespace_);
    node_handle_->Init();


    const YAML::Node nodeTest = YAML::Load("{name: Brewers, city: Milwaukee}");
    std::string id_string;
    gzdbg<<"yaml safe get: "<<safeGet<std::string>(nodeTest, "name", &id_string)<<"\n";


    /*
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
            ++n_props;
        }

        gzdbg<<"found "<<n_props<<" propellers for this plugin. \n";
        propellers = new propeller [n_props];
        _sdf_propeller = _sdf->GetElement("prop");

        for(int idx=0; idx<n_props; idx++){
            gzdbg<<"processing propeller nr: "<<idx<<" \n";

            if (_sdf_propeller->HasElement("jointName")) {
                propellers[idx].propJoint =  model_->GetJoint(_sdf_propeller->GetElement("jointName")->Get<std::string>());

                if (propellers[idx].propJoint == NULL)
                    gzthrow("[gazebo_motor_model] Couldn't find specified joint " << _sdf_propeller->GetElement("jointName")->Get<std::string>() << "\".");

            } else {
                gzerr << "[gazebo_motor_model] Please specify propeller joinName.\n";
            }

            if (_sdf_propeller->HasElement("linkNameParent")) {
                propellers[idx].parentLink =  model_->GetLink(_sdf_propeller->GetElement("linkNameParent")->Get<std::string>());

                if (propellers[idx].propJoint == NULL)
                    gzthrow("[gazebo_motor_model] Couldn't find specified parent link " << _sdf_propeller->GetElement("jointName")->Get<std::string>() << "\".");

            } else {
                gzerr << "[gazebo_motor_model] Please specify propeller joinName.\n";
            }

            if (_sdf_propeller->HasElement("cp"))
                propellers[idx].P_cp = _sdf_propeller->Get<v3d>("cp");
            else
                gzwarn<<"_sdf_propeller ["<<idx<<"] is missing 'cp' element \n";

            if (_sdf_propeller->HasElement("axisPosRot"))
                propellers[idx].P_joint = _sdf_propeller->Get<v3d>("axisPosRot");
            else
                gzwarn<<"_sdf_propeller ["<<idx<<"] is missing 'axisPosRot' element \n";

/*
            propellers[idx].propLink = propellers[idx].propJoint->GetChild();   // propeller link
            //propellers[idx].parentLink = propellers[idx].propJoint->GetParent();   // propeller link

#if GAZEBO_MAJOR_VERSION >= 9
            ignition::math::Pose3d pose = propellers[idx].propLink->WorldPose();
            ignition::math::Pose3d pose_parent = propellers[idx].parentLink->WorldPose();
#else
            ignition::math::Pose3d pose = ignitionFromGazeboMath(propellers[idx].propLink->GetWorldPose());
            ignition::math::Pose3d pose_parent = ignitionFromGazeboMath(propellers[idx].parentLink->GetWorldPose());
#endif

            // parent frame relative position of propeller cp
            propellers[idx].P_joint = pose_parent.Rot().RotateVectorReverse(propellers[idx].propJoint->GlobalAxis(0));
            propellers[idx].P_cp = pose_parent.Rot().RotateVectorReverse(pose.Pos() + pose.Rot().RotateVector(propellers[idx].cp)-pose_parent.Pos());
*/

            if (_sdf_propeller->HasElement("turnDir")) {
                std::string turning_direction = _sdf_propeller->GetElement("turnDir")->Get<std::string>();
                if (turning_direction == "cw")
                    propellers[idx].turning_direction_ = -1; // -1
                else if (turning_direction == "ccw")
                    propellers[idx].turning_direction_ = 1;  // 1
                else
                    gzerr << "[gazebo_motor_model] Please only use 'cw' or 'ccw' as turningDirection.\n";

            } else {
                gzerr << "[gazebo_motor_model] Please specify a turning direction ('cw' or 'ccw').\n";
            }

            if (_sdf_propeller->HasElement("speedRefTopic")) {
                propellers[idx].omega_ref_sub_topic = "~/" + model_->GetName() + _sdf_propeller->GetElement("speedRefTopic")->Get<std::string>();
                gzdbg<<"speedRefTopic "<<propellers[idx].omega_ref_sub_topic<<"\n";
            } else {
                gzwarn<<"no speedRefTopic specified.\n";
            }

            if (_sdf_propeller->HasElement("propParamsYAML")) {
                std::string prop_params_yaml = _sdf_propeller->GetElement("propParamsYAML")->Get<std::string>();
                propellers[idx].prop_params_.LoadPropParamsYAML(prop_params_yaml);

            } else {
                gzwarn<<"propeller ["<<idx<<"] is missing propeller parameters YAML file, "
                     <<"using default parameters.\n";
            }

            // calculate inertia tensor (assuming flat disk)
            propellers[idx].inertia = propellers[idx].prop_params_.mass *
                                      pow(propellers[idx].prop_params_.diameter/2,2) *
                                      m3d(0.25,0,0,0,0.25,0,0,0,0.5);

            if (_sdf_propeller->HasElement("slpstrTopic")) {
                propellers[idx].prop_slpstr_pub_topic_ = _sdf_propeller->GetElement("slpstrTopic")->Get<std::string>();
            } else {
                gzwarn<<"no slisptream topic specified.\n";
            }

            if (_sdf_propeller->HasElement("visTopic")) {
                propellers[idx].vector_vis_array_topic = _sdf_propeller->GetElement("visTopic")->Get<std::string>()+std::to_string(idx);
            } else {
                gzwarn<<"no slisptream topic specified.\n";
            }

            propellers[idx].vector_vis_array_msg.mutable_header()->mutable_stamp()->set_sec(0.0);
            propellers[idx].vector_vis_array_msg.mutable_header()->mutable_stamp()->set_nsec(0.0);
            propellers[idx].vector_vis_array_msg.mutable_header()->set_frame_id(propellers[idx].parentLink->GetName());

            for(int idx_vis=0; idx_vis<propellers[idx].vec_vis.size(); idx_vis++){
                propellers[idx].vec_vis[idx_vis] = propellers[idx].vector_vis_array_msg.add_vector();
                propellers[idx].vec_vis[idx_vis]->set_ns(namespace_+std::to_string(idx));
                propellers[idx].vec_vis[idx_vis]->set_id(idx_vis);
                propellers[idx].vec_vis[idx_vis]->mutable_scale()->set_x(0.025);
                propellers[idx].vec_vis[idx_vis]->mutable_scale()->set_y(0.05);
                propellers[idx].vec_vis[idx_vis]->mutable_scale()->set_z(0.05);
                propellers[idx].vec_vis[idx_vis]->mutable_color()->set_x(1.0);
                propellers[idx].vec_vis[idx_vis]->mutable_color()->set_y(0.0);
                propellers[idx].vec_vis[idx_vis]->mutable_color()->set_z(0.0);
            }

            _sdf_propeller = _sdf_propeller->GetNextElement("prop");
        }

    } else {
        gzerr << "[gazebo_propulsion_model] Please specify propellers\n";
    }

    getSdfParam<double>(_sdf, "rho_air", rho_air, rho_air);

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboPropulsion::OnUpdate, this, _1));

    gzdbg<<"load completed\n";

}

// This gets called by the world update start event.
void GazeboPropulsion::OnUpdate(const common::UpdateInfo& _info) {

    sampling_time_ = world_->SimTime().Double() - prev_sim_time_;
    prev_sim_time_ = world_->SimTime().Double();

    if (!pubs_and_subs_created) {

        gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
                node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
                    "~/" + kConnectGazeboToRosSubtopic, 1);
        gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;
        gzdbg<<"advertised ~/" + kConnectGazeboToRosSubtopic + "\n";

        for (int idx = 0; idx<n_props; idx++) {
            propellers[idx].prop_slpstr_pub_ = node_handle_->Advertise<gz_mav_msgs::PropulsionSlipstream>
                    ("~/" + model_->GetName() + "/" + propellers[idx].prop_slpstr_pub_topic_, 1);
            propellers[idx].vector_vis_array_pub = node_handle_->Advertise<gz_visualization_msgs::VisVectorArray>
                    ("~/" + propellers[idx].vector_vis_array_topic, 1);

            connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + propellers[idx].vector_vis_array_topic);
            connect_gazebo_to_ros_topic_msg.set_ros_topic(propellers[idx].vector_vis_array_topic);
            connect_gazebo_to_ros_topic_msg.set_msgtype(gz_std_msgs::ConnectGazeboToRosTopic::VIS_VECTOR_ARRAY);
            connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg, true);

            propellers[idx].omega_ref_sub_ = node_handle_->Subscribe(propellers[idx].omega_ref_sub_topic,
                                                                     &GazeboPropulsion::propeller::PropSpeedCallback,
                                                                     &propellers[idx]);
        }

        pubs_and_subs_created = true;
    }

    for (int idx = 0; idx<n_props; idx++) {

        propellers[idx].dt = sampling_time_;

        // pose of propeller frame
#if GAZEBO_MAJOR_VERSION >= 9
        //ignition::math::Pose3d pose = propellers[idx].propLink->WorldPose();
        ignition::math::Pose3d pose_parent = propellers[idx].parentLink->WorldPose();
#else
        //ignition::math::Pose3d pose = ignitionFromGazeboMath(propellers[idx].propLink->GetWorldPose());
        ignition::math::Pose3d pose_parent = ignitionFromGazeboMath(propellers[idx].parentLink->GetWorldPose());
#endif

        // Moments due to change in angular momentum of propeller, expressed in Hub-frame (parent-link frame)
        // Fast spinning link is problematic due to aliasing effects in the physics engine

        v3d H_Omega = propellers[idx].parentLink->RelativeAngularVel();
        v3d H_Omega_dot = propellers[idx].parentLink->RelativeAngularAccel();
        v3d H_moment_inertial = propellers[idx].inertia*H_Omega_dot + H_Omega.Cross(propellers[idx].inertia*H_Omega) +
                                propellers[idx].inertia*propellers[idx].P_joint*propellers[idx].omega_dot +
                                H_Omega.Cross(propellers[idx].inertia*propellers[idx].P_joint*propellers[idx].omega);

        H_moment_inertial.Correct();
        propellers[idx].parentLink->AddRelativeTorque(-H_moment_inertial);


        // velocity of propeller hub
#if GAZEBO_MAJOR_VERSION >= 9
        v3d body_velocity = propellers[idx].parentLink->WorldLinearVel(propellers[idx].P_cp);
#else
        v3d body_velocity = ignitionFromGazeboMath(propellers[idx].parentLink->GetWorldLinearVel(propellers[idx].P_cp));
#endif

        // propeller cp position in world frame
        //v3d W_cp = pose.Pos() + pose.Rot().RotateVector(propellers[idx].cp);
        /*
#if GAZEBO_MAJOR_VERSION >= 9
        v3d joint_axis = propellers[idx].propJoint->GlobalAxis(0);
#else
        v3d joint_axis = ignitionFromGazeboMath(propellers[idx].propJoint->GetGlobalAxis(0));
#endif
*/

        // propeller axis expressed in world frame, in positive rot direction
        v3d joint_axis = pose_parent.Rot().RotateVector(propellers[idx].P_joint);
        joint_axis.Normalize();

        double rps = propellers[idx].omega/(2*M_PI);
        //gzdbg<<"rps: "<<rps<<"\n";

        //double rps = (propellers[idx].omega + joint_axis.Dot(propellers[idx].parentLink->WorldAngularVel()))/(2*M_PI);
        //includes body rotation along prop axis - just to be unnecessarily precise ;)

        /*
        if (rps / (2 * M_PI) > 1 / (2 * sampling_time_)) {
           gzerr << "Aliasing on motor might occur. Consider making smaller simulation time steps or raising the rotor_velocity_slowdown_sim_ param.\n";
        }
        */

        v3d forward = joint_axis*propellers[idx].turning_direction_*(rps>=0 ? 1.0 : -1.0);   // positive thrust direction for symmetric propellers, we want enable two-way operation

        // resolve local airspeed into axial (V_inf_a) and radial (V_inf_r) component
        v3d body_velocity_radial = body_velocity - body_velocity.Dot(forward) * forward;
        v3d body_velocity_axial = body_velocity.Dot(forward) * forward;
        v3d rotor_pos = propellers[idx].parentLink->WorldPose().Pos()+
                                             pose_parent.Rot().RotateVector(propellers[idx].P_cp);  // link position expressed in world frame

        double V_inf_a = body_velocity.Dot(forward);    // axial component of relative flow wrt propeller disk (no wind assumed)
        double V_inf_a_clmpd = std::max(V_inf_a,0.0);   // treat reverse flow as static case
        double V_inf_r = body_velocity_radial.Length(); // radial component of relative flow wrt propeller disk (no wind assumed)

        double J   = V_inf_a_clmpd/(std::max(std::abs(rps),0.1) * propellers[idx].prop_params_.diameter);    // Advance ratio (set minimum rps to prevent division by zero...)

        // ---- Thrust ----

        //double C_T = propellers[idx].prop_params_.k_T*J + propellers[idx].prop_params_.k_T0;
        //double thrust = rho_air*pow(rps,2)*pow(propellers[idx].prop_params_.diameter,4)*C_T;
        double thrust = rho_air*std::abs(rps)*pow(propellers[idx].prop_params_.diameter,3)*
                        (propellers[idx].prop_params_.k_T*V_inf_a_clmpd +
                         std::abs(rps)*propellers[idx].prop_params_.diameter*propellers[idx].prop_params_.k_T0);

        //propellers[idx].propLink->AddForceAtRelativePosition(thrust*forward, propellers[idx].cp);
        propellers[idx].parentLink->AddForceAtRelativePosition(thrust*forward, propellers[idx].P_cp);

        // ---- Normal force ----

        v3d hub_force_ = -std::abs(rps*2*M_PI) * propellers[idx].prop_params_.rotor_drag_coefficient_ * body_velocity_radial;
        propellers[idx].parentLink->AddForceAtRelativePosition(hub_force_, propellers[idx].P_cp);

        // ---- Propeller slipstream ----

        thrust = std::max(thrust,0.0);  //Currently, breaking (neg. thrust can result from above calc) is ignored for wake modelling since this flight regime is assumed to be entered rarely and handlig breaking would require a separate case (i.e. jf>=1, k_w would need to be set to a value smaller than one -> in breaking state, the streamlines are diverging opposite to the direction of travel, hence the induced velocity behind the prop decreases monotonically (in contrast to the non-braking case, where the induced velocity first increases behind the prop before exhibiting loss-driven decay further downstream).
        double disk_area = pow(propellers[idx].prop_params_.diameter,2)/4*M_PI;
        double w = 0.5*(-V_inf_a_clmpd + sqrt(pow(V_inf_a_clmpd,2)+2*thrust/(rho_air*disk_area)));  // induced velocity at disk, w>=0
        double V_disk_a = V_inf_a+w;    //airflow velocity at propeller disk, w>=0

        v3d w_ds_a_;   // axial induced velocity
        v3d w_ds_r_;   // radial induced velocity
        v3d w_dir(0.0,0.0,0.0);    // propeller hub -> wake end
        v3d w_end(0.0,0.0,0.0);    // induced velocity at end of wake
        v3d w_disk(0.0,0.0,0.0);   // induced velocity at disk

        double k_w;
        double m_dot_clmpd = disk_area*(V_inf_a_clmpd + w)*rho_air; //>=0

        if (V_disk_a>0) {
            //wake extends beyond prop...

            double jf = V_inf_a/V_disk_a;     // jet flow parameter (m in selig paper)

            if (jf<=0)                        // hover & descent(V_inf_a<=0)
                k_w = 1.0;
            else if (jf>=0.75)                // full cruise
                k_w = 1.8;
            else
                k_w = 1.0 + jf/0.75*0.8;

            // induced velocity at disk in axial direction, accounting for losses (k_w)
            w_ds_a_ = -k_w*w*forward;

            // induced velocity at disk in radial direction
            //w_ds_r_ = -hub_force_/hub_force_.Length() * ignition::math::clamp(hub_force_.Length()/m_dot_clmpd*pow(std::abs(rps*2*M_PI)/max_rot_velocity_,0.25), 0.0, 0.8*body_velocity_radial.Length());
            w_ds_r_ = v3d(0,0,0);
            //double hub_vel_rel = w_ds_r_.Length()/V_inf_r; // debug: so far no check if hub_vel_rel < 1 ... should it be <1?

            w_disk = w_ds_a_+w_ds_r_; // downstream values already at disk for simplification

            // get wake
            if (V_inf_a >= 0) {
                // case I: No reverse free-stream, declare wake as terminated if induced velocity decayed to 1/8 of its initial value
                double t_end = log(8.0)/propellers[idx].prop_params_.d_flow;
                w_dir = -t_end*body_velocity + (7.0/8.0)*(w_ds_a_+w_ds_r_)/propellers[idx].prop_params_.d_flow;
                w_end = (w_ds_a_+w_ds_r_)/8.0;

            } else {
                // case II: Flow in wake changes direction (relative to propeller) after some time due to reverse free-stream
                double t_end = log(-w_ds_a_.Length()/V_inf_a)/propellers[idx].prop_params_.d_flow;
                w_dir = -t_end*body_velocity + (1.0-exp(-propellers[idx].prop_params_.d_flow*t_end))*(w_ds_a_+w_ds_r_)/propellers[idx].prop_params_.d_flow;
                w_end = (w_ds_a_+w_ds_r_)*exp(-propellers[idx].prop_params_.d_flow*t_end);
            }

        } else {
            // case III: wake does not extend beyond prop, reverse flow through prop disk, no wake modelled
            w_dir = v3d(0.0,0.0,0.0);
            w_end = v3d(0.0,0.0,0.0);
        }

        // ---- Drag torque ----
        double drag_torque = -rho_air*std::abs(rps)*pow(propellers[idx].prop_params_.diameter,4)*
                             (V_inf_a_clmpd*propellers[idx].prop_params_.k_Q +
                              std::abs(rps)*propellers[idx].prop_params_.diameter*propellers[idx].prop_params_.k_Q0)*
                             (rps>=0 ? 1.0 : -1.0);
        v3d drag_torque_ = joint_axis * drag_torque;
        propellers[idx].parentLink->AddTorque(drag_torque_);

        //Want to apply torques to parent link, not to propeller link (otherwise possibly problems when using _joint->SetVelocity)
        //physics::Link_V parent_links = link_->GetParentJointsLinks();
        //parent_links.at(0)->AddTorque(drag_torque_);

        // ---- Rolling moment ----
        v3d rolling_moment = -propellers[idx].turning_direction_*std::abs(rps*2*M_PI) *
                                                   propellers[idx].prop_params_.rolling_moment_coefficient_ *
                                                   body_velocity_radial;
        propellers[idx].parentLink->AddTorque(rolling_moment);

        // ---- Fill propeller slipstream message ----
        {
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
            propulsion_slipstream_msg_.set_prop_diam(propellers[idx].prop_params_.diameter);

            propellers[idx].prop_slpstr_pub_->Publish(propulsion_slipstream_msg_);
        }

        float r,g,b;
        v3d P_start = propellers[idx].P_cp;//v3d(-0.025,0,0.15); //
        v3d P_vec;

        // Test joint issues
        ignition::math::Pose3d err_joint = propellers[idx].propJoint->AnchorErrorPose();
        //v3d err_axis;
        //double err_angle;
        //err_joint.Rot().ToAxis(err_axis,err_angle);
        //if(err_angle*err_axis.Dot>0.03)
            //gzdbg<<"err_angle: "<<err_angle<<"\n";

        for(int idx_vis=0; idx_vis<propellers[idx].vec_vis.size(); idx_vis++){

            switch (idx_vis) {
            case 0:
                r=1;g=0;b=0;
                P_vec = pose_parent.Rot().RotateVectorReverse(thrust*forward/10);
                break;
            case 1:
                r=0;g=1;b=0;
                P_vec = pose_parent.Rot().RotateVectorReverse(hub_force_/10);
                break;
            case 2:
                r=0;g=0;b=1;
                P_vec = pose_parent.Rot().RotateVectorReverse(drag_torque_);
                break;
            case 3:
                r=1;g=1;b=0;
                P_vec = pose_parent.Rot().RotateVectorReverse(rolling_moment);
                break;
            case 4:
                r=1;g=0;b=1;
                P_vec = pose_parent.Rot().RotateVectorReverse(w_dir);
                break;
            case 5:
                r=0;g=1;b=1;
                P_vec = pose_parent.Rot().RotateVectorReverse(w_disk);
                break;
            case 6:
                r=1;g=1;b=1;
                P_vec = H_moment_inertial;
                break;
            }

            propellers[idx].vec_vis[idx_vis]->mutable_color()->set_x(r);
            propellers[idx].vec_vis[idx_vis]->mutable_color()->set_y(g);
            propellers[idx].vec_vis[idx_vis]->mutable_color()->set_z(b);
            propellers[idx].vec_vis[idx_vis]->mutable_startpoint()->set_x(P_start.X());
            propellers[idx].vec_vis[idx_vis]->mutable_startpoint()->set_y(P_start.Y());
            propellers[idx].vec_vis[idx_vis]->mutable_startpoint()->set_z(P_start.Z());
            propellers[idx].vec_vis[idx_vis]->mutable_vector()->set_x(P_vec.X());
            propellers[idx].vec_vis[idx_vis]->mutable_vector()->set_y(P_vec.Y());
            propellers[idx].vec_vis[idx_vis]->mutable_vector()->set_z(P_vec.Z());
        }
        propellers[idx].vector_vis_array_pub->Publish(propellers[idx].vector_vis_array_msg);

    }

    updateCounter++;

}

GZ_REGISTER_MODEL_PLUGIN(GazeboPropulsion);
}

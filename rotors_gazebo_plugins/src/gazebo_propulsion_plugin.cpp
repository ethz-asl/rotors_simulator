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
  std::cout<<"motor_model destructed"<<std::endl;  
}

void GazeboPropulsion::InitializeParams() {}

void GazeboPropulsion::Publish() {
    
  //turning_velocity_msg_.set_data(joint_->GetVelocity(0));
  //motor_velocity_pub_->Publish(turning_velocity_msg_);
  propulsion_slipstream_pub_->Publish(propulsion_slipstream_msg_);  // propulsion_slipstream_msg_ is filled in updateForcesAndMoments function
    
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
  node_handle_->Init(namespace_);

  if (_sdf->HasElement("jointName"))
    joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a jointName, where the rotor is attached.\n";
  
  // Get the pointer to the joint.
  joint_ = model_->GetJoint(joint_name_);
  if (joint_ == NULL)
    gzthrow("[gazebo_motor_model] Couldn't find specified joint \"" << joint_name_ << "\".");
    
  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a linkName of the rotor.\n";
  link_ = model_->GetLink(link_name_);
    
  if (link_ == NULL)
    gzthrow("[gazebo_motor_model] Couldn't find specified link \"" << link_name_ << "\".");

  if (_sdf->HasElement("motorNumber"))
    motor_number_ = _sdf->GetElement("motorNumber")->Get<int>();
  else
    gzerr << "[gazebo_motor_model] Please specify a motorNumber.\n";

  if (_sdf->HasElement("turningDirection")) {
    std::string turning_direction = _sdf->GetElement("turningDirection")->Get<std::string>();
    if (turning_direction == "cw")
      turning_direction_ = turning_direction::CW;   // -1
    else if (turning_direction == "ccw")
      turning_direction_ = turning_direction::CCW;  // 1
    else
      gzerr << "[gazebo_motor_model] Please only use 'cw' or 'ccw' as turningDirection.\n";
  }
  else
    gzerr << "[gazebo_motor_model] Please specify a turning direction ('cw' or 'ccw').\n";

  if (_sdf->HasElement("diameter")){
        diameter = _sdf->GetElement("diameter")->Get<double>();
        disk_area = M_PI/4.0*diameter*diameter;
        propulsion_slipstream_msg_.set_prop_diam(diameter);
  }else{
    gzerr << "[gazebo_motor_model] Please specify a propeller diameter.\n";
  }
    
  getSdfParam<double>(_sdf, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_, 10);

  getSdfParam<double>(_sdf, "rho_air", rho_air, rho_air);
  getSdfParam<std::string>(_sdf, "escRotVelCurve", esc_, esc_);
  if(!(esc_=="lin"||esc_=="sqrt"))
    gzerr<<"[gazebo_motor_model]  specify escRotVelCurve as either 'lin' or 'sqrt'\n";

  getSdfParam<double>(_sdf, "maxRotVelocity", max_rot_velocity_, max_rot_velocity_);
  getSdfParam<double>(_sdf, "timeConstantUp", time_constant_up_, time_constant_up_);
  getSdfParam<double>(_sdf, "timeConstantDown", time_constant_down_, time_constant_down_);

  getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic_, command_sub_topic_);
  //getSdfParam<std::string>(_sdf, "motorSpeedPubTopic", motor_speed_pub_topic_, motor_speed_pub_topic_);
  getSdfParam<std::string>(_sdf, "propSlipstreamPubTopic", propulsion_slipstream_pub_topic_, propulsion_slipstream_pub_topic_);

  getSdfParam<double>(_sdf, "rotorDragCoefficient", rotor_drag_coefficient_, rotor_drag_coefficient_);
  getSdfParam<double>(_sdf, "rollingMomentCoefficient", rolling_moment_coefficient_, rolling_moment_coefficient_);
  getSdfParam<double>(_sdf, "k_T", k_T, k_T);
  getSdfParam<double>(_sdf, "k_T0", k_T0, k_T0);
  getSdfParam<double>(_sdf, "k_Q", k_Q, k_Q);
  getSdfParam<double>(_sdf, "k_Q0", k_Q0, k_Q0);

  getSdfParam<double>(_sdf, "d_flow", d_flow, d_flow);

  if(_sdf->HasElement("log_itv"))
    this->logItv = _sdf->Get<int>("log_itv");

  if(_sdf->HasElement("log_name")){
    this->logName = _sdf->Get<std::string>("log_name");
    this->logEnable = true;
  }
    
  // Set the maximumForce on the joint. This is deprecated from V5 on, and the joint won't move.
#if GAZEBO_MAJOR_VERSION < 5
  joint_->SetMaxForce(0, max_force_);
#endif
    
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboPropulsion::OnUpdate, this, _1));

  // PUB/SUB
  // default: command_sub_topic_ = kDefaultCommandSubTopic = "/gazebo/command/motor_speed",     With std_vtol: "/gazebo/command/motor_speed"
  // default: motor_speed_pub_topic_ = kDefaultMotorVelocityPubTopic = "/motor_speed",          With std_vtol: "motor_speed/i"
  
  command_sub_ = node_handle_->Subscribe<gz_mav_msgs::CommandMotorSpeed>("~/" + model_->GetName() + command_sub_topic_, &GazeboPropulsion::VelocityCallback, this);
  motor_failure_sub_ = node_handle_->Subscribe<msgs::Int>(motor_failure_sub_topic_, &GazeboPropulsion::MotorFailureCallback, this);
  //motor_velocity_pub_ = node_handle_->Advertise<std_msgs::msgs::Float>("~/" + model_->GetName() + motor_speed_pub_topic_, 1);
  propulsion_slipstream_pub_ = node_handle_->Advertise<gz_mav_msgs::PropulsionSlipstream>("~/" + model_->GetName() + propulsion_slipstream_pub_topic_, 1);
  //do_log_sub_ = node_handle_->Subscribe<std_msgs::msgs::Int32>("~/" + model_->GetName() + do_log_sub_topic_, &GazeboPropulsion::DoLogCallback, this);
    
  // Create the first order filter.
  rotor_velocity_filter_.reset(new FirstOrderFilter<double>(time_constant_up_, time_constant_down_, ref_motor_rot_vel_));
}

// This gets called by the world update start event.
void GazeboPropulsion::OnUpdate(const common::UpdateInfo& _info) {
  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();
  UpdateForcesAndMoments();
  UpdateMotorFail();
  Publish();
}

void GazeboPropulsion::VelocityCallback(CommandMotorSpeedPtr &rot_velocities) {
  if(rot_velocities->motor_speed_size() < motor_number_) {
    std::cout  << "You tried to access index " << motor_number_
      << " of the MotorSpeed message array which is of size " << rot_velocities->motor_speed_size() << "." << std::endl;
  } else {
      
      //Motor ESC
      double pwm_n = 0.5*(static_cast<double>(rot_velocities->motor_speed(motor_number_))+1.0);   //[-1,1] to [0,1]
      if(esc_=="lin")
          ref_motor_rot_vel_ = pwm_n*static_cast<double>(max_rot_velocity_); // in rad/s
      else if(esc_=="sqrt")
          ref_motor_rot_vel_ = sqrt(pwm_n)*static_cast<double>(max_rot_velocity_);  // in rad/s
      else
          std::cout<<"esc pwm-to-rpm curve-form not defined"<<std::endl;
  }
}

void GazeboPropulsion::MotorFailureCallback(const boost::shared_ptr<const msgs::Int> &fail_msg) {
  motor_Failure_Number_ = fail_msg->data();
}

/*
void GazeboPropulsion::DoLogCallback(Int32Ptr& do_log){
    if (do_log->data() == 1 && logEnable) {
        logFlag = true; // start log
    
    } else {
        logFlag = false; // stop log
    }
}
*/

void GazeboPropulsion::UpdateForcesAndMoments() {
    

    // logging of interesting quantities for debugging purpose. Filepath hardcoded, improve
    // log is started/stopped via command in do_log_sub_topic_ (cf. DoLogCallback) if logEnable=true
    if (!logStarted && logFlag) {
        std::cout<<"starting log"<<std::endl;
        logStarted = true;
        this->logfile.open (std::string("/Users/davidrohr/Documents/ETH/Faecher/Master/Master_Thesis/Simulation/MATLAB/GazeboLogs/") + logName, std::ofstream::out | std::ofstream::app);
        if (this->headerFlag) {
            this->headerFlag = false;
            this->logfile << "t_sim, Pr_x, Pr_y, Pr_z, Qr_w, Qr_x, Qr_y, Qr_z, w_disk_x, w_disk_y, w_disk_z";
            this->logfile << ", w_end_x, w_end_y, w_end_z, w_dir_x, w_dir_y, w_dir_z";
            this->logfile << ", w_ds_a_x, w_ds_a_y, w_ds_a_z, w_ds_r_x, w_ds_r_y, w_ds_r_z";
            this->logfile << ", hubF_x, hubF_y, hubF_z, thrust, revps, revps_sp";
            this->logfile << ", vp_x, vp_y, vp_z, radps";
            this->logfile << ", vn_x, vn_y, vn_z";
            this->logfile << ", vinf_x, vinf_y, vinf_z";
            this->logfile << "\n";
        }
    }
    if (this->logfile.is_open() && !logFlag) {
        std::cout<<"stopping log"<<std::endl;
        this->logfile.close();
        this->logStarted = false;
    }

    
    motor_rot_vel_ = joint_->GetVelocity(0);
    if (motor_rot_vel_ / (2 * M_PI) > 1 / (2 * sampling_time_)) {
        gzerr << "Aliasing on motor [" << motor_number_ << "] might occur. Consider making smaller simulation time steps or raising the rotor_velocity_slowdown_sim_ param.\n";
    }
    double real_motor_velocity = std::abs(motor_rot_vel_) * rotor_velocity_slowdown_sim_; // actual propeller speed [rad/s]
    double rps = real_motor_velocity/(2*M_PI);  //positive
    double rps_sp = ref_motor_rot_vel_/(2*M_PI); //positive
    
    // velocity of propeller hub
    #if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Vector3d body_velocity = link_->WorldLinearVel();
    #else
    ignition::math::Vector3d body_velocity = ignitionFromGazeboMath(link_->GetWorldLinearVel());
    #endif
    
    // propeller axle, points in positive thrust direction
    #if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Vector3d joint_axis = joint_->GlobalAxis(0);
    #else
    ignition::math::Vector3d joint_axis = ignitionFromGazeboMath(joint_->GetGlobalAxis(0));
    #endif
    joint_axis.Normalize();
    
    // resolve local airspeed into axial (V_inf_a) and radial (V_inf_r) component
    ignition::math::Vector3d body_velocity_radial = body_velocity - body_velocity.Dot(joint_axis) * joint_axis;
    ignition::math::Vector3d body_velocity_axial = body_velocity.Dot(joint_axis) * joint_axis;
    ignition::math::Vector3d rotor_pos = link_->WorldPose().Pos();  // link position expressed in world frame

    double V_inf_a = body_velocity_axial.Length() * sgn(body_velocity.Dot(joint_axis));  // axial component of relative flow wrt propeller disk (no wind assumed)
    double V_inf_a_clmpd = std::max(V_inf_a,0.0);                                        // treat reverse flow as static case
    double V_inf_r = body_velocity_radial.Length();                                      // radial component of relative flow wrt propeller disk (no wind assumed)
    
    double J   = V_inf_a_clmpd/(std::max(rps,1.0) * diameter);    // Advance ratio (set minimum rps to prevent division by zero...)

    // ---- Thrust ----
    
    double C_T = k_T*J + k_T0;
    double thrust = rho_air*pow(rps,2)*pow(diameter,4)*C_T;
    link_->AddRelativeForce(ignition::math::Vector3d(0, 0, thrust));

    /*
    // DEPRECATED - had a wrong understanding of advance ratio (David Rohr)
    // To Do: work with V_inf_a_clmpd or V_inf_a? -> V_inf_a_clmpd, w nonsense if v_inf_a<0
    // NAN if rps=0, V_inf_a = 0, cf matlab -> problem lied in calc of adv_ratio
    double C1 = rho_air*rps*pow(diameter,3)*k_T;
    double C2 = rho_air*pow(rps,2)*pow(diameter,4)*k_T0;
    double C3 = C1*V_inf_a_clmpd/2+C2;
    double C4 = 2*C3 + pow(C1,2)/(2*rho_air*disk_area);
    double C5 = pow(C3,2) - pow(C1,2)/4*pow(V_inf_a_clmpd,2);
    thrust = (C4 - sqrt(C4*C4-4*C5))/2;
    if(thrust-C3>0)
        gzerr<<"thrust calculation invalid\n";
    */


    // ---- Normal force ----
    
    ignition::math::Vector3d hub_force_ = -std::abs(real_motor_velocity) * rotor_drag_coefficient_ * body_velocity_radial;
    link_->AddForce(hub_force_);
    
    // ---- Propeller slipstream ----

    thrust = std::max(thrust,0.0);  //Currently, breaking (neg. thrust can result from above calc) is ignored for wake modelling since this flight regime is assumed to be entered rarely and handlig breaking would require a separate case (i.e. jf>=1, k_w would need to be set to a value smaller than one -> in breaking state, the streamlines are diverging opposite to the direction of travel, hence the induced velocity behind the prop decreases monotonically (in contrast to the non-braking case, where the induced velocity first increases behind the prop before exhibiting loss-driven decay further downstream).
    
    double w = 0.5*(-V_inf_a_clmpd + sqrt(pow(V_inf_a_clmpd,2)+2*thrust/(rho_air*disk_area)));  // induced velocity at disk, w>=0
    double V_disk_a = V_inf_a+w;    //airflow velocity at propeller disk, w>=0
    
    ignition::math::Vector3d w_ds_a_;   // axial induced velocity
    ignition::math::Vector3d w_ds_r_;   // radial induced velocity
    ignition::math::Vector3d w_dir(0.0,0.0,0.0);    // propeller hub -> wake end
    ignition::math::Vector3d w_end(0.0,0.0,0.0);    // induced velcoiyt at end of wake
    ignition::math::Vector3d w_disk(0.0,0.0,0.0);   // induced velcoiyt at disk
    
    double k_w;
    double m_dot_clmpd = disk_area*(V_inf_a_clmpd + w)*rho_air; //>=0
    //double hub_vel_rel = 0.0; // flow turning ratio, for debugging
    
    
    if (V_disk_a>0) {
        //wake extends beyond prop...
        
        double jf = V_inf_a/V_disk_a;             // jet flow parameter (m in selig paper)
        
        if (jf<=0)                        // hover & descent(V_inf_a<=0)
            k_w = 1.0;
        
        else if (jf>=0.75)                // full cruise
            k_w = 1.8;
        
        else
            k_w = 1.0 + jf/0.75*0.8;

        // induced velocity at disk in axial direction, accounting for losses (k_w)
        w_ds_a_ = -k_w*w*joint_axis;
        
        // induced velocity at disk in radial direction
        w_ds_r_ = -hub_force_/hub_force_.Length() * ignition::math::clamp(hub_force_.Length()/m_dot_clmpd*pow(real_motor_velocity/max_rot_velocity_,0.25), 0.0, 0.8*body_velocity_radial.Length());
        //hub_vel_rel = w_ds_r_.Length()/V_inf_r; // debug: so far no check if hub_vel_rel < 1 ... should it be <1?
        
        w_disk = w_ds_a_+w_ds_r_; // downstream values already at disk for simplification
        
        // get wake
        if (V_inf_a >= 0) {
            // case I: No reverse free-stream, declare wake as terminated if induced velocity decayed to 1/8 of its initial value
            double t_end = log(8.0)/d_flow;
            w_dir = -t_end*body_velocity + (7.0/8.0)*(w_ds_a_+w_ds_r_)/d_flow;
            w_end = (w_ds_a_+w_ds_r_)/8.0;
        
        } else {
            // case II: Flow in wake changes direction (relative to propeller) after some time due to reverse free-stream
            double t_end = log(-w_ds_a_.Length()/V_inf_a)/d_flow;
            w_dir = -t_end*body_velocity + (1.0-exp(-d_flow*t_end))*(w_ds_a_+w_ds_r_)/d_flow;
            w_end = (w_ds_a_+w_ds_r_)*exp(-d_flow*t_end);
        }
        
    } else {
        // case III: wake does not extend beyond prop, reverse flow through prop disk, no wake modelled
        w_dir = ignition::math::Vector3d(0.0,0.0,0.0);
        w_end = ignition::math::Vector3d(0.0,0.0,0.0);
    }
    
    
    // ---- Drag torque ----
    
    // Want to apply torques to parent link, not to propeller link (otherwise possibly problems when using _joint->SetVelocity)
    physics::Link_V parent_links = link_->GetParentJointsLinks();
    
    ignition::math::Vector3d drag_torque = -joint_axis * turning_direction_ * rho_air*pow(rps,2) * pow(diameter,5)*(k_Q0 + J*k_Q);
    parent_links.at(0)->AddTorque(drag_torque);


    /*
    ignition::math::Vector3d drag_torque(0, 0, -turning_direction_ * rho_air*pow(rps,2) * pow(diameter,5)*(k_Q0 + J*k_Q)); // implies rotational axis = link z_axis !!
    // The tansformation from the parent_link to the link_.
    #if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Pose3d pose_difference = link_->WorldCoGPose() - parent_links.at(0)->WorldCoGPose();
    #else
    ignition::math::Pose3d pose_difference = ignitionFromGazeboMath(link_->GetWorldCoGPose() - parent_links.at(0)->GetWorldCoGPose());
    #endif
    
    // Transforming the drag torque into the parent frame to handle arbitrary rotor orientations.
    ignition::math::Vector3d drag_torque_parent_frame = pose_difference.Rot().RotateVector(drag_torque);
    parent_links.at(0)->AddRelativeTorque(drag_torque_parent_frame);
    */
    
    // ---- Rolling moment ----
    ignition::math::Vector3d rolling_moment = -turning_direction_*std::abs(real_motor_velocity) * rolling_moment_coefficient_ * body_velocity_radial;
    parent_links.at(0)->AddTorque(rolling_moment);

    // ---- Fill propeller slipstream message ----
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
    propulsion_slipstream_msg_.set_prop_diam(diameter);

    // Apply the filter on the motor's velocity.
    double filt_motor_rot_vel = rotor_velocity_filter_->updateFilter(ref_motor_rot_vel_, sampling_time_);
    joint_->SetVelocity(0, turning_direction_ * filt_motor_rot_vel / rotor_velocity_slowdown_sim_);

    // Logging
    if(this->logfile.is_open() && this->updateCounter%this->logItv==0){
        
#if GAZEBO_MAJOR_VERSION >= 9
        ignition::math::Pose3d pose = this->link_->WorldPose();
#else
        ignition::math::Pose3d pose = ignitionFromGazeboMath(this->link_->GetWorldPose());
#endif
        this->logfile<<this->world_->SimTime().Double()<<", "<<rotor_pos.X()<<", "<<rotor_pos.Y()<<", "<<rotor_pos.Z();
        this->logfile<<", "<<pose.Rot().W()<<", "<<pose.Rot().X()<<", "<<pose.Rot().Y()<<", "<<pose.Rot().Z();
        this->logfile<<", "<<w_disk.X()<<", "<<w_disk.Y()<<", "<<w_disk.Z();
        this->logfile<<", "<< w_end.X()<<", "<< w_end.Y()<<", "<<w_end.Z();
        this->logfile<<", "<< w_dir.X()<<", "<< w_dir.Y()<<", "<<w_dir.Z();
        this->logfile<<", "<< w_ds_a_.X()<<", "<< w_ds_a_.Y()<<", "<<w_ds_a_.Z();
        this->logfile<<", "<< w_ds_r_.X()<<", "<< w_ds_r_.Y()<<", "<<w_ds_r_.Z();
        this->logfile<<", "<<hub_force_.X()<<", "<<hub_force_.Y()<<", "<<hub_force_.Z();
        this->logfile<<", "<<thrust<<", "<<rps<<", "<<rps_sp;
        this->logfile<<", "<<body_velocity_radial.X()<<", "<<body_velocity_radial.Y()<<", "<<body_velocity_radial.Z()<<", "<<real_motor_velocity;
        this->logfile<<", "<<body_velocity_axial.X()<<", "<<body_velocity_axial.Y()<<", "<<body_velocity_axial.Z();
        this->logfile<<", "<<body_velocity.X()<<", "<<body_velocity.Y()<<", "<<body_velocity.Z();
        this->logfile<< "\n";

        //std::cout<<"hub_vel_rel: "<<hub_vel_rel<<std::endl;
        std::cout<<"hub_force_: "<<hub_force_.Length()<<std::endl;
        std::cout<<"m_dot_clmpd: "<<m_dot_clmpd<<std::endl;
        std::cout<<"w_ds_r_: "<<w_ds_r_.Length()<<std::endl;
        std::cout<<"V_inf_r: "<<V_inf_r<<std::endl;
    }
    
    updateCounter++;
    
}

void GazeboPropulsion::UpdateMotorFail() {
  if (motor_number_ == motor_Failure_Number_ - 1){
    joint_->SetVelocity(0,0);
    if (screen_msg_flag){
      std::cout << "Motor number [" << motor_Failure_Number_ <<"] failed!  [Motor thrust = 0]" << std::endl;
      tmp_motor_num = motor_Failure_Number_;

      screen_msg_flag = 0;
    }
  }else if (motor_Failure_Number_ == 0 && motor_number_ ==  tmp_motor_num - 1){
     if (!screen_msg_flag){
       std::cout << "Motor number [" << tmp_motor_num <<"] running! [Motor thrust = (default)]" << std::endl;
       screen_msg_flag = 1;
     }
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboPropulsion);
}

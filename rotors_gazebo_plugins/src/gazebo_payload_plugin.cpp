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

#include "rotors_gazebo_plugins/gazebo_payload_plugin.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(GazeboPayloadPlugin)

/////////////////////////////////////////////////
GazeboPayloadPlugin::GazeboPayloadPlugin()
{
    gzdbg<<"GazeboPayloadPlugin constructed\n";
}

/////////////////////////////////////////////////
GazeboPayloadPlugin::~GazeboPayloadPlugin()
{
    gzdbg<<"GazeboPayloadPlugin destructed\n";
}

/////////////////////////////////////////////////
void GazeboPayloadPlugin::Load(physics::ModelPtr _model,
                              sdf::ElementPtr _sdf)
{
    gzdbg<<"load called"<<std::endl;

    GZ_ASSERT(_model, "GazeboPayloadPlugin _model pointer is NULL");
    GZ_ASSERT(_sdf, "GazeboPayloadPlugin _sdf pointer is NULL");

    this->model_ = _model;
    this->world_ = this->model_->GetWorld();
    hook_ctrl_.world = this->model_->GetWorld();
    GZ_ASSERT(this->world_, "GazeboPayloadPlugin world pointer is NULL");

    namespace_.clear();

    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init();

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboPayloadPlugin::OnUpdate, this));


    if (_sdf->HasElement("payload")) {
        std::string payload = _sdf->Get<std::string>("payload");
        this->payload_ = this->model_->GetLink(payload);
    }

    if (_sdf->HasElement("parent")) {
        std::string parent = _sdf->Get<std::string>("parent");
        this->parent_ = this->model_->GetLink(parent);
    }

    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzerr<<"Please specify a robotNamespace.\n";

    if (_sdf->HasElement("hoistPosParent"))
        hoist_pos_parent_ = _sdf->Get<ignition::math::Vector3d>("hoistPosParent");
    else
        hoist_pos_parent_ = ignition::math::Vector3d(0,0,0);

    if (_sdf->HasElement("hoistPosPayload"))
        hoist_pos_payload_ = _sdf->Get<ignition::math::Vector3d>("hoistPosPayload");
    else
        hoist_pos_payload_ = ignition::math::Vector3d(0,0,0);

    if (_sdf->HasElement("pickupWorldPosPayload"))
        pickup_world_pos_payload_ = _sdf->Get<ignition::math::Vector3d>("pickupWorldPosPayload");
    else
        pickup_world_pos_payload_ = ignition::math::Vector3d(0,0,0);

    if (_sdf->HasElement("teleportBackToPickup"))
        teleport_back_to_pickup_ = _sdf->Get<bool>("teleportBackToPickup");

    if (_sdf->HasElement("teleportBackToAircraft"))
        teleport_back_to_aircraft_ = _sdf->Get<bool>("teleportBackToAircraft");

    int state_ini = RESTING;
    if (_sdf->HasElement("initialState")) {
        state_ini = _sdf->Get<int>("initialState");
        if (state_ini!=RESTING && state_ini!=ATTACHED && state_ini!=FALLING && state_ini!=TELEPORTING)
          state_ini = RESTING;
    }

    state_ = static_cast<State>(state_ini);

    if (_sdf->HasElement("dropTopic")) {
        drop_topic_ = _sdf->Get<std::string>("dropTopic");
    }

    if (_sdf->HasElement("payloadTrackingTopic")) {
        tracking_pos_pub_topic_ = _sdf->Get<std::string>("payloadTrackingTopic");
    }

    if (_sdf->HasElement("omega")) {
        omega_ = _sdf->Get<float>("omega");
    }

    if (_sdf->HasElement("posOnly")) {
        pos_only_ = _sdf->Get<bool>("posOnly");
    }

    if (_sdf->HasElement("proxRec")) {
        proximity_recovery_ = _sdf->Get<bool>("proxRec");
    }

    if (_sdf->HasElement("radiusRec")) {
        recovery_radius_ = _sdf->Get<float>("radiusRec");
    }

    if (_sdf->HasElement("hookStab")) {

        do_hook_ctrl = true;
        sdf::ElementPtr _sdf_hookStab= _sdf->GetElement("hookStab");

        if (_sdf_hookStab->HasElement("hookJoint")) {
            hook_ctrl_.hook_joint_name = _sdf_hookStab->Get<std::string>("hookJoint");
            hook_ctrl_.hook_joint = model_->GetJoint(hook_ctrl_.hook_joint_name);

            if (hook_ctrl_.hook_joint == nullptr) {
                gzwarn << "joint " << hook_ctrl_.hook_joint_name << " not found.\n";
            }

        } else {
            gzwarn<<"Missing hookJoint element\n";
        }

        if (_sdf_hookStab->HasElement("hookElevJoint")) {
            hook_ctrl_.hook_elev_joint_name = _sdf_hookStab->Get<std::string>("hookElevJoint");
            hook_ctrl_.hook_elev_joint = model_->GetJoint(hook_ctrl_.hook_elev_joint_name);

            if (hook_ctrl_.hook_elev_joint == nullptr) {
                gzwarn << "joint " << hook_ctrl_.hook_elev_joint_name << " not found.\n";
            }

        } else {
            gzwarn<<"Missing hookElevJoint element\n";
        }

        if (_sdf_hookStab->HasElement("hookRefTopic")) {
            hook_ctrl_.hook_ref_topic = _sdf_hookStab->Get<std::string>("hookRefTopic");

        } else {
            gzwarn<<"Missing hookRefTopic\n";
        }

        if (_sdf_hookStab->HasElement("hookNoLoadPID")) {
            V3D gains = _sdf_hookStab->Get<V3D>("hookNoLoadPID");
            hook_ctrl_.pid_no_load.Init(gains.X(),gains.Y(),gains.Z(),0.3,-1,0.3,-1);

        } else {
            gzwarn<<"Missing hookNoLoadPID, using default values\n";
        }

        if (_sdf_hookStab->HasElement("hookLoadPID")) {
            V3D gains = _sdf_hookStab->Get<V3D>("hookLoadPID");
            hook_ctrl_.pid_w_load.Init(gains.X(),gains.Y(),gains.Z(),0,0,0.3,-1);
        } else {
            gzwarn<<"Missing hookLoadPID, using default values\n";
        }

        if (_sdf_hookStab->HasElement("lidarTopic")) {
            hook_ctrl_.lidar_topic = _sdf_hookStab->Get<std::string>("lidarTopic");
        } else {
            gzwarn<<"Missing lidarTopic\n";
        }

    }
}

/////////////////////////////////////////////////
void GazeboPayloadPlugin::CreatePubsAndSubs()
{
    drop_sub_ = node_handle_->Subscribe("~/" + namespace_ + "/" + drop_topic_, &GazeboPayloadPlugin::DropCallback, this);

    if (do_hook_ctrl) {
        hook_ctrl_.hook_ref_sub = node_handle_->Subscribe("~/" + namespace_ + "/" + hook_ctrl_.hook_ref_topic,
                                                          &GazeboPayloadPlugin::HookControl::RefCallback, &hook_ctrl_);
        hook_ctrl_.lidar_sub = node_handle_->Subscribe(namespace_ + "/" + hook_ctrl_.lidar_topic,
                                                          &GazeboPayloadPlugin::HookControl::LidarCallback, &hook_ctrl_);
    }

    tracking_pos_pub_ = node_handle_->Advertise<gazebo::msgs::Vector3d>(namespace_ + "/" + tracking_pos_pub_topic_, 1);
}

/////////////////////////////////////////////////
void GazeboPayloadPlugin::DropCallback(Float32Ptr &drop_msg)
{
    if (drop_msg->data() > 0.0) {
        trigg_ = true;
        gzdbg<<"trigg\n";

    } else {
        trigg_ = false;
    }
}

void GazeboPayloadPlugin::HookControl::DoControl(){

    common::Time now = world->SimTime();
    if (!init) {
        init = true;
        last_time = now;
    }
    double dt = (now - last_time).Double();
    last_time = now;

    float u;
    float h, h_min, h_max;

    std::unique_lock<std::mutex> lock1(ref_lock);
    u = hook_ref;
    lock1.unlock();

    std::unique_lock<std::mutex> lock2(lidar_lock);
    h = lidar_dist;
    h_min = lidar_min_dist;
    h_max = lidar_max_dist;
    lock2.unlock();

    double err = u - hook_joint->Position(0);
    double force = pid_no_load.Update(-err, dt);

    if (isnan(force)){
        force = 0;
        pid_no_load.Reset();
    }

    //force = pid_no_load.GetPGain()*err - pid_no_load.GetDGain()*hook_joint->GetVelocity(0);

    double elev_ref = force;

    /*
    gzdbg<<"ref: "<<u<<"\n";
    gzdbg<<"err: "<<err<<"\n";
    gzdbg<<"dt: "<<dt<<"\n";
    gzdbg<<"pid_out: "<<force<<"\n";
    gzdbg<<"\n";
    */

    // Servo-like actuation of hook-elevator
    double elev_pos = hook_elev_joint->Position(0);
    double elev_rate = hook_elev_joint->GetVelocity(0);

    if (!srv.init) {
        srv.last_srv_time = now;
        srv.init = true;
    }

    double d_ref = srv.slew * dt;

    // implementation of slew-rate constraint
    double rate_ref;
    if (elev_ref>srv.ref+d_ref) {
        rate_ref = srv.slew;
        srv.ref+=d_ref;
    } else if (elev_ref<srv.ref-d_ref) {
        rate_ref = -srv.slew;
        srv.ref -= d_ref;
    } else {
        rate_ref = (elev_ref-srv.ref)/dt;
        srv.ref = elev_ref;
    }

    // sanity
    if (isnan(srv.ref))
        srv.ref = 0.0;

    double err_pos = srv.ref-elev_pos;
    //double err_vel = rate_ref-elev_rate;
    double err_vel = -elev_rate;
    double torque = srv.P_pos*err_pos + srv.P_vel*err_vel;
    hook_elev_joint->SetForce(0, torque);

    /*gzdbg<<hook_elev_joint->Position(0)<<"\n";
    gzdbg<<hook_elev_joint->GetForceTorque(0).body2Torque.Y()<<"\n";
    gzdbg<<"\n";
    gzdbg<<"ref: "<<u<<"\n";
    gzdbg<<"lidar: "<<h<<"\n";
    gzdbg<<"hook_joint: "<<hook_joint->Position(0)<<"\n";
    gzdbg<<"hook_elev_torque: "<<torque<<"\n";
    */
}

/////////////////////////////////////////////////
void GazeboPayloadPlugin::OnUpdate()
{

    if (!pubs_and_subs_created_) {
      CreatePubsAndSubs();
      pubs_and_subs_created_ = true;

      if (state_ == RESTING) {
        ignition::math::Pose3d pose_ini;
        pose_ini.Pos() = pickup_world_pos_payload_;
        payload_->SetWorldPose(pose_ini);
        payload_->SetAngularVel(V3D(0,0,0));
        payload_->SetLinearVel(V3D(0,0,0));
      }

    }

    common::Time current_time = world_->SimTime();

    // get payload pose on aircraft
    ignition::math::Pose3d pose_parent = parent_->WorldPose();
    ignition::math::Pose3d pose_payload = payload_->WorldPose();
    ignition::math::Pose3d pose_payload_reset;
    pose_payload_reset.Pos() = pose_parent.Pos() + pose_parent.Rot().RotateVector(hoist_pos_parent_) - pose_payload.Rot().RotateVector(hoist_pos_payload_);
    pose_payload_reset.Rot() = pose_parent.Rot();

    // get payload pose on pickup location
    ignition::math::Pose3d pose_payload_pickup;
    pose_payload_pickup.Pos() = pickup_world_pos_payload_;

    // position error quantities (expressed in world frame)
    ignition::math::Vector3d lin_vel_err = payload_->WorldLinearVel(hoist_pos_payload_) - parent_->WorldLinearVel(hoist_pos_parent_);
    ignition::math::Vector3d pos_err = pose_payload_reset.Pos()-pose_payload.Pos();
    lin_vel_err.Correct();

    // orientation error quantities (expressed in payload frame)
    ignition::math::Vector3d rot_vel_err = pose_payload.Rot().RotateVectorReverse(payload_->WorldAngularVel()-parent_->WorldAngularVel());
    ignition::math::Quaterniond rot_err_q = pose_parent.Rot().Inverse()*pose_payload.Rot();
    double angle_err;
    ignition::math::Vector3d rot_err;
    rot_err_q.ToAxis(rot_err,angle_err);
    rot_err *= -angle_err;

    // sticky-joint wrench controller, -> to get somewhat rigid connection between payload & aircraft
    double omega = omega_;            // desired natural frequency of connection
    double zeta = 1.0;                // damping ratio of connection
    double mass = 0.25;               // payload mass (<<aircraft mass) -> needs to match simulated mass!
    double inertia = 0.4*mass*0.0025; // inertia of solid sphere: 0.4*m*r²
    double k_p_lin = omega*omega*mass;
    double k_p_rot = omega*omega*inertia;
    double k_d_lin = 2*zeta*omega*mass;
    double k_d_rot = 2*zeta*omega*inertia;
    ignition::math::Vector3d force  = k_p_lin*pos_err - k_d_lin*lin_vel_err;
    ignition::math::Vector3d torque = k_p_rot*rot_err - k_d_rot*rot_vel_err;
    force.Correct();
    torque.Correct();

    double actio;   // apply force/torque on payload
    double reactio; // apply force/torque on aircraft

    // state machine

    State next_state;

    switch (state_) {

      case ATTACHED:
        actio = 1;
        reactio = 1;

        if (trigg_)
          next_state = FALLING;

        else
          next_state = ATTACHED;

        break;

      case FALLING:
        actio = 0;
        reactio = 0;

        if (prev_state_ != FALLING) {
          reload_timer_.Start();
          next_state = FALLING;

        } else if (reload_timer_.GetElapsed().Float() < 8.0) {
          next_state = FALLING;

        } else {
          reload_timer_.Stop();
          reload_timer_.Reset();
          next_state = TELEPORTING;
        }

        break;

      case TELEPORTING:
        actio = 0;
        reactio = 0;

        if (teleport_back_to_pickup_) { // ... to pickup location
            payload_->SetWorldPose(pose_payload_pickup);
            payload_->SetAngularVel(V3D(0,0,0));
            payload_->SetLinearVel(V3D(0,0,0));
            next_state = RESTING;

        } else if (teleport_back_to_aircraft_) { // ... back to aircraft
            payload_->SetWorldPose(pose_payload_reset);
            payload_->SetAngularVel(pose_payload.Rot().RotateVectorReverse(parent_->WorldAngularVel()));
            payload_->SetLinearVel(pose_payload.Rot().RotateVectorReverse(parent_->WorldLinearVel(hoist_pos_parent_)));
            next_state = ATTACHED;

        } else {
            next_state = RESTING;
        }

        break;

      case RESTING:
        actio = 0;
        reactio = 0;

        if (proximity_recovery_ && pos_err.Length() <= recovery_radius_) {
          payload_->SetWorldPose(pose_payload_reset);
          payload_->SetAngularVel(pose_payload.Rot().RotateVectorReverse(parent_->WorldAngularVel()));
          payload_->SetLinearVel(pose_payload.Rot().RotateVectorReverse(parent_->WorldLinearVel(hoist_pos_parent_)));
          next_state = ATTACHED;

        } else {
          next_state = RESTING;
        }

        break;
    }

    prev_state_ = state_;
    state_ = next_state;

    // update connection
    //parent_->AddForceAtRelativePosition(-reactio*force, hoist_pos_parent_);
    payload_->AddForceAtRelativePosition(actio*force, hoist_pos_payload_);

    if (!pos_only_)
      payload_->AddRelativeTorque((int)actio*torque);

    // others
    if (do_hook_ctrl)
        hook_ctrl_.DoControl();

    gazebo::msgs::Vector3d tracking_pos;
    tracking_pos.set_x(pose_payload.Pos().X());
    tracking_pos.set_y(pose_payload.Pos().Y());
    tracking_pos.set_z(pose_payload.Pos().Z());
    tracking_pos_pub_->Publish(tracking_pos);

}

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

#include <stdio.h>
#include <iostream>
#include <fstream>

#include <ignition/math.hh>
#include <math.h>
#include <boost/bind.hpp>
#include <Eigen/Eigen>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>

#include "CommandMotorSpeed.pb.h"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "PropulsionSlipstream.pb.h"
#include "VisVectorArray.pb.h"
#include "ConnectGazeboToRosTopic.pb.h"
#include "Float32.pb.h"

#include "common.h"
#include "uav_parameters.h"

namespace gazebo {

typedef const boost::shared_ptr<const gz_visualization_msgs::VisVectorArray> GzVisVectorArrayMsgPtr;
typedef const boost::shared_ptr<const gz_std_msgs::Float32> GzFloat32MsgPtr;
typedef ignition::math::Vector3d V3D;
typedef ignition::math::Matrix3<double> M3D;

// Default values
static constexpr double kDefaulMaxRotVelocity = 838.0;
static constexpr double kDefaultRhoAir = 1.255;

class GazeboPropulsion : public ModelPlugin{
public:
    GazeboPropulsion():
        ModelPlugin(){}

    virtual ~GazeboPropulsion();

protected:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    //virtual void OnUpdate(const common::UpdateInfo & _info);
    virtual void OnUpdate();

private:
    std::string namespace_;
    transport::NodePtr node_handle_;
    physics::WorldPtr world_;
    physics::ModelPtr model_;
    event::ConnectionPtr update_connection_;

    int update_counter_ = 0;
    double sampling_time_ = 0.0;    // simulation time-step [s]
    double prev_sim_time_ = 0.0;    // simulation time [s]

    struct Propeller {
        Propeller(){}

        PropellerParameters prop_params;

        physics::LinkPtr parent_link = nullptr;
        ignition::math::Vector3d p_joint{1,0,0};    // propeller joint, pos. rot. dir., expr. in parent link
        ignition::math::Vector3d p_cp{0,0,0};       // propeller hub position w.r.t parent link, expr. in parent link
        int turning_direction = 1;                  // 1: if thrust and rot. vect in same dir, otherwise -1

        M3D inertia;           // propeller (disk) inertia tensor, expressed in parent frame
        double omega = 0;      // propeller angular speed (wrt parent link) [rad/s]
        double omega_dot = 0;  // propeller angular acc (wrt parent link) [rad/s²]
        double omega_ref = 0;  // propeller angular ref speed (wrt parent link) [rad/s]
        double tau = 0.1;      // prop/motor time constant [s]
        double dt = 0.0;       // discrete time step for motor simulation [s]

        transport::SubscriberPtr omega_ref_sub = nullptr;
        std::string omega_ref_subtopic;

        ignition::math::Vector3d cp{0,0,0};

        transport::PublisherPtr prop_slpstr_pub = nullptr;
        std::string prop_slpstr_pubtopic;

        std::array<gz_visualization_msgs::ArrowMarker*,7> vec_vis;
        std::string vector_vis_array_topic;
        gz_visualization_msgs::VisVectorArray vector_vis_array_msg;
        gazebo::transport::PublisherPtr vector_vis_array_pub;

        void PropSpeedCallback(GzFloat32MsgPtr& ref){
            omega_ref = (double)ref->data();
            MotorDyn();
        }

        void MotorDyn(){

            if (tau>0.0 && dt>0.0) {
                omega = omega + std::min(dt, tau)/tau*(omega_ref-omega);
                omega_dot = (omega_ref-omega)*std::min(dt, tau)/(dt*tau);

            } else {
                gzerr<<"simulation time-step and/or motor time-constant equals zero\n";
            }

            if(isnan(omega)||isinf(omega)){
                omega = 0.0;
                gzerr<<"bad omega detected, setting to zero!!\n";
            }
        }
    };

    Propeller* propellers_;
    int num_props_ = 0;

    double max_rot_velocity_ = kDefaulMaxRotVelocity;   // [rad/s]
    double rho_air_ = kDefaultRhoAir;                   // [kg/m^3]

    gz_mav_msgs::PropulsionSlipstream propulsion_slipstream_msg_;
    bool pubs_and_subs_created_ = false;

    int Sgn(double val) {
        return (int)(0.0 < val) - (int)(val < 0.0);
    }

};
}

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

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <rotors_gazebo_plugins/motor_model.hpp>
#include "CommandMotorSpeed.pb.h"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "PropulsionSlipstream.pb.h"
#include "VisVectorArray.pb.h"
#include "ConnectGazeboToRosTopic.pb.h"
#include "Float32.pb.h"
#include <iostream>
#include <fstream>

#include "common.h"
#include "uav_parameters.h"

namespace gazebo {

typedef const boost::shared_ptr<const gz_visualization_msgs::VisVectorArray> GzVisVectorArrayMsgPtr;
typedef const boost::shared_ptr<const gz_std_msgs::Float32> GzFloat32MsgPtr;
typedef ignition::math::Vector3d v3d;
typedef ignition::math::Matrix3<double> m3d;
// Default values
static constexpr double kDefaulMaxRotVelocity = 838.0;
static constexpr double kDefaultRhoAir = 1.255;

class GazeboPropulsion : public ModelPlugin{
public:
    GazeboPropulsion():
          ModelPlugin(),
          n_props(0),
          max_rot_velocity_(kDefaulMaxRotVelocity),
          rho_air(kDefaultRhoAir),
          updateCounter(0),
          pubs_and_subs_created(false),
          prev_sim_time_(0),
          sampling_time_(0){}
    virtual ~GazeboPropulsion();

protected:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

private:

    std::string namespace_;                   //from sdf, required
    transport::NodePtr node_handle_;
    physics::WorldPtr world_;
    physics::ModelPtr model_;

    /// \brief Pointer to the update event connection.
    event::ConnectionPtr updateConnection_;
    int updateCounter;
    double sampling_time_;
    double prev_sim_time_;

    struct propeller {
        propeller(){}

        PropellerParameters prop_params_;

        physics::LinkPtr parentLink = nullptr;
        ignition::math::Vector3d P_joint{1,0,0};
        ignition::math::Vector3d P_cp{0,0,0};
        int turning_direction_ = 1;

        m3d inertia;
        double omega = 0;
        double omega_dot = 0;
        double omega_ref = 0;
        double tau = 0.1;
        double dt = 0.0;
        transport::SubscriberPtr omega_ref_sub_ = nullptr;
        std::string omega_ref_sub_topic;

        physics::JointPtr propJoint = nullptr;
        physics::LinkPtr propLink = nullptr;
        ignition::math::Vector3d cp{0,0,0};

        transport::PublisherPtr prop_slpstr_pub_ = nullptr;
        std::string prop_slpstr_pub_topic_;

        std::array<gz_visualization_msgs::ArrowMarker*,7> vec_vis;
        std::string vector_vis_array_topic;
        gz_visualization_msgs::VisVectorArray vector_vis_array_msg;
        gazebo::transport::PublisherPtr vector_vis_array_pub;

        void PropSpeedCallback(GzFloat32MsgPtr& ref){
            omega_ref = (double)ref->data();
            MotorDyn();
        }

        void MotorDyn(){
            // To Do: implement proper motor dynamics
            //omega = omega_ref;

            if (tau>0.0 && dt>0.0) {
                omega = omega + std::min(dt, tau)/tau*(omega_ref-omega);
                omega_dot = (omega_ref-omega)*std::min(dt, tau)/(dt*tau);

            } else {
                gzerr<<"simulation time-step and/or motor time-constant equals zero, division by zero\n";
            }

            if(isnan(omega)||isinf(omega)){
                omega = 0.0;
                gzerr<<"bad omega detected, setting to zero!!\n";
            }

        }
    };

    propeller* propellers;
    int n_props;

    double max_rot_velocity_;    // maximum rotor speed [rad/s]
    double rho_air;   // air density, [kg/m^3]

    gz_mav_msgs::PropulsionSlipstream propulsion_slipstream_msg_;

    bool pubs_and_subs_created;

    int sgn(double val) {
        return (int)(0.0 < val) - (int)(val < 0.0);
    }

};
}

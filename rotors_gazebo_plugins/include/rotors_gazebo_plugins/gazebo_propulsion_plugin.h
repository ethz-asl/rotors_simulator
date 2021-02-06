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
#include "WindSpeedBeta.pb.h"
#include "ConnectGazeboToRosTopic.pb.h"
#include "Float32.pb.h"
#include "vector2d.pb.h"

#include "common.h"
#include "uav_parameters.h"

namespace gazebo {

typedef const boost::shared_ptr<const gz_visualization_msgs::VisVectorArray> GzVisVectorArrayMsgPtr;
typedef const boost::shared_ptr<const gz_std_msgs::Float32> GzFloat32MsgPtr;
typedef const boost::shared_ptr<const gz_mav_msgs::WindSpeedBeta> WindPtr;
typedef ignition::math::Vector3d V3D;
typedef ignition::math::Matrix3<double> M3D;

// Default values
static constexpr double kDefaulMaxRotVelocity = 838.0;
static constexpr double kDefaultRhoAir = 1.2041; // air density 1.2041 kg/m³ (dry, @20 °C, 101.325 kPa)

class GazeboPropulsion : public ModelPlugin{
public:
    GazeboPropulsion():
        ModelPlugin(){}

    ~GazeboPropulsion();

protected:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    //void OnUpdate(const common::UpdateInfo & _info);
    void OnUpdate();

private:
    std::string namespace_;
    transport::NodePtr node_handle_;
    physics::WorldPtr world_;
    physics::ModelPtr model_;
    event::ConnectionPtr update_connection_;

    int update_counter_ = 0;
    double sampling_time_ = 0.0;    // simulation time-step [s]
    double prev_sim_time_ = 0.0;    // simulation time [s]

    struct Wind {
        Wind(){}

        transport::SubscriberPtr wind_sub_ = nullptr;
        std::mutex wind_lock;
        std::string wind_topic;

        V3D pos_ned = V3D(0,0,0);
        V3D wind_ned = V3D(0,0,0);
        M3D wind_grad_ned = M3D(0,0,0,0,0,0,0,0,0);

        void Callback(WindPtr& wind){
            std::unique_lock<std::mutex> lock(wind_lock);

            pos_ned = V3D(wind->pos_ned().x(),
                          wind->pos_ned().y(),
                          wind->pos_ned().z());

            wind_ned = V3D(wind->wind_ned().x(),
                           wind->wind_ned().y(),
                           wind->wind_ned().z());

            wind_grad_ned = M3D(wind->wind_grad_ned().xx(),
                                wind->wind_grad_ned().xy(),
                                wind->wind_grad_ned().xz(),
                                wind->wind_grad_ned().yx(),
                                wind->wind_grad_ned().yy(),
                                wind->wind_grad_ned().yz(),
                                wind->wind_grad_ned().zx(),
                                wind->wind_grad_ned().zy(),
                                wind->wind_grad_ned().zz());
            lock.unlock();
        }

        V3D GetWind(V3D p_cp){
            std::unique_lock<std::mutex> lock(wind_lock);   //necessary? atomic V3D?
            V3D wind_local = wind_ned + wind_grad_ned*(p_cp-pos_ned);
            //V3D wind_local = wind_grad_ned*(p_cp-pos_ned);
            lock.unlock();
            return wind_local;
        }
    };

    struct Propeller {
        Propeller(){}

        PropellerParameters prop_params;

        physics::LinkPtr ref_link = nullptr;  // reference link for force/torque calculation
        physics::LinkPtr act_link = nullptr;  // link to apply force/torque to

        ignition::math::Vector3d p_joint{1,0,0};    // propeller joint, pos. rot. dir., expr. in parent link
        ignition::math::Vector3d p_cp{0,0,0};       // propeller hub position w.r.t parent link, expr. in parent link
        int turning_direction = 1;                  // 1: if thrust and rot. vect in same dir, otherwise -1

        V3D H_Omega_Prev_{0,0,0};

        M3D inertia;           // propeller (disk) inertia tensor, expressed in parent frame
        double omega = 0;      // propeller angular speed (wrt parent link) [rad/s]
        double omega_dot = 0;  // propeller angular acc (wrt parent link) [rad/s²]
        double omega_ref = 0;  // propeller angular ref speed (wrt parent link) [rad/s]
        double omega_dead = 0; // ESC dead-zone. If omega_ref below this value prop wont spin [rad/s]
        double omega_max = 1e5;//
        double tau_p = 0.1;    // prop/motor time constant rising [s]
        double tau_n = 0.3;    // prop/motor time constant falling [s]
        double tau_su = 0.1;     // prop/motor time constant spool up [s]
        double dt = 0.0;       // discrete time step for motor simulation [s]

        transport::SubscriberPtr omega_ref_sub = nullptr;
        std::string omega_ref_subtopic;

        //ignition::math::Vector3d cp{0,0,0};

        transport::PublisherPtr prop_slpstr_pub = nullptr;
        std::string prop_slpstr_pubtopic;

        transport::PublisherPtr speed_pub = nullptr;

        std::array<gz_visualization_msgs::ArrowMarker*,7> vec_vis;
        gz_visualization_msgs::ArrowMarker* wind_vis;
        std::string vector_vis_array_topic;
        gz_visualization_msgs::VisVectorArray vector_vis_array_msg;
        gazebo::transport::PublisherPtr vector_vis_array_pub;

        Wind * wind;
        V3D wind_cp;
        int n_wind = 0;

        void PropSpeedCallback(GzFloat32MsgPtr& ref){
            omega_ref = (double)ref->data();
        }

        void MotorDyn(){

            // constrain speed reference with dead-zone
            if (omega_ref<omega_dead)
                omega_ref = 0.0;

            if (omega_ref>omega_max)
                omega_ref=omega_max;

            if (tau_p>0.0 && tau_n>0.0 && tau_su>0.0 && dt>0.0) {

                if (omega<omega_dead && omega_ref>=omega_dead) {
                    // throttle up from within deadzone

                    // Model 1: 1st order
                    omega = omega + std::min(dt, tau_su)/tau_su*(omega_ref-omega);
                    omega_dot = (omega_ref-omega)*std::min(dt, tau_su)/(dt*tau_su);

                    // Model 2: slew-rate:
                    /* double slew_dead = omega_dead/0.15/tau_su; // approx. fastest rise
                    omega = ignition::math::clamp(omega + slew_dead*dt, 0.0, omega_ref);
                    omega_dot = slew_dead; */

                } else {
                    if (omega_ref>=omega){
                        omega = omega + std::min(dt, tau_p)/tau_p*(omega_ref-omega);
                        omega_dot = (omega_ref-omega)*std::min(dt, tau_p)/(dt*tau_p);

                    } else {
                        omega = omega + std::min(dt, tau_n)/tau_n*(omega_ref-omega);
                        omega_dot = (omega_ref-omega)*std::min(dt, tau_n)/(dt*tau_n);
                    }
                }

                gazebo::msgs::Vector2d speed_msg;
                speed_msg.set_x(omega);
                speed_msg.set_y(omega_dot);

                if (speed_pub)
                    speed_pub->Publish(speed_msg);

            } else {
                gzerr<<"simulation time-step and/or motor time-constant equals zero\n";
            }

            if(isnan(omega)||isinf(omega)){
                omega = 0.0;
                gzerr<<"bad omega detected, setting to zero!!\n";
            }
        }

        void UpdateWind(V3D w_cp){
            wind_cp = V3D(0,0,0);
            for(int j=0; j<n_wind; j++){
                wind_cp += wind[j].GetWind(w_cp);
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

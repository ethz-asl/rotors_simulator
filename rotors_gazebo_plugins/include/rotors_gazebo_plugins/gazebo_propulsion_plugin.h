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
#include <iostream>
#include <fstream>

#include "common.h"
#include "uav_parameters.h"

namespace gazebo {

typedef const boost::shared_ptr<const gz_visualization_msgs::VisVectorArray> GzVisVectorArrayMsgPtr;
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
          pubs_and_subs_created(false){}
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

    struct propeller {
        propeller():
            propJoint(nullptr),
            propLink(nullptr),
            turning_direction_(1),
            cp(ignition::math::Vector3d(0,0,0)),
            prop_slpstr_pub_(nullptr),
            prop_slpstr_pub_topic_("slipstream"),
            vector_vis_array_topic("prop_vis"),
            vector_vis_array_pub(nullptr){}

        PropellerParameters prop_params_;

        physics::JointPtr propJoint;
        physics::LinkPtr propLink;
        int turning_direction_;

        ignition::math::Vector3d cp;
        transport::PublisherPtr prop_slpstr_pub_;
        std::string prop_slpstr_pub_topic_;

        std::array<gz_visualization_msgs::ArrowMarker*,4> vec_vis;
        std::string vector_vis_array_topic;
        gz_visualization_msgs::VisVectorArray vector_vis_array_msg;
        gazebo::transport::PublisherPtr vector_vis_array_pub;
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

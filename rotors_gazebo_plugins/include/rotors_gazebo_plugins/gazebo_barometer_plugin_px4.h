/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Barometer Plugin
 *
 * This plugin simulates barometer data
 *
 * @author Elia Tarasov <elias.tarasov@gmail.com>
 */

#ifndef _GAZEBO_BAROMETER_PLUGIN_HH_
#define _GAZEBO_BAROMETER_PLUGIN_HH_

#include "common.h"
#include <sdf/sdf.hh>

#include <string>
#include <random>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

#include <Pressure.pb.h>

namespace gazebo {

  static constexpr auto kDefaultBarometerTopic = "/baro";
  static constexpr auto kDefaultPubRate = 50.0;  // [Hz]. Note: averages the supported Baro device ODR in PX4
  static constexpr auto kDefaultAltHome = 488.0; // meters

  class BarometerPlugin : public ModelPlugin {
  public:
    BarometerPlugin();
    virtual ~BarometerPlugin();

  protected:
    virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
    virtual void OnUpdate(const common::UpdateInfo&);

  private:
    std::string namespace_;
    physics::ModelPtr model_;
    physics::WorldPtr world_;
    physics::LinkPtr link_;
    event::ConnectionPtr update_connection_;
    std::string baro_topic_;

    transport::NodePtr node_handle_;
    transport::PublisherPtr pub_baro_;

    sensor_msgs::msgs::Pressure baro_msg_;
    unsigned int pub_rate_;

    std::default_random_engine random_generator_;
    std::normal_distribution<double> standard_normal_distribution_;

    common::Time last_pub_time_;
    common::Time last_time_;

    ignition::math::Pose3d pose_model_start_;
    ignition::math::Vector3d gravity_W_;
    double alt_home_;

    // state variables for baro pressure sensor random noise generator
    double baro_rnd_y2_;
    bool baro_rnd_use_last_;

    double baro_drift_pa_per_sec_;
    double baro_drift_pa_;
  }; // class BarometerPlugin
} // namespace gazebo
#endif // _GAZEBO_BAROMETER_PLUGIN_HH_

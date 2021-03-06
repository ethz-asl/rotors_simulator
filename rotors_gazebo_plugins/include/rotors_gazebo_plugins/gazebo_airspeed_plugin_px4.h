/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @brief Airspeed Plugin
 *
 * This plugin publishes Airspeed sensor data
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 */

#ifndef _GAZEBO_AIRSPEED_PLUGIN_HH_
#define _GAZEBO_AIRSPEED_PLUGIN_HH_

#include <math.h>
#include <cstdio>
#include <cstdlib>
#include <queue>
#include <random>

#include <sdf/sdf.hh>
#include "common.h"
#include <random>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

#include <Airspeed.pb.h>
#include "WindSpeedBeta.pb.h"

namespace gazebo
{

//typedef const boost::shared_ptr<const physics_msgs::msgs::Wind> WindPtr;
typedef const boost::shared_ptr<const gz_mav_msgs::WindSpeedBeta> WindPtr;
typedef ignition::math::Vector3d V3D;
typedef ignition::math::Matrix3<double> M3D;

class GAZEBO_VISIBLE AirspeedPlugin : public ModelPlugin
{
public:
  AirspeedPlugin();
  virtual ~AirspeedPlugin();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo&);

private:

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::LinkPtr link_;

  transport::NodePtr node_handle_;
  transport::SubscriberPtr wind_sub_;
  transport::PublisherPtr airspeed_pub_;
  event::ConnectionPtr updateConnection_;

  common::Time last_time_;
  std::string namespace_;
  std::string link_name_;

  ignition::math::Vector3d wind_vel_;

  std::default_random_engine random_generator_;
  std::normal_distribution<float> standard_normal_distribution_;

  float diff_pressure_stddev_;
  float temperature_;

  struct Wind {
      Wind(){}

      transport::SubscriberPtr wind_sub_ = nullptr;
      std::mutex wind_lock;
      std::string wind_topic;

      V3D pos_ned = V3D(0,0,0);
      V3D wind_ned = V3D(0,0,0);
      M3D wind_grad_ned = M3D(0,0,0,0,0,0,0,0,0);

      void Callback(WindPtr& wind) {
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

      V3D GetWind(V3D p_cp) {
          std::unique_lock<std::mutex> lock(wind_lock);   //necessary? atomic V3D?
          V3D wind_local = wind_ned + wind_grad_ned*(p_cp-pos_ned);
          //V3D wind_local = wind_grad_ned*(p_cp-pos_ned);
          lock.unlock();
          return wind_local;
      }

  } wind_;

};     // class GAZEBO_VISIBLE AirspeedPlugin
}      // namespace gazebo
#endif // _GAZEBO_AIRSPEED_PLUGIN_HH_

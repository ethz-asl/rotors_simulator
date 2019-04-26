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
//#include "MotorSpeed.pb.h"
//#include "Float.pb.h"
#include "PropulsionSlipstream.pb.h"
//#include <Int32.pb.h>
#include <iostream>
#include <fstream>

#include "common.h"


namespace turning_direction {
const static int CCW = 1;
const static int CW = -1;
}

namespace gazebo {
// Default values
//static const std::string kDefaultNamespace = "";
static const std::string kDefaultCommandSubTopic = "/gazebo/command/motor_speed";
static const std::string kDefaultMotorFailureNumSubTopic = "/gazebo/motor_failure_num";
//static const std::string kDefaultMotorVelocityPubTopic = "/motor_speed";
static const std::string kDefaultPropulsionSlipstreamPubTopic = "/propulsion_slipstream";
static const std::string kDefaultDoLogSubTopic = "/do_log";
    
typedef const boost::shared_ptr<const gz_mav_msgs::CommandMotorSpeed> CommandMotorSpeedPtr;
//typedef const boost::shared_ptr<const std_msgs::msgs::Int32> Int32Ptr;

// Set the max_force_ to the max double value. The limitations get handled by the FirstOrderFilter.
static constexpr double kDefaultMaxForce = std::numeric_limits<double>::max();
static constexpr double kDefaultTimeConstantUp = 1.0 / 80.0;
static constexpr double kDefaultTimeConstantDown = 1.0 / 40.0;
static constexpr double kDefaulMaxRotVelocity = 0.0;//838.0;
static constexpr double kDefaultRotorDragCoefficient = 0.0;//1.0e-4;
static constexpr double kDefaultRollingMomentCoefficient = 0.0;//1.0e-6;
//static constexpr double kDefaultRotorVelocitySlowdownSim = 10.0;
static constexpr double kDefaultDiameter = 0.0;//0.2;
static constexpr double kDefaultRhoAir = 1.255;

class GazeboPropulsion : public MotorModel, public ModelPlugin{
 public:
  GazeboPropulsion()
      : ModelPlugin(),
        MotorModel(),
        command_sub_topic_(kDefaultCommandSubTopic),
        motor_failure_sub_topic_(kDefaultMotorFailureNumSubTopic),
        //motor_speed_pub_topic_(kDefaultMotorVelocityPubTopic),
        propulsion_slipstream_pub_topic_(kDefaultPropulsionSlipstreamPubTopic),
        motor_number_(0),
        motor_Failure_Number_(0),
        turning_direction_(turning_direction::CW),
        esc_("sqrt"),
        max_force_(kDefaultMaxForce),
        max_rot_velocity_(kDefaulMaxRotVelocity),
        ref_motor_rot_vel_(0.0),
        rolling_moment_coefficient_(kDefaultRollingMomentCoefficient),
        rotor_drag_coefficient_(kDefaultRotorDragCoefficient),
        rotor_velocity_slowdown_sim_(kDefaultRotorVelocitySlowdownSim),
        time_constant_down_(kDefaultTimeConstantDown),
        time_constant_up_(kDefaultTimeConstantUp),
        diameter(kDefaultDiameter),
        disk_area(M_PI/4*kDefaultDiameter*kDefaultDiameter),
        rho_air(kDefaultRhoAir),
        k_T(0.0),
        k_T0(0.0),
        k_Q(0.0),
        k_Q0(0.0),
        d_flow(-5),
        updateCounter(0),
        logItv(100),
        logName("example.txt"),
        logFlag(false),
        logStarted(false),
        logEnable(false),
        headerFlag(true),
        do_log_sub_topic_(kDefaultDoLogSubTopic){
            
        propulsion_slipstream_msg_.set_prop_diam(kDefaultDiameter);
            
        std::cout<<"motor_model constructed"<<std::endl;
  }

  virtual ~GazeboPropulsion();

  virtual void InitializeParams();
  virtual void Publish();

 protected:
  virtual void UpdateForcesAndMoments();
  /// \brief A function to check the motor_Failure_Number_ and stimulate motor fail
  /// \details Doing joint_->SetVelocity(0,0) for the flagged motor to fail
  virtual void UpdateMotorFail();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

 private:
  std::string command_sub_topic_;           //from sdf
  std::string motor_failure_sub_topic_;     //default only
  std::string joint_name_;                  //from sdf, required
  std::string link_name_;                   //from sdf, required
  //std::string motor_speed_pub_topic_;       //from sdf
  std::string propulsion_slipstream_pub_topic_;   //from sdf
  std::string namespace_;                   //from sdf, required

  int motor_number_;        //from sdf, required
  int turning_direction_;   //from sdf, required

  int motor_Failure_Number_;    //!< motor_Failure_Number is (motor_number_ + 1) as (0) is considered no_fail. Publish accordingly
  int tmp_motor_num;            // A temporary variable used to print msg

  int screen_msg_flag = 1;

  std::string esc_;                     // describes mapping from pwm to rpm, either: 'sqrt' -> n/n_max = sqrt(pwm/pwm_max) | 'lin' -> n/n_max = pwm/pwm_max
  double max_force_;                    // gets set to the max double value
  double max_rot_velocity_;             // maximum rotor speed [rad/s]
  double ref_motor_rot_vel_;            // input to first order filter approximating motor dynamics, [rad/s]
  double rolling_moment_coefficient_;   // used for prop roll moment: R = turn_dir*n[rad/s]*coeff*v_perp[m/s], [m*kg/rad], from sdf
  double rotor_drag_coefficient_;       // used for normal force: N = n [rad/s]*coeff*v_perp[m/s], [kg/rad], from sdf
  double rotor_velocity_slowdown_sim_;  // to prevent aliasing in sim, [-]
  double time_constant_down_;           // propeller deceleration time constant (1st order system approx), [s], from sdf
  double time_constant_up_;             // propeller acceleration time constant (1st order system approx), [s], from sdf
  double diameter;                      // propeller diameter, [m], from sdf
  double disk_area;                     // calculated from diameter, [m], from sdf
  
  double rho_air;   // air density, [kg/m^3]
    
  double k_T;       // = dC_T/dJ [-], for thrust: T = rho*n^2*D^4*(k_T0+J*k_T), n in [revs/s]!
  double k_T0;      // = C_T(J=0)[-], for thrust: T = rho*n^2*D^4*(k_T0+J*k_T), n in [revs/s]!
  double k_Q;       // = dC_Q/dJ [-], for drag-torque: Q = rho*n^2*D^5*(k_Q0+J*Q_T), n in [revs/s]!
  double k_Q0;      // = C_Q(J=0)[-], for drag-torque: Q = rho*n^2*D^5*(k_Q0+J*Q_T), n in [revs/s]!
  double d_flow;
    
  transport::NodePtr node_handle_;
  transport::PublisherPtr motor_velocity_pub_;
  transport::PublisherPtr propulsion_slipstream_pub_;
  transport::SubscriberPtr command_sub_;
  transport::SubscriberPtr motor_failure_sub_;  /*!< Subscribing to motor_failure_sub_topic_; receiving motor number to fail, as an integer */

  int debug_counter = 0;
  
  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::JointPtr joint_;
  common::PID pid_;
  bool use_pid_;
  physics::LinkPtr link_;
  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread();
  //std_msgs::msgs::Float turning_velocity_msg_;
  gz_mav_msgs::PropulsionSlipstream propulsion_slipstream_msg_;
  void VelocityCallback(CommandMotorSpeedPtr &rot_velocities);
  void MotorFailureCallback(const boost::shared_ptr<const msgs::Int> &fail_msg);  /*!< Callback for the motor_failure_sub_ subscriber */
  std::unique_ptr<FirstOrderFilter<double>>  rotor_velocity_filter_;
   
  //logging

  int updateCounter = 0;
  int logItv = 100;
  std::ofstream logfile;
  std::string logName = "example.txt";
  bool logFlag = false;
  bool logStarted = false;
  bool logEnable = false;
  bool headerFlag = true;
  std::string do_log_sub_topic_;
  transport::SubscriberPtr do_log_sub_;
  //void DoLogCallback(Int32Ptr& do_log);
    
  int sgn(double val) {
        return (int)(0.0 < val) - (int)(val < 0.0);
  }

};
}

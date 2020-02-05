/*
 * Copyright 2016 Pavel Vechersky, ASL, ETH Zurich, Switzerland
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
 */

// MODULE HEADER
#include "rotors_gazebo_plugins/gazebo_pressure_plugin.h"

// USER HEADERS
#include "ConnectGazeboToRosTopic.pb.h"

namespace gazebo {

GazeboPressurePlugin::GazeboPressurePlugin()
    : ModelPlugin(),
      node_handle_(0),
      pubs_and_subs_created_(false) {
}

GazeboPressurePlugin::~GazeboPressurePlugin() {
}

void GazeboPressurePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  gzdbg << "_model = " << _model->GetName() << std::endl;

  // Store the pointer to the model and the world.
  model_ = _model;
  world_ = model_->GetWorld();

  //==============================================//
  //========== READ IN PARAMS FROM SDF ===========//
  //==============================================//

  // Use the robot namespace to create the node handle.
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_pressure_plugin] Please specify a robotNamespace.\n";

  // Get node handle.
  node_handle_ = transport::NodePtr(new transport::Node());

  // Initialise with default namespace (typically /gazebo/default/).
  node_handle_->Init();

  std::string link_name;
  if (_sdf->HasElement("linkName"))
    link_name = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_pressure_plugin] Please specify a linkName.\n";
  // Get the pointer to the link.
  link_ = model_->GetLink(link_name);
  if (link_ == NULL)
    gzthrow("[gazebo_pressure_plugin] Couldn't find specified link \"" << link_name << "\".");

  frame_id_ = link_name;

  // Retrieve the rest of the SDF parameters.
  getSdfParam<std::string>(_sdf, "pressureTopic", pressure_topic_, kDefaultPressurePubTopic);
  getSdfParam<double>(_sdf, "referenceAltitude", ref_alt_, kDefaultRefAlt);
  getSdfParam<double>(_sdf, "pressureVariance", pressure_var_, kDefaultPressureVar);
  CHECK(pressure_var_ >= 0.0);

  // Initialize the normal distribution for pressure.
  double mean = 0.0;
  pressure_n_[0] = NormalDistribution(mean, sqrt(pressure_var_));

  // Listen to the update event. This event is broadcast every simulation
  // iteration.
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboPressurePlugin::OnUpdate, this, _1));

  //==============================================//
  //=== POPULATE STATIC PARTS OF PRESSURE MSG ====//
  //==============================================//

  pressure_message_.mutable_header()->set_frame_id(frame_id_);
  pressure_message_.set_variance(pressure_var_);
}

void GazeboPressurePlugin::OnUpdate(const common::UpdateInfo& _info) {
  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  common::Time current_time = world_->GetSimTime();

  // Get the current geometric height.
  double height_geometric_m = ref_alt_ + model_->GetWorldPose().pos.z;

  // Compute the geopotential height.
  double height_geopotential_m = kEarthRadiusMeters * height_geometric_m /
      (kEarthRadiusMeters + height_geometric_m);

  // Compute the temperature at the current altitude.
  double temperature_at_altitude_kelvin =
      kSeaLevelTempKelvin - kTempLapseKelvinPerMeter * height_geopotential_m;

  // Compute the current air pressure.
  double pressure_at_altitude_pascal =
      kPressureOneAtmospherePascals * exp(kAirConstantDimensionless *
          log(kSeaLevelTempKelvin / temperature_at_altitude_kelvin));

  // Add noise to pressure measurement.
  if(pressure_var_ > 0.0) {
    pressure_at_altitude_pascal += pressure_n_[0](random_generator_);
  }

  // Fill the pressure message.
  pressure_message_.mutable_header()->mutable_stamp()->set_sec(
      current_time.sec);
  pressure_message_.mutable_header()->mutable_stamp()->set_nsec(
      current_time.nsec);
  pressure_message_.set_fluid_pressure(pressure_at_altitude_pascal);

  // Publish the pressure message.
  pressure_pub_->Publish(pressure_message_);
}

void GazeboPressurePlugin::CreatePubsAndSubs() {
  // Create temporary "ConnectGazeboToRosTopic" publisher and message.
  gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
          "~/" + kConnectGazeboToRosSubtopic, 1);

  // ============================================ //
  // ========= FLUID PRESSURE MSG SETUP ========= //
  // ============================================ //

  pressure_pub_ = node_handle_->Advertise<gz_sensor_msgs::FluidPressure>(
      "~/" + namespace_ + "/" + pressure_topic_, 1);

  gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;
  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   pressure_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                pressure_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::FLUID_PRESSURE);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboPressurePlugin);

}  // namespace gazebo

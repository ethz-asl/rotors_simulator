/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
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

#include "rotors_gazebo_plugins/gazebo_external_wrench_plugin.h"

#include <fstream>
#include <math.h>

#include "ConnectGazeboToRosTopic.pb.h"
#include <boost/bind.hpp>
namespace gazebo {

GazeboExternalWrenchPlugin::~GazeboExternalWrenchPlugin() {
  
}

void GazeboExternalWrenchPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }
  std::cout << "gazebo_external_wrench gets loaded!! " << std::endl;

  // Store the pointer to the model.
  model_ = _model;
  world_ = model_->GetWorld();

  // wrench_msg_pub_ =
  //           nh_.advertise<mav_msgs::TorqueThrust>("wrench_target", 1);

  double wind_gust_start = kDefaultWindGustStart;
  double wind_gust_duration = kDefaultWindGustDuration;

  //==============================================//
  //========== READ IN PARAMS FROM SDF ===========//
  //==============================================//

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_external_wrench_plugin] Please specify a robotNamespace.\n";

  // Create Gazebo Node.
  gz_node_handle_ = gazebo::transport::NodePtr(new transport::Node());

  // Initialise with default namespace (typically /gazebo/default/).
  gz_node_handle_->Init();

//  if (_sdf->HasElement("xyzOffset"))
//    xyz_offset_ = _sdf->GetElement("xyzOffset")->Get<ignition::math::Vector3d >();
//  else
//    gzerr << "[gazebo_external_wrench_plugin] Please specify a xyzOffset.\n";
//
//  getSdfParam<std::string>(_sdf, "windForcePubTopic", wind_force_pub_topic_,
//                           wind_force_pub_topic_);
//  getSdfParam<std::string>(_sdf, "windSpeedPubTopic", wind_speed_pub_topic_,
//                           wind_speed_pub_topic_);
//  getSdfParam<std::string>(_sdf, "frameId", frame_id_, frame_id_);
  getSdfParam<std::string>(_sdf, "linkName", link_name_, link_name_);
  getSdfParam<std::string>(_sdf, "externalWrenchSubTopic", external_wrench_sub_topic_, external_wrench_sub_topic_);
//  // Get the wind speed params from SDF.
//  getSdfParam<double>(_sdf, "windSpeedMean", wind_speed_mean_,
//                      wind_speed_mean_);
//  getSdfParam<double>(_sdf, "windSpeedVariance", wind_speed_variance_,
//                      wind_speed_variance_);
//  getSdfParam<ignition::math::Vector3d >(_sdf, "windDirection", wind_direction_,
//                      wind_direction_);
//  // Check if a custom static wind field should be used.
//  getSdfParam<bool>(_sdf, "useCustomStaticWindField", use_custom_static_wind_field_,
//                      use_custom_static_wind_field_);
//  if (!use_custom_static_wind_field_) {
//    gzdbg << "[gazebo_external_wrench_plugin] Using user-defined constant wind field and gusts.\n";
//    // Get the wind params from SDF.
//    getSdfParam<double>(_sdf, "windForceMean", wind_force_mean_,
//                        wind_force_mean_);
//    getSdfParam<double>(_sdf, "windForceVariance", wind_force_variance_,
//                        wind_force_variance_);
//    // Get the wind gust params from SDF.
//    getSdfParam<double>(_sdf, "windGustStart", wind_gust_start, wind_gust_start);
//    getSdfParam<double>(_sdf, "windGustDuration", wind_gust_duration,
//                        wind_gust_duration);
//    getSdfParam<double>(_sdf, "windGustForceMean", wind_gust_force_mean_,
//                        wind_gust_force_mean_);
//    getSdfParam<double>(_sdf, "windGustForceVariance", wind_gust_force_variance_,
//                        wind_gust_force_variance_);
//    getSdfParam<ignition::math::Vector3d >(_sdf, "windGustDirection", wind_gust_direction_,
//                        wind_gust_direction_);
//
//    wind_direction_.Normalize();
//    wind_gust_direction_.Normalize();
//    wind_gust_start_ = common::Time(wind_gust_start);
//    wind_gust_end_ = common::Time(wind_gust_start + wind_gust_duration);
//  } else {
//    gzdbg << "[gazebo_external_wrench_plugin] Using custom wind field from text file.\n";
//    // Get the wind field text file path, read it and save data.
//    std::string custom_wind_field_path;
//    getSdfParam<std::string>(_sdf, "customWindFieldPath", custom_wind_field_path,
//                        custom_wind_field_path);
//    ReadCustomWindField(custom_wind_field_path);
//  }

  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_external_wrench_plugin] Couldn't find specified link \"" << link_name_
                                                                   << "\".");

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboExternalWrenchPlugin::OnUpdate, this, _1));
}

// This gets called by the world update start event.
void GazeboExternalWrenchPlugin::OnUpdate(const common::UpdateInfo& _info) {
  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }
  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  // Get the current simulation time.
  common::Time now = world_->SimTime();
  
  ignition::math::Vector3d wind_velocity(0.0, 0.0, 0.0);
  ignition::math::Vector3d disturbance_force(wrench_stamped_msg_.wrench().force().x(), wrench_stamped_msg_.wrench().force().y(), wrench_stamped_msg_.wrench().force().z());
  ignition::math::Vector3d disturbance_torque(wrench_stamped_msg_.wrench().torque().x(), wrench_stamped_msg_.wrench().torque().y(), wrench_stamped_msg_.wrench().torque().z());
  std::cout << "disturbance_force: (" << wrench_stamped_msg_.wrench().force().x() << ", " << wrench_stamped_msg_.wrench().force().y() << ", " << wrench_stamped_msg_.wrench().force().z() << ")" << std::endl;
  std::cout << "disturbance_torque: (" << wrench_stamped_msg_.wrench().torque().x() << ", " << wrench_stamped_msg_.wrench().torque().y() << ", " << wrench_stamped_msg_.wrench().torque().z() << ")" << std::endl;
  //add disturbance force and torque in the vehicle's own frame 
  link_->AddRelativeForce(disturbance_force); 
  link_->AddRelativeTorque(disturbance_torque);


  
}


void GazeboExternalWrenchPlugin::CreatePubsAndSubs() {
  // Create temporary "ConnectGazeboToRosTopic" publisher and message.
  gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
      gz_node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
          "~/" + kConnectGazeboToRosSubtopic, 1);


  gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;
    std::cout << "kConnectRosToGazeboSubtopic" << kConnectRosToGazeboSubtopic << std::endl;
    // Create temporary "ConnectRosToGazeboTopic" publisher and message
    gazebo::transport::PublisherPtr gz_connect_ros_to_gazebo_topic_pub =
            gz_node_handle_->Advertise<gz_std_msgs::ConnectRosToGazeboTopic>(
                    "~/" + kConnectRosToGazeboSubtopic, 1);
    gz_std_msgs::ConnectRosToGazeboTopic connect_ros_to_gazebo_topic_msg;

  // ============================================ //
  // ========= WRENCH STAMPED MSG SETUP ========= //
  // ============================================ //
  wind_force_pub_ = gz_node_handle_->Advertise<gz_geometry_msgs::WrenchStamped>(
      "~/" + namespace_ + "/" + wind_force_pub_topic_, 1);

  // connect_gazebo_to_ros_topic_msg.set_gazebo_namespace(namespace_);
  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   wind_force_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                wind_force_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::WRENCH_STAMPED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);

  // ============================================ //
  // ========== WIND SPEED MSG SETUP ============ //
  // ============================================ //
  wind_speed_pub_ = gz_node_handle_->Advertise<gz_mav_msgs::WindSpeed>(
      "~/" + namespace_ + "/" + wind_speed_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   wind_speed_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                wind_speed_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::WIND_SPEED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);

  // ================================================================== //
  // ========== WRENCH DISTURBANCE MSG SETUP (ROS->GAZEBO) ============ //
  // ================================================================== //
  external_wrench_sub_ =
    gz_node_handle_->Subscribe("~/" + namespace_ + "/" + external_wrench_sub_topic_, &GazeboExternalWrenchPlugin::RosWrenchMsgCallback, this);
//    RosWrenchMsgCallback is not getting called
    connect_ros_to_gazebo_topic_msg.set_ros_topic(
            namespace_ + "/" + external_wrench_sub_topic_);
    connect_ros_to_gazebo_topic_msg.set_gazebo_topic(
            "~/" + namespace_ + "/" + external_wrench_sub_topic_);
    connect_ros_to_gazebo_topic_msg.set_msgtype(
            gz_std_msgs::ConnectRosToGazeboTopic::WRENCH_STAMPED);
    gz_connect_ros_to_gazebo_topic_pub->Publish(
            connect_ros_to_gazebo_topic_msg, true);

}

void GazeboExternalWrenchPlugin::ConvertHeaderRosToGz(
        const std_msgs::Header_<std::allocator<void> >& ros_header,
        gz_std_msgs::Header* gz_header) {
    gz_header->mutable_stamp()->set_sec(ros_header.stamp.sec);
    gz_header->mutable_stamp()->set_nsec(ros_header.stamp.nsec);
    gz_header->set_frame_id(ros_header.frame_id);
}

void GazeboExternalWrenchPlugin::RosWrenchMsgCallback(GzWrenchStampedMsgPtr& wrench_stamped_msg){

    // Convert ROS message to Gazebo message
//    ConvertHeaderRosToGz(ros_wrench_msg_ptr->header, gz_wrench_stamped_msg.mutable_header());
    wrench_stamped_msg_.mutable_wrench()->mutable_force()->set_x(wrench_stamped_msg->wrench().force().x());
    wrench_stamped_msg_.mutable_wrench()->mutable_force()->set_y(wrench_stamped_msg->wrench().force().y());
    wrench_stamped_msg_.mutable_wrench()->mutable_force()->set_z(wrench_stamped_msg->wrench().force().z());
    wrench_stamped_msg_.mutable_wrench()->mutable_torque()->set_x(wrench_stamped_msg->wrench().torque().x());
    wrench_stamped_msg_.mutable_wrench()->mutable_torque()->set_y(wrench_stamped_msg->wrench().torque().y());
    wrench_stamped_msg_.mutable_wrench()->mutable_torque()->set_z(wrench_stamped_msg->wrench().torque().z());
//    // Publish to Gazebo
//    gz_publisher_ptr->Publish(gz_wrench_stamped_msg);


}

void GazeboExternalWrenchPlugin::GzConnectRosToGazeboTopicMsgCallback(
    GzConnectRosToGazeboTopicMsgPtr& gz_connect_ros_to_gazebo_topic_msg) {

    static std::vector<ros::Subscriber> ros_subscribers;

//    switch (gz_connect_ros_to_gazebo_topic_msg->msgtype()){
//        case gz_std_msgs::ConnectGazeboToRosTopic::WRENCH_STAMPED: {
        gazebo::transport::PublisherPtr gz_publisher_ptr =
                gz_node_handle_->Advertise<gz_geometry_msgs::WrenchStamped>(
                        gz_connect_ros_to_gazebo_topic_msg->gazebo_topic(),1);
        //create ROS subscriber
//        ros::Subscriber ros_subscriber =
//                ros_node_handle_->subscribe<mav_msgs::TorqueThrust>(
//                        gz_connect_ros_to_gazebo_topic_msg->ros_topic(), 1,
//                        boost::bind(&GazeboExternalWrenchPlugin::RosWrenchMsgCallback, this, _1, gz_publisher_ptr));

        // Save reference to the ROS subscriber so callback will continue to be
        // called.
//        ros_subscribers.push_back(ros_subscriber);

//            break;
//
//        }
//        default: {
//            gzthrow("ConnectRosToGazeboTopic message type with enum val = "
//                            << gz_connect_ros_to_gazebo_topic_msg->msgtype()
//                            << " is not supported by GazeboExternalWrenchPlugin.");
//        }
//    }

}

void GazeboExternalWrenchPlugin::ReadCustomWindField(std::string& custom_wind_field_path) {
  std::ifstream fin;
  fin.open(custom_wind_field_path);
  if (fin.is_open()) {
    std::string data_name;
    float data;
    // Read the line with the variable name.
    while (fin >> data_name) {
      // Save data on following line into the correct variable.
      if (data_name == "min_x:") {
        fin >> min_x_;
      } else if (data_name == "min_y:") {
        fin >> min_y_;
      } else if (data_name == "n_x:") {
        fin >> n_x_;
      } else if (data_name == "n_y:") {
        fin >> n_y_;
      } else if (data_name == "res_x:") {
        fin >> res_x_;
      } else if (data_name == "res_y:") {
        fin >> res_y_;
      } else if (data_name == "vertical_spacing_factors:") {
        while (fin >> data) {
          vertical_spacing_factors_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "bottom_z:") {
        while (fin >> data) {
          bottom_z_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "top_z:") {
        while (fin >> data) {
          top_z_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "u:") {
        while (fin >> data) {
          u_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "v:") {
        while (fin >> data) {
          v_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "w:") {
        while (fin >> data) {
          w_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else {
        // If invalid data name, read the rest of the invalid line, 
        // publish a message and ignore data on next line. Then resume reading.
        std::string restOfLine;
        getline(fin, restOfLine);
        gzerr << " [gazebo_external_wrench_plugin] Invalid data name '" << data_name << restOfLine <<
              "' in custom wind field text file. Ignoring data on next line.\n";
        fin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      }
    }
    fin.close();
    gzdbg << "[gazebo_external_wrench_plugin] Successfully read custom wind field from text file.\n";
  } else {
    gzerr << "[gazebo_external_wrench_plugin] Could not open custom wind field text file.\n";
  }

}

ignition::math::Vector3d GazeboExternalWrenchPlugin::LinearInterpolation(
  double position, ignition::math::Vector3d * values, double* points) const {
  ignition::math::Vector3d value = values[0] + (values[1] - values[0]) /
                        (points[1] - points[0]) * (position - points[0]);
  return value;
}

ignition::math::Vector3d GazeboExternalWrenchPlugin::BilinearInterpolation(
  double* position, ignition::math::Vector3d * values, double* points) const {
  ignition::math::Vector3d intermediate_values[2] = { LinearInterpolation(
                                             position[0], &(values[0]), &(points[0])),
                                           LinearInterpolation(
                                             position[0], &(values[2]), &(points[2])) };
  ignition::math::Vector3d value = LinearInterpolation(
                          position[1], intermediate_values, &(points[4]));
  return value;
}

ignition::math::Vector3d GazeboExternalWrenchPlugin::TrilinearInterpolation(
  ignition::math::Vector3d link_position, ignition::math::Vector3d * values, double* points) const {
  double position[3] = {link_position.X(),link_position.Y(),link_position.Z()};
  ignition::math::Vector3d intermediate_values[4] = { LinearInterpolation(
                                             position[2], &(values[0]), &(points[0])),
                                           LinearInterpolation(
                                             position[2], &(values[2]), &(points[2])),
                                           LinearInterpolation(
                                             position[2], &(values[4]), &(points[4])),
                                           LinearInterpolation(
                                             position[2], &(values[6]), &(points[6])) };
  ignition::math::Vector3d value = BilinearInterpolation(
    &(position[0]), intermediate_values, &(points[8]));
  return value;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboExternalWrenchPlugin);

}  // namespace gazebo

#ifndef INCLUDE_ROTORS_CONTROL_PARAMETERS_ROS_H_
#define INCLUDE_ROTORS_CONTROL_PARAMETERS_ROS_H_

#include <glog/logging.h>
#include <ros/ros.h>

namespace rotors_control {

template<typename T> void GetRosParameter(const ros::NodeHandle& nh,
                                          const std::string& key,
                                          const T& default_value,
                                          T* value) {
  CHECK_NOTNULL(value);
  bool have_parameter = nh.getParam(key, *value);
  if (!have_parameter) {
    ROS_WARN_STREAM("[rosparam]: could not find parameter " << nh.getNamespace()
                    << "/" << key << ", setting to default: " << default_value);
    *value = default_value;
  }
}
}

#endif /* INCLUDE_ROTORS_CONTROL_PARAMETERS_ROS_H_ */

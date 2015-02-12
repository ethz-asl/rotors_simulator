#include <ros/ros.h>

template<typename T> void GetRosParameter(const ros::NodeHandle& nh,
                                          const std::string& key,
                                          const T& default_value,
                                          T* value) {
  bool have_parameter = nh.getParam(key, *value);
  if (!have_parameter) {
    ROS_WARN_STREAM("[rosparam]: could not find parameter " << nh.getNamespace()
                    << "/" << key << ", setting to default: " << default_value);
    *value = default_value;
  }
}

/*
 * Copyright (C) 2014 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch> and
 * <markus.achtelik_devel at mavt dot ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <gazebo/gazebo.hh>

namespace gazebo{

/**
 * \brief Obtains a parameter from sdf.
 * \param[in] sdf Pointer to the sdf object.
 * \param[in] name Name of the parameter.
 * \param[out] param Param Variable to write the parameter to.
 * \param[in] default_value Default value, if the parameter not available.
 * \param[in] message If passed, then this error message will be passed to gzerror if the parameter is not available.
 */
template<class T>
bool getSdfParam(sdf::ElementPtr sdf, const std::string& name, T& param, const T& default_value,
                 const std::string& message = "") {
  if (sdf->HasElement(name)) {
    param = sdf->GetElement(name)->Get<T>();
    return true;
  } else {
    param = default_value;
    if (message.empty())
      gzerr << "[mav_gazebo_plugins] Please specify a value for parameter \"" << name << "\".\n";
    else if (message == "default") {
      gzwarn << "[mav_gazebo_plugins] No value specified in sdf for parameter \""
         << name << "\", using default value " << default_value << ".\n";
    }
    else
      gzerr << "[mav_gazebo_plugins] " << message <<".\n";
  }
  return false;
}

}



#endif /* COMMON_H_ */

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
 */


#ifndef ROTORS_GAZEBO_PLUGINS_SDF_API_WRAPPER_H
#define ROTORS_GAZEBO_PLUGINS_SDF_API_WRAPPER_H

#include <gazebo/gazebo.hh>

namespace gazebo {

#if SDF_MAJOR_VERSION >= 3
  typedef math::Vector3 SdfVector3;
#else
  class SdfVector3 : public sdf::Vector3 {
  /*
  A wrapper class for deprecated sdf::Vector3 class to provide the same accessor
  functions as in the newer ignition::math::Vector3 class.
  */

   public:
    using sdf::Vector3::Vector3;
    virtual ~SdfVector3() {}

    /// \brief Get the x value
    /// \return the x value
    double X() { return this->x; }

    /// \brief Get the y value
    /// \return the y value
    double Y() { return this->y; }

    /// \brief Get the z value
    /// \return the z value
    double Z() { return this->z; }
  };
#endif
}

#endif // ROTORS_GAZEBO_PLUGINS_SDF_API_WRAPPER_H

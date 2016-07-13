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


#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_API_WRAPPER_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_API_WRAPPER_H

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo {

#if GAZEBO_MAJOR_VERSION > 6
  typedef sensors::GpsSensor GazeboGpsSensor;
#elif GAZEBO_MAJOR_VERSION == 6
  class GazeboGpsSensor : public sensors::GpsSensor {
  /*
  A wrapper class for member functions in sensors::GpsSensor class of Gazebo
  API version 6 that have since been deprecated.
  */

   public:
    using sensors::GpsSensor::GpsSensor;
    virtual ~GazeboGpsSensor() {}

    /// \brief Get the altitude value
    /// \return the altitude value
    double Altitude() { return this->GetAltitude(); }

    /// \brief Get the last measurement time
    /// \return the last measurement time
    common::Time LastMeasurementTime() { return this->GetLastMeasurementTime(); }
  };
#else
  class GazeboGpsSensor : public sensors::GpsSensor {
  /*
  A wrapper class for member functions in sensors::GpsSensor class of Gazebo
  API version 5 or older that have since been deprecated.
  */

   public:
    using sensors::GpsSensor::GpsSensor;
    virtual ~GazeboGpsSensor() {}

    /// \brief Get the altitude value
    /// \return the altitude value
    double Altitude() { return this->GetAltitude(); }

    /// \brief Get the latitude value
    /// \return the latitude value
    math::Angle Latitude() { return this->GetLatitude(); }

    /// \brief Get the longitude value
    /// \return the longitude value
    math::Angle Longitude() { return this->GetLongitude(); }

    /// \brief Get the last measurement time
    /// \return the last measurement time
    common::Time LastMeasurementTime() { return this->GetLastMeasurementTime(); }
  };
#endif

typedef std::shared_ptr<GazeboGpsSensor> GazeboGpsSensorPtr;
}

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_API_WRAPPER_H

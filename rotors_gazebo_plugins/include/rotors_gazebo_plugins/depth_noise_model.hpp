/*
 * Copyright 2018 Michael Pantic, ASL, ETH Zurich, Switzerland
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


#ifndef ROTORS_GAZEBO_PLUGINS_DEPTH_NOISE_MODEL_H
#define ROTORS_GAZEBO_PLUGINS_DEPTH_NOISE_MODEL_H

#include <random>
#include <Eigen/Eigen>

class DepthNoiseModel {

 public:
  DepthNoiseModel() :
      gen(std::random_device{}()) {

  }
  virtual void ApplyNoise(float *data, int width, int height) = 0;

 protected:
  std::normal_distribution<float> dist;
  std::mt19937 gen;
};

class KinectDepthNoiseModel : public DepthNoiseModel {

 public:
  KinectDepthNoiseModel() :
      DepthNoiseModel() {

  }

  void ApplyNoise(float *data, int width, int height);
};


class D435DepthNoiseModel : public DepthNoiseModel {

 public:

  D435DepthNoiseModel() :
      h_fov(M_PI_2), // Default 90deg for D435
      baseline(50),  // Default 50 mm for D435
      subpixel_err(0.1), // Default subpixel calibration error
      max_depth(100),
      min_depth(0.2),
      max_stdev(3.0),
      DepthNoiseModel() {

  }

  void ApplyNoise(float *data, int width, int height);

 private:

  const float bad_point = std::numeric_limits<float>::quiet_NaN();
  float min_depth;    // [m]
  float max_depth;    // [m]
  // parameters
  float h_fov;        // [rad]
  float baseline;     // [mm]
  float subpixel_err; // [pixel] Calibration error
  float max_stdev;    // [m] cutoff for distance covariance

};

#endif // ROTORS_GAZEBO_PLUGINS_DEPTH_NOISE_MODEL_H

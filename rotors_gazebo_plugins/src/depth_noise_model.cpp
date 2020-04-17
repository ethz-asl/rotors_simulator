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
#include <algorithm>
#include <iostream>

#include <rotors_gazebo_plugins/depth_noise_model.hpp>

inline bool DepthNoiseModel::InRange(const float depth) const {
  return depth > this->min_depth && depth < this->max_depth;
}

void D435DepthNoiseModel::ApplyNoise(const uint32_t width,
                                     const uint32_t height, float *data) {
  if (data == nullptr) {
    return;
  }

  float f = 0.5f * (width / tanf(h_fov / 2.0f));
  float multiplier = (subpixel_err) / (f * baseline * 1e6f);
  Eigen::Map<Eigen::VectorXf> data_vector_map(data, width * height);

  // Formula taken from the Intel Whitepaper:
  // "Best-Known-Methods for Tuning Intel RealSense™ D400 Depth Cameras for Best Performance".
  // We are using the theoretical RMS model formula.
  Eigen::VectorXf rms_noise = (data_vector_map * 1000.0).array().square() * multiplier;
  Eigen::VectorXf noise = rms_noise.array().square();

  // Sample noise for each pixel and transform variance according to error at this depth.
  for (int i = 0; i < width * height; ++i) {
    if (InRange(data_vector_map[i])) {
      data_vector_map[i] +=
          this->dist(this->gen) * std::min(((float)noise(i)), max_stdev);
    } else {
      data_vector_map[i] = this->bad_point;
    }
  }
}

void KinectDepthNoiseModel::ApplyNoise(const uint32_t width,
                                       const uint32_t height, float *data) {
  if (data == nullptr) {
    return;
  }

  // Axial noise model from
  // https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6375037,
  // Nguyen, Izadi & Lovell: "Modeling Kinect Sensor Noise for Improved 3D Reconstrucion and Tracking", 3DIM/3DPVT, 2012.
  // We are using the 10-60 Degree model as an approximation.
  Eigen::Map<Eigen::VectorXf> data_vector_map(data, width * height);
  Eigen::VectorXf var_noise = 0.0012f + 0.0019f * (data_vector_map.array() - 0.4f).array().square();

  // Sample noise for each pixel and transform variance according to error at this depth.
  for (int i = 0; i < width * height; ++i) {
    if (InRange(data_vector_map[i])) {
      data_vector_map[i] += this->dist(this->gen) * var_noise(i);
    } else {
      data_vector_map[i] = this->bad_point;
    }
  }
}



void PMDDepthNoiseModel::ApplyNoise(const uint32_t width,
                                       const uint32_t height, float *data) {
  if (data == nullptr) {
    return;
  }

  // 1% error claimed by PMD
  Eigen::Map<Eigen::VectorXf> data_vector_map(data, width * height);
  Eigen::VectorXf var_noise =  data_vector_map.array() * 0.01;

  // Sample noise for each pixel and transform variance according to error at this depth.
  for (int i = 0; i < width * height; ++i) {
    if (InRange(data_vector_map[i])) {
      data_vector_map[i] += this->dist(this->gen) * var_noise(i);
    } else {
      data_vector_map[i] = this->bad_point;
    }
  }
}

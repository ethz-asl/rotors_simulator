#include <rotors_gazebo_plugins/depth_noise_model.hpp>
#include <iostream>
#include <algorithm>    // std::max

void D435DepthNoiseModel::ApplyNoise(float *data, int width, int height) {

  float f = 0.5 * (width / tan(h_fov / 2.0));
  float multiplier = (subpixel_err) / (f * baseline * 1000.0);

  Eigen::Map<Eigen::VectorXf> v_data(data, width * height);

  // formula as specified in Intel Whitepaper
  // "Best-Known-Methods for Tuning Intel® RealSense™ D400 Depth Cameras for Best Performance"
  // -> this is using the theoretical RMS limit formula.
  auto rms_noise = (v_data * 1000.0).array().square() * multiplier;
  auto noise = rms_noise.array().square();

  // sample noise for each pixel and transform variance according to error at this depth.
  for (int i = 0; i < width * height; i++) {
    if (v_data[i] > this->min_depth &&
        v_data[i] < this->max_depth) {
      v_data[i] += this->dist(this->gen) * std::min(((float) noise(i)), max_stdev);
    } else {
      v_data[i] = this->bad_point;
    }
  }
}

void KinectDepthNoiseModel::ApplyNoise(float *data, int width, int height) {
  // Axial noise model from https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6375037
  // Nguyen, Izadi & Lovell: "Modeling Kinect Sensor Noise for Improved 3D Reconstrucion and Tracking", 3DIM/3DPVT, 2012


  // model for 10-60 deg. angle. Not very correct, but easy to implement..

  Eigen::Map<Eigen::VectorXf> v_data(data, width * height);

  auto var_noise = 0.0012 + 0.0019 * (v_data.array() - 0.4).array().square();

  // sample noise for each pixel and transform variance according to error at this depth.
  for (int i = 0; i < width * height; i++) {
    v_data[i] += this->dist(this->gen) * var_noise(i);
  }

}


#pragma once

namespace ORB_SLAM3 {

enum class Sensor {
  Monocular         = 0, // Monocular camera
  Stereo            = 1, // Stereo camera
  RGBD              = 2, // RGB-D camera
  InertialMonocular = 3, // IMU + Monocular camera
  InertialStereo    = 4, // IMU + Stereo camera
  InertialRGBD      = 5, // IMU + RGB-D camera
};

} // namespace ORB_SLAM3

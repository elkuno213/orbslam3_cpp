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

// Checks if sensor has IMU.
bool IsInertialBased(Sensor sensor);

// Check if sensor has monocular camera.
bool IsMonocularBased(Sensor sensor);

// Check if sensor has stereo camera.
bool IsStereoBased(Sensor sensor);

// Check if sensor has RGB-D camera.
bool IsRGBDBased(Sensor sensor);

} // namespace ORB_SLAM3

#include "Common/Common.h"

namespace ORB_SLAM3 {

bool IsInertialBased(Sensor sensor) {
  // clang-format off
  return sensor == Sensor::InertialMonocular
      || sensor == Sensor::InertialStereo
      || sensor == Sensor::InertialRGBD;
  // clang-format on
}

bool IsMonocularBased(Sensor sensor) {
  return sensor == Sensor::Monocular || sensor == Sensor::InertialMonocular;
}

bool IsStereoBased(Sensor sensor) {
  return sensor == Sensor::Stereo || sensor == Sensor::InertialStereo;
}

bool IsRGBDBased(Sensor sensor) {
  return sensor == Sensor::RGBD || sensor == Sensor::InertialRGBD;
}

} // namespace ORB_SLAM3

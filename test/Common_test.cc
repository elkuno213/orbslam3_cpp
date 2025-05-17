#include "Common/Common.h"
#include <gtest/gtest.h>

using namespace ORB_SLAM3;

TEST(CommonTest, IsInertialBased) {
  EXPECT_TRUE(IsInertialBased(Sensor::InertialMonocular));
  EXPECT_TRUE(IsInertialBased(Sensor::InertialStereo));
  EXPECT_TRUE(IsInertialBased(Sensor::InertialRGBD));

  EXPECT_FALSE(IsInertialBased(Sensor::Monocular));
  EXPECT_FALSE(IsInertialBased(Sensor::Stereo));
  EXPECT_FALSE(IsInertialBased(Sensor::RGBD));
}

TEST(CommonTest, IsMonocularBased) {
  EXPECT_TRUE(IsMonocularBased(Sensor::Monocular));
  EXPECT_TRUE(IsMonocularBased(Sensor::InertialMonocular));

  EXPECT_FALSE(IsMonocularBased(Sensor::Stereo));
  EXPECT_FALSE(IsMonocularBased(Sensor::RGBD));
  EXPECT_FALSE(IsMonocularBased(Sensor::InertialStereo));
  EXPECT_FALSE(IsMonocularBased(Sensor::InertialRGBD));
}

TEST(CommonTest, IsStereoBased) {
  EXPECT_TRUE(IsStereoBased(Sensor::Stereo));
  EXPECT_TRUE(IsStereoBased(Sensor::InertialStereo));

  EXPECT_FALSE(IsStereoBased(Sensor::Monocular));
  EXPECT_FALSE(IsStereoBased(Sensor::RGBD));
  EXPECT_FALSE(IsStereoBased(Sensor::InertialMonocular));
  EXPECT_FALSE(IsStereoBased(Sensor::InertialRGBD));
}

TEST(CommonTest, IsRGBDBased) {
  EXPECT_TRUE(IsRGBDBased(Sensor::RGBD));
  EXPECT_TRUE(IsRGBDBased(Sensor::InertialRGBD));

  EXPECT_FALSE(IsRGBDBased(Sensor::Monocular));
  EXPECT_FALSE(IsRGBDBased(Sensor::Stereo));
  EXPECT_FALSE(IsRGBDBased(Sensor::InertialMonocular));
  EXPECT_FALSE(IsRGBDBased(Sensor::InertialStereo));
}

#include "Tracking.h"
#include <fstream>
#include <gtest/gtest.h>
#include "Atlas.h"
#include "FrameDrawer.h"
#include "KeyFrameDatabase.h"
#include "MapDrawer.h"
#include "ORBVocabulary.h"
#include "System.h"

using namespace ORB_SLAM3;

const std::string kSettingsFile = "/tmp/settings.yaml";

const char* kPinholeYamlConfig = R"(%YAML:1.0
---
Camera.type: "PinHole"

# Camera 1 information
Camera.fx: 400.0
Camera.fy: 450.0
Camera.cx: 350.0
Camera.cy: 250.0
Camera.k1: 0.1
Camera.k2: 0.2
Camera.k3: 0.3
Camera.p1: 0.0002
Camera.p2: 1e-05
Camera.imageScale: 1.0
)";

const char* kKannalaBrandt8YamlConfig = R"(%YAML:1.0
---
Camera.type: "KannalaBrandt8"

# Camera 1 information
Camera.fx: 400.0
Camera.fy: 450.0
Camera.cx: 350.0
Camera.cy: 250.0
Camera.k1: 0.1
Camera.k2: 0.2
Camera.k3: 0.3
Camera.k4: 0.4
Camera.imageScale: 1.0

# Camera 2 information
Camera2.fx: 400.0
Camera2.fy: 450.0
Camera2.cx: 350.0
Camera2.cy: 250.0
Camera2.k1: 0.1
Camera2.k2: 0.2
Camera2.k3: 0.3
Camera1.k4: 0.4
Camera2.p1: -0.0001
Camera2.p2: -1e-05

# Stereo/RGB-D configuration
Camera.lappingBegin: 0
Camera.lappingEnd: 100
Camera2.lappingBegin: 10
Camera2.lappingEnd: 90
T_c1_c2: !!opencv-matrix
  rows: 4
  cols: 4
  dt: f
  data: [1,0,0,0,
         0,1,0,0,
         0,0,1,0]

Camera.bf: 0.1
Camera.fps: 30.0
Camera.RGB: 1
ThDepth: 30.0

# RGB-D configuration
DepthMapFactor: 5000.0
)";

const char* kCommonYamlConfig = R"(
# ORB configuration
ORBextractor.nFeatures: 1000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

# IMU configuration
Tbc: !!opencv-matrix
  rows: 4
  cols: 4
  dt: f
  data: [1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0]
InsertKFsWhenLost: 1
IMU.Frequency: 100.0
IMU.NoiseGyro: 0.01
IMU.NoiseAcc: 0.01
IMU.GyroWalk: 0.01
IMU.AccWalk: 0.01
IMU.fastInit: 1
)";

const char* kInvalidYamlConfig = R"(%YAML:1.0
---
Tracking.InexistentParam: 1
)";

class TrackingTest : public ::testing::Test {
protected:
  void TearDown() override {
    // std::remove(kSettingsFile.c_str());
  }

  void CreateValidConfig(const std::string& camera_model) {
    std::ofstream ofs(kSettingsFile);
    if (camera_model == "Pinhole") {
      ofs << kPinholeYamlConfig;
    } else if (camera_model == "KannalaBrandt8") {
      ofs << kKannalaBrandt8YamlConfig;
    }
    // ofs << kCommonYamlConfig;
    ofs.close();
  }

  void CreateInvalidConfig() {
    std::ofstream ofs(kSettingsFile);
    ofs << kInvalidYamlConfig;
    ofs.close();
  }
};

TEST_F(TrackingTest, ConstructorNonExistentFile) {
  ASSERT_THROW(
    {
      Tracking t(
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        "nonexistent.yaml",
        Sensor::Monocular,
        nullptr
      );
    },
    std::runtime_error
  );
}

TEST_F(TrackingTest, ConstructorInvalidFile) {
  CreateInvalidConfig();
  ASSERT_THROW(
    {
      Tracking t(
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        kSettingsFile,
        Sensor::Monocular,
        nullptr
      );
    },
    std::runtime_error
  );
}

// TODO(VuHoi): bug in Atlas::AddCamera, correct it and add this test.
// TEST_F(TrackingTest, ConstructorValidFile) {
// }

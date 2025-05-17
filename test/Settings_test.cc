#include "Settings.h"
#include <fstream>
#include <gtest/gtest.h>
#include "System.h"

using namespace ORB_SLAM3;

const std::string kSettingsFile = "/tmp/settings.yaml";

const char* kCommonYamlConfig = R"(
# Image information
Camera.height: 480
Camera.width: 752
Camera.newHeight: 350
Camera.newWidth: 600
Camera.fps: 20
Camera.RGB: 1

# IMU configuration
IMU.NoiseGyro: 0.01
IMU.NoiseAcc: 0.01
IMU.GyroWalk: 0.01
IMU.AccWalk: 0.01
IMU.Frequency: 100.0
IMU.T_b_c1: !!opencv-matrix
  rows: 4
  cols: 4
  dt: f
  data: [1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0]

# ORB settings
ORBextractor.nFeatures: 1000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

# Viewer configuration
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500.0
)";

const char* kPinholeYamlConfig = R"(%YAML:1.0
---
Camera.type: "PinHole"

# Camera 1 information
Camera1.fx: 400.0
Camera1.fy: 450.0
Camera1.cx: 350.0
Camera1.cy: 250.0
Camera1.k1: 0.1
Camera1.k2: 0.2
Camera1.k3: 0.3
Camera1.p1: 0.0002
Camera1.p2: 1e-05

# Camera 2 information
Camera2.fx: 400.0
Camera2.fy: 450.0
Camera2.cx: 350.0
Camera2.cy: 250.0
Camera2.k1: 0.1
Camera2.k2: 0.2
Camera2.k3: 0.3
Camera2.p1: -0.0001
Camera2.p2: -1e-05

# Stereo configuration
Stereo.ThDepth: 30.0
Stereo.T_c1_c2: !!opencv-matrix
  rows: 4
  cols: 4
  dt: f
  data: [0.999997256477797,-0.002317135723275,-0.000343393120620, 0.110074137800478,
         0.002312067192432, 0.999898048507103,-0.014090668452683,-0.000156612054392,
         0.000376008102320, 0.014089835846691, 0.999900662638081, 0.000889382785432,
                         0,                 0,                 0,                 1]

# RGB-D configuration
RGBD.DepthMapFactor: 5000.0
Stereo.b: 0.1
)";

const char* kKannalaBrandt8YamlConfig = R"(%YAML:1.0
---
Camera.type: "KannalaBrandt8"

# Camera 1 information
Camera1.fx: 400.0
Camera1.fy: 450.0
Camera1.cx: 350.0
Camera1.cy: 250.0
Camera1.k1: 0.1
Camera1.k2: 0.2
Camera1.k3: 0.3
Camera1.k4: 0.4
Camera1.overlappingBegin: 0
Camera1.overlappingEnd: 100

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
Camera2.overlappingBegin: 10
Camera2.overlappingEnd: 90

# Stereo configuration
Stereo.ThDepth: 30.0
Stereo.T_c1_c2: !!opencv-matrix
  rows: 4
  cols: 4
  dt: f
  data: [0.999997256477797,-0.002317135723275,-0.000343393120620, 0.110074137800478,
         0.002312067192432, 0.999898048507103,-0.014090668452683,-0.000156612054392,
         0.000376008102320, 0.014089835846691, 0.999900662638081, 0.000889382785432,
                         0,                 0,                 0,                 1]

# RGB-D configuration
RGBD.DepthMapFactor: 5000.0
Stereo.b: 0.1
)";

const char* kRectifiedYamlConfig = R"(%YAML:1.0
---
Camera.type: "Rectified"

# Camera 1 information
Camera1.fx: 400.0
Camera1.fy: 450.0
Camera1.cx: 350.0
Camera1.cy: 250.0

# Stereo configuration
Stereo.b: 0.1
Stereo.ThDepth: 30.0

# RGB-D configuration
RGBD.DepthMapFactor: 5000.0
)";

const char* kInvalidYamlConfig = R"(%YAML:1.0
---
Camera.type: "InvalidCamera"
)";

class SettingsTest : public ::testing::Test {
protected:
  void TearDown() override {
    std::remove(kSettingsFile.c_str());
  }

  void CreateValidConfig(const std::string& camera_model) {
    std::ofstream ofs(kSettingsFile);
    if (camera_model == "Pinhole") {
      ofs << kPinholeYamlConfig;
    } else if (camera_model == "KannalaBrandt8") {
      ofs << kKannalaBrandt8YamlConfig;
    } else if (camera_model == "Rectified") {
      ofs << kRectifiedYamlConfig;
    }
    ofs << kCommonYamlConfig;
    ofs.close();
  }

  void CreateInvalidConfig() {
    std::ofstream ofs(kSettingsFile);
    ofs << kInvalidYamlConfig;
    ofs.close();
  }
};

TEST_F(SettingsTest, ConstructorNonExistentFile) {
  ASSERT_THROW({ Settings s("nonexistent.yaml", Sensor::Monocular); }, std::runtime_error);
}

TEST_F(SettingsTest, ConstructorInvalidFile) {
  CreateInvalidConfig();
  ASSERT_THROW({ Settings s(kSettingsFile, Sensor::Monocular); }, std::runtime_error);
}

TEST_F(SettingsTest, ConstructorValidFile) {
  const std::vector<std::string> camera_models = {"Pinhole", "KannalaBrandt8", "Rectified"};
  for (const auto& model : camera_models) {
    CreateValidConfig(model);
    // clang-format off
    ASSERT_NO_THROW({ Settings s(kSettingsFile, Sensor::Stereo           ); });
    ASSERT_NO_THROW({ Settings s(kSettingsFile, Sensor::InertialStereo   ); });
    ASSERT_NO_THROW({ Settings s(kSettingsFile, Sensor::Monocular        ); });
    ASSERT_NO_THROW({ Settings s(kSettingsFile, Sensor::InertialMonocular); });
    ASSERT_NO_THROW({ Settings s(kSettingsFile, Sensor::RGBD             ); });
    ASSERT_NO_THROW({ Settings s(kSettingsFile, Sensor::InertialRGBD     ); });
    // clang-format on
  }
}

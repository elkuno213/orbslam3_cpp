/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel
 * and Juan D. Tardós, University of Zaragoza. Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M.
 * Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU
 * General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "Settings.h"
#include <fmt/core.h>
#include <fmt/ranges.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include "CameraModels/KannalaBrandt8.h"
#include "CameraModels/Pinhole.h"
#include "Converter.h"
#include "LoggingUtils.h"
#include "System.h"

namespace ORB_SLAM3 {

template <>
float Settings::readParameter<float>(
  cv::FileStorage& fSettings, const std::string& name, bool& found, const bool required
) {
  cv::FileNode node = fSettings[name];
  if (node.empty()) {
    if (required) {
      throw std::runtime_error(fmt::format("{} required parameter does not exist", name));
    } else {
      _logger->warn("{} optional parameter does not exist", name);
      found = false;
      return 0.0f;
    }
  } else if (!node.isReal()) {
    throw std::runtime_error(fmt::format("{} parameter must be a real number", name));
  } else {
    found = true;
    return node.real();
  }
}

template <>
int Settings::readParameter<int>(
  cv::FileStorage& fSettings, const std::string& name, bool& found, const bool required
) {
  cv::FileNode node = fSettings[name];
  if (node.empty()) {
    if (required) {
      throw std::runtime_error(fmt::format("{} required parameter does not exist", name));
    } else {
      _logger->warn("{} optional parameter does not exist", name);
      found = false;
      return 0;
    }
  } else if (!node.isInt()) {
    throw std::runtime_error(fmt::format("{} parameter must be an integer number", name));
  } else {
    found = true;
    return node.operator int();
  }
}

template <>
std::string Settings::readParameter<std::string>(
  cv::FileStorage& fSettings, const std::string& name, bool& found, const bool required
) {
  cv::FileNode node = fSettings[name];
  if (node.empty()) {
    if (required) {
      throw std::runtime_error(fmt::format("{} required parameter does not exist", name));
    } else {
      _logger->warn("{} optional parameter does not exist", name);
      found = false;
      return std::string();
    }
  } else if (!node.isString()) {
    throw std::runtime_error(fmt::format("{} parameter must be a string", name));
  } else {
    found = true;
    return node.string();
  }
}

template <>
cv::Mat Settings::readParameter<cv::Mat>(
  cv::FileStorage& fSettings, const std::string& name, bool& found, const bool required
) {
  cv::FileNode node = fSettings[name];
  if (node.empty()) {
    if (required) {
      throw std::runtime_error(fmt::format("{} required parameter does not exist", name));
    } else {
      _logger->warn("{} optional parameter does not exist", name);
      found = false;
      return cv::Mat();
    }
  } else {
    found = true;
    return node.mat();
  }
}

Settings::Settings(const std::string& configFile, const int& sensor)
  : bNeedToUndistort_(false)
  , bNeedToRectify_(false)
  , bNeedToResize1_(false)
  , bNeedToResize2_(false)
  , _logger(logging::CreateModuleLogger("Settings")) {
  sensor_ = sensor;

  // Open settings file
  cv::FileStorage fSettings(configFile, cv::FileStorage::READ);
  if (!fSettings.isOpened()) {
    throw std::runtime_error(fmt::format("Failed to open configuration file at {}", configFile));
  }

  _logger->info("Loading settings from {}...", configFile);

  // Read first camera
  readCamera1(fSettings);
  _logger->info("Camera 1 loaded");

  // Read second camera if stereo (not rectified)
  if (sensor_ == System::STEREO || sensor_ == System::IMU_STEREO) {
    readCamera2(fSettings);
    _logger->info("Camera 2 loaded");
  }

  // Read image info
  readImageInfo(fSettings);
  _logger->info("Camera info loaded");

  if (sensor_ == System::IMU_MONOCULAR || sensor_ == System::IMU_STEREO || sensor_ == System::IMU_RGBD) {
    readIMU(fSettings);
    _logger->info("IMU calibration loaded");
  }

  if (sensor_ == System::RGBD || sensor_ == System::IMU_RGBD) {
    readRGBD(fSettings);
    _logger->info("RGB-D calibration loaded");
  }

  readORB(fSettings);
  _logger->info("ORB settings loaded");
  readViewer(fSettings);
  _logger->info("Viewer settings loaded");
  readLoadAndSave(fSettings);
  _logger->info("Atlas settings loaded");
  readOtherParameters(fSettings);
  _logger->info("Misc parameters loaded");

  if (bNeedToRectify_) {
    precomputeRectificationMaps();
    _logger->info("Rectification maps computed");
  }
}

void Settings::readCamera1(cv::FileStorage& fSettings) {
  bool found;

  // Read camera model
  std::string cameraModel = readParameter<std::string>(fSettings, "Camera.type", found);

  std::vector<float> vCalibration;
  if (cameraModel == "PinHole") {
    cameraType_ = PinHole;

    // Read intrinsic parameters
    float fx = readParameter<float>(fSettings, "Camera1.fx", found);
    float fy = readParameter<float>(fSettings, "Camera1.fy", found);
    float cx = readParameter<float>(fSettings, "Camera1.cx", found);
    float cy = readParameter<float>(fSettings, "Camera1.cy", found);

    vCalibration = {fx, fy, cx, cy};

    calibration1_   = new Pinhole(vCalibration);
    originalCalib1_ = new Pinhole(vCalibration);

    // Check if it is a distorted PinHole
    readParameter<float>(fSettings, "Camera1.k1", found, false);
    if (found) {
      readParameter<float>(fSettings, "Camera1.k3", found, false);
      if (found) {
        vPinHoleDistorsion1_.resize(5);
        vPinHoleDistorsion1_[4] = readParameter<float>(fSettings, "Camera1.k3", found);
      } else {
        vPinHoleDistorsion1_.resize(4);
      }
      vPinHoleDistorsion1_[0] = readParameter<float>(fSettings, "Camera1.k1", found);
      vPinHoleDistorsion1_[1] = readParameter<float>(fSettings, "Camera1.k2", found);
      vPinHoleDistorsion1_[2] = readParameter<float>(fSettings, "Camera1.p1", found);
      vPinHoleDistorsion1_[3] = readParameter<float>(fSettings, "Camera1.p2", found);
    }

    // Check if we need to correct distortion from the images
    if ((sensor_ == System::MONOCULAR || sensor_ == System::IMU_MONOCULAR) && vPinHoleDistorsion1_.size() != 0) {
      bNeedToUndistort_ = true;
    }
  } else if (cameraModel == "Rectified") {
    cameraType_ = Rectified;

    // Read intrinsic parameters
    float fx = readParameter<float>(fSettings, "Camera1.fx", found);
    float fy = readParameter<float>(fSettings, "Camera1.fy", found);
    float cx = readParameter<float>(fSettings, "Camera1.cx", found);
    float cy = readParameter<float>(fSettings, "Camera1.cy", found);

    vCalibration = {fx, fy, cx, cy};

    calibration1_   = new Pinhole(vCalibration);
    originalCalib1_ = new Pinhole(vCalibration);

    // Rectified images are assumed to be ideal PinHole images (no distortion)
  } else if (cameraModel == "KannalaBrandt8") {
    cameraType_ = KannalaBrandt;

    // Read intrinsic parameters
    float fx = readParameter<float>(fSettings, "Camera1.fx", found);
    float fy = readParameter<float>(fSettings, "Camera1.fy", found);
    float cx = readParameter<float>(fSettings, "Camera1.cx", found);
    float cy = readParameter<float>(fSettings, "Camera1.cy", found);

    float k0 = readParameter<float>(fSettings, "Camera1.k1", found);
    float k1 = readParameter<float>(fSettings, "Camera1.k2", found);
    float k2 = readParameter<float>(fSettings, "Camera1.k3", found);
    float k3 = readParameter<float>(fSettings, "Camera1.k4", found);

    vCalibration = {fx, fy, cx, cy, k0, k1, k2, k3};

    calibration1_   = new KannalaBrandt8(vCalibration);
    originalCalib1_ = new KannalaBrandt8(vCalibration);

    if (sensor_ == System::STEREO || sensor_ == System::IMU_STEREO) {
      int              colBegin = readParameter<int>(fSettings, "Camera1.overlappingBegin", found);
      int              colEnd   = readParameter<int>(fSettings, "Camera1.overlappingEnd", found);
      std::vector<int> vOverlapping = {colBegin, colEnd};

      static_cast<KannalaBrandt8*>(calibration1_)->mvLappingArea = vOverlapping;
    }
  } else {
    throw std::runtime_error(fmt::format("{} not known", cameraModel));
  }
}

void Settings::readCamera2(cv::FileStorage& fSettings) {
  bool               found;
  std::vector<float> vCalibration;
  if (cameraType_ == PinHole) {
    bNeedToRectify_ = true;

    // Read intrinsic parameters
    float fx = readParameter<float>(fSettings, "Camera2.fx", found);
    float fy = readParameter<float>(fSettings, "Camera2.fy", found);
    float cx = readParameter<float>(fSettings, "Camera2.cx", found);
    float cy = readParameter<float>(fSettings, "Camera2.cy", found);

    vCalibration = {fx, fy, cx, cy};

    calibration2_   = new Pinhole(vCalibration);
    originalCalib2_ = new Pinhole(vCalibration);

    // Check if it is a distorted PinHole
    readParameter<float>(fSettings, "Camera2.k1", found, false);
    if (found) {
      readParameter<float>(fSettings, "Camera2.k3", found, false);
      if (found) {
        vPinHoleDistorsion2_.resize(5);
        vPinHoleDistorsion2_[4] = readParameter<float>(fSettings, "Camera2.k3", found);
      } else {
        vPinHoleDistorsion2_.resize(4);
      }
      vPinHoleDistorsion2_[0] = readParameter<float>(fSettings, "Camera2.k1", found);
      vPinHoleDistorsion2_[1] = readParameter<float>(fSettings, "Camera2.k2", found);
      vPinHoleDistorsion2_[2] = readParameter<float>(fSettings, "Camera2.p1", found);
      vPinHoleDistorsion2_[3] = readParameter<float>(fSettings, "Camera2.p2", found);
    }
  } else if (cameraType_ == KannalaBrandt) {
    // Read intrinsic parameters
    float fx = readParameter<float>(fSettings, "Camera2.fx", found);
    float fy = readParameter<float>(fSettings, "Camera2.fy", found);
    float cx = readParameter<float>(fSettings, "Camera2.cx", found);
    float cy = readParameter<float>(fSettings, "Camera2.cy", found);

    float k0 = readParameter<float>(fSettings, "Camera1.k1", found);
    float k1 = readParameter<float>(fSettings, "Camera1.k2", found);
    float k2 = readParameter<float>(fSettings, "Camera1.k3", found);
    float k3 = readParameter<float>(fSettings, "Camera1.k4", found);

    vCalibration = {fx, fy, cx, cy, k0, k1, k2, k3};

    calibration2_   = new KannalaBrandt8(vCalibration);
    originalCalib2_ = new KannalaBrandt8(vCalibration);

    int              colBegin = readParameter<int>(fSettings, "Camera2.overlappingBegin", found);
    int              colEnd   = readParameter<int>(fSettings, "Camera2.overlappingEnd", found);
    std::vector<int> vOverlapping = {colBegin, colEnd};

    static_cast<KannalaBrandt8*>(calibration2_)->mvLappingArea = vOverlapping;
  }

  // Load stereo extrinsic calibration
  if (cameraType_ == Rectified) {
    b_  = readParameter<float>(fSettings, "Stereo.b", found);
    bf_ = b_ * calibration1_->getParameter(0);
  } else {
    cv::Mat cvTlr = readParameter<cv::Mat>(fSettings, "Stereo.T_c1_c2", found);
    Tlr_          = Converter::toSophus(cvTlr);

    // TODO: also search for Trl and invert if necessary

    b_  = Tlr_.translation().norm();
    bf_ = b_ * calibration1_->getParameter(0);
  }

  thDepth_ = readParameter<float>(fSettings, "Stereo.ThDepth", found);
}

void Settings::readImageInfo(cv::FileStorage& fSettings) {
  bool found;
  // Read original and desired image dimensions
  int originalRows       = readParameter<int>(fSettings, "Camera.height", found);
  int originalCols       = readParameter<int>(fSettings, "Camera.width", found);
  originalImSize_.width  = originalCols;
  originalImSize_.height = originalRows;

  newImSize_   = originalImSize_;
  int newHeigh = readParameter<int>(fSettings, "Camera.newHeight", found, false);
  if (found) {
    bNeedToResize1_   = true;
    newImSize_.height = newHeigh;

    if (!bNeedToRectify_) {
      // Update calibration
      float scaleRowFactor = (float)newImSize_.height / (float)originalImSize_.height;
      calibration1_->setParameter(calibration1_->getParameter(1) * scaleRowFactor, 1);
      calibration1_->setParameter(calibration1_->getParameter(3) * scaleRowFactor, 3);

      if ((sensor_ == System::STEREO || sensor_ == System::IMU_STEREO) && cameraType_ != Rectified) {
        calibration2_->setParameter(calibration2_->getParameter(1) * scaleRowFactor, 1);
        calibration2_->setParameter(calibration2_->getParameter(3) * scaleRowFactor, 3);
      }
    }
  }

  int newWidth = readParameter<int>(fSettings, "Camera.newWidth", found, false);
  if (found) {
    bNeedToResize1_  = true;
    newImSize_.width = newWidth;

    if (!bNeedToRectify_) {
      // Update calibration
      float scaleColFactor = (float)newImSize_.width / (float)originalImSize_.width;
      calibration1_->setParameter(calibration1_->getParameter(0) * scaleColFactor, 0);
      calibration1_->setParameter(calibration1_->getParameter(2) * scaleColFactor, 2);

      if ((sensor_ == System::STEREO || sensor_ == System::IMU_STEREO) && cameraType_ != Rectified) {
        calibration2_->setParameter(calibration2_->getParameter(0) * scaleColFactor, 0);
        calibration2_->setParameter(calibration2_->getParameter(2) * scaleColFactor, 2);

        if (cameraType_ == KannalaBrandt) {
          static_cast<KannalaBrandt8*>(calibration1_)->mvLappingArea[0] *= scaleColFactor;
          static_cast<KannalaBrandt8*>(calibration1_)->mvLappingArea[1] *= scaleColFactor;

          static_cast<KannalaBrandt8*>(calibration2_)->mvLappingArea[0] *= scaleColFactor;
          static_cast<KannalaBrandt8*>(calibration2_)->mvLappingArea[1] *= scaleColFactor;
        }
      }
    }
  }

  fps_  = readParameter<int>(fSettings, "Camera.fps", found);
  bRGB_ = (bool)readParameter<int>(fSettings, "Camera.RGB", found);
}

void Settings::readIMU(cv::FileStorage& fSettings) {
  bool found;
  noiseGyro_    = readParameter<float>(fSettings, "IMU.NoiseGyro", found);
  noiseAcc_     = readParameter<float>(fSettings, "IMU.NoiseAcc", found);
  gyroWalk_     = readParameter<float>(fSettings, "IMU.GyroWalk", found);
  accWalk_      = readParameter<float>(fSettings, "IMU.AccWalk", found);
  imuFrequency_ = readParameter<float>(fSettings, "IMU.Frequency", found);

  cv::Mat cvTbc = readParameter<cv::Mat>(fSettings, "IMU.T_b_c1", found);
  Tbc_          = Converter::toSophus(cvTbc);

  readParameter<int>(fSettings, "IMU.InsertKFsWhenLost", found, false);
  if (found) {
    insertKFsWhenLost_ = (bool)readParameter<int>(fSettings, "IMU.InsertKFsWhenLost", found, false);
  } else {
    insertKFsWhenLost_ = true;
  }
}

void Settings::readRGBD(cv::FileStorage& fSettings) {
  bool found;

  depthMapFactor_ = readParameter<float>(fSettings, "RGBD.DepthMapFactor", found);
  thDepth_        = readParameter<float>(fSettings, "Stereo.ThDepth", found);
  b_              = readParameter<float>(fSettings, "Stereo.b", found);
  bf_             = b_ * calibration1_->getParameter(0);
}

void Settings::readORB(cv::FileStorage& fSettings) {
  bool found;

  nFeatures_   = readParameter<int>(fSettings, "ORBextractor.nFeatures", found);
  scaleFactor_ = readParameter<float>(fSettings, "ORBextractor.scaleFactor", found);
  nLevels_     = readParameter<int>(fSettings, "ORBextractor.nLevels", found);
  initThFAST_  = readParameter<int>(fSettings, "ORBextractor.iniThFAST", found);
  minThFAST_   = readParameter<int>(fSettings, "ORBextractor.minThFAST", found);
}

void Settings::readViewer(cv::FileStorage& fSettings) {
  bool found;

  keyFrameSize_      = readParameter<float>(fSettings, "Viewer.KeyFrameSize", found);
  keyFrameLineWidth_ = readParameter<float>(fSettings, "Viewer.KeyFrameLineWidth", found);
  graphLineWidth_    = readParameter<float>(fSettings, "Viewer.GraphLineWidth", found);
  pointSize_         = readParameter<float>(fSettings, "Viewer.PointSize", found);
  cameraSize_        = readParameter<float>(fSettings, "Viewer.CameraSize", found);
  cameraLineWidth_   = readParameter<float>(fSettings, "Viewer.CameraLineWidth", found);
  viewPointX_        = readParameter<float>(fSettings, "Viewer.ViewpointX", found);
  viewPointY_        = readParameter<float>(fSettings, "Viewer.ViewpointY", found);
  viewPointZ_        = readParameter<float>(fSettings, "Viewer.ViewpointZ", found);
  viewPointF_        = readParameter<float>(fSettings, "Viewer.ViewpointF", found);
  imageViewerScale_  = readParameter<float>(fSettings, "Viewer.imageViewScale", found, false);

  if (!found) {
    imageViewerScale_ = 1.0f;
  }
}

void Settings::readLoadAndSave(cv::FileStorage& fSettings) {
  bool found;

  sLoadFrom_ = readParameter<std::string>(fSettings, "System.LoadAtlasFromFile", found, false);
  sSaveto_   = readParameter<std::string>(fSettings, "System.SaveAtlasToFile", found, false);
}

void Settings::readOtherParameters(cv::FileStorage& fSettings) {
  bool found;

  thFarPoints_ = readParameter<float>(fSettings, "System.thFarPoints", found, false);
}

void Settings::precomputeRectificationMaps() {
  // Precompute rectification maps, new calibrations, ...
  cv::Mat K1 = static_cast<Pinhole*>(calibration1_)->toK();
  K1.convertTo(K1, CV_64F);
  cv::Mat K2 = static_cast<Pinhole*>(calibration2_)->toK();
  K2.convertTo(K2, CV_64F);

  cv::Mat cvTlr;
  cv::eigen2cv(Tlr_.inverse().matrix3x4(), cvTlr);
  cv::Mat R12 = cvTlr.rowRange(0, 3).colRange(0, 3);
  R12.convertTo(R12, CV_64F);
  cv::Mat t12 = cvTlr.rowRange(0, 3).col(3);
  t12.convertTo(t12, CV_64F);

  cv::Mat R_r1_u1, R_r2_u2;
  cv::Mat P1, P2, Q;

  cv::stereoRectify(
    K1,
    camera1DistortionCoef(),
    K2,
    camera2DistortionCoef(),
    newImSize_,
    R12,
    t12,
    R_r1_u1,
    R_r2_u2,
    P1,
    P2,
    Q,
    cv::CALIB_ZERO_DISPARITY,
    -1,
    newImSize_
  );
  cv::initUndistortRectifyMap(
    K1,
    camera1DistortionCoef(),
    R_r1_u1,
    P1.rowRange(0, 3).colRange(0, 3),
    newImSize_,
    CV_32F,
    M1l_,
    M2l_
  );
  cv::initUndistortRectifyMap(
    K2,
    camera2DistortionCoef(),
    R_r2_u2,
    P2.rowRange(0, 3).colRange(0, 3),
    newImSize_,
    CV_32F,
    M1r_,
    M2r_
  );

  // Update calibration
  calibration1_->setParameter(P1.at<double>(0, 0), 0);
  calibration1_->setParameter(P1.at<double>(1, 1), 1);
  calibration1_->setParameter(P1.at<double>(0, 2), 2);
  calibration1_->setParameter(P1.at<double>(1, 2), 3);

  // Update bf
  bf_ = b_ * P1.at<double>(0, 0);

  // Update relative pose between camera 1 and IMU if necessary
  if (sensor_ == System::IMU_STEREO) {
    Eigen::Matrix3f eigenR_r1_u1;
    cv::cv2eigen(R_r1_u1, eigenR_r1_u1);
    Sophus::SE3f T_r1_u1(eigenR_r1_u1, Eigen::Vector3f::Zero());
    Tbc_ = Tbc_ * T_r1_u1.inverse();
  }
}

std::string Settings::Str() const {
  std::string output;

  output += fmt::format("SLAM settings:\n");

  output += fmt::format(
    "- Camera 1 parameters ({}): [ {:.6f} ]\n",
    (cameraType_ == PinHole || cameraType_ == Rectified) ? "Pinhole" : "Kannala-Brandt",
    fmt::join(originalCalib1_->parameters(), " ")
  );

  if (!vPinHoleDistorsion1_.empty()) {
    output += fmt::format(
      "- Camera 1 distortion parameters: [ {:.6f} ]\n",
      fmt::join(vPinHoleDistorsion1_, " ")
    );
  }

  if (sensor_ == System::STEREO || sensor_ == System::IMU_STEREO) {
    output += fmt::format(
      "- Camera 2 parameters ({}): [ {:.6f} ]\n",
      (cameraType_ == PinHole || cameraType_ == Rectified) ? "Pinhole" : "Kannala-Brandt",
      fmt::join(originalCalib2_->parameters(), " ")
    );

    if (!vPinHoleDistorsion2_.empty()) {
      output += fmt::format(
        "- Camera 2 distortion parameters: [ {:.6f} ]\n",
        fmt::join(vPinHoleDistorsion2_, " ")
      );
    }
  }

  output += fmt::format(
    "- Original image size: [ {}, {} ]\n",
    originalImSize_.width,
    originalImSize_.height
  );
  output += fmt::format( // clang-format off
    "- Current image size: [ {}, {} ]\n",
    newImSize_.width,
    newImSize_.height
  ); // clang-format on

  if (bNeedToRectify_) {
    output += fmt::format(
      "- Camera 1 parameters after rectification: [ {:.6f} ]\n",
      fmt::join(calibration1_->parameters(), " ")
    );
  } else if (bNeedToResize1_) {
    output += fmt::format(
      "- Camera 1 parameters after resize: [ {:.6f} ]\n",
      fmt::join(calibration1_->parameters(), " ")
    );

    if ((sensor_ == System::STEREO || sensor_ == System::IMU_STEREO) && cameraType_ == KannalaBrandt) {
      output += fmt::format(
        "- Camera 2 parameters after resize: [ {:.6f} ]\n",
        fmt::join(calibration2_->parameters(), " ")
      );
    }
  }

  output += fmt::format("- Sequence FPS: {}\n", fps_);

  if (sensor_ == System::STEREO || sensor_ == System::IMU_STEREO) {
    output += fmt::format("- Stereo baseline: {:.6f}\n", b_);
    output += fmt::format("- Stereo depth threshold: {:.6f}\n", thDepth_);

    if (cameraType_ == KannalaBrandt) {
      auto vOverlapping1 = static_cast<KannalaBrandt8*>(calibration1_)->mvLappingArea;
      auto vOverlapping2 = static_cast<KannalaBrandt8*>(calibration2_)->mvLappingArea;

      output += fmt::format(
        "- Camera 1 overlapping area: [ {}, {} ]\n",
        vOverlapping1[0],
        vOverlapping1[1]
      );
      output += fmt::format(
        "- Camera 2 overlapping area: [ {}, {} ]\n",
        vOverlapping2[0],
        vOverlapping2[1]
      );
    }
  }

  if (sensor_ == System::IMU_MONOCULAR || sensor_ == System::IMU_STEREO || sensor_ == System::IMU_RGBD) {
    // clang-format off
    output += fmt::format("- Gyro noise: {:.6f}\n"         , noiseGyro_   );
    output += fmt::format("- Accelerometer noise: {:.6f}\n", noiseAcc_    );
    output += fmt::format("- Gyro walk: {:.6f}\n"          , gyroWalk_    );
    output += fmt::format("- Accelerometer walk: {:.6f}\n" , accWalk_     );
    output += fmt::format("- IMU frequency: {:.6f}\n"      , imuFrequency_);
    // clang-format on
  }

  if (sensor_ == System::RGBD || sensor_ == System::IMU_RGBD) {
    output += fmt::format("- RGB-D depth map factor: {}\n", depthMapFactor_);
  }

  // clang-format off
  output += fmt::format("- Features per image: {}\n"    , nFeatures_  );
  output += fmt::format("- ORB scale factor: {:.6f}\n"  , scaleFactor_);
  output += fmt::format("- ORB number of scales: {}\n"  , nLevels_    );
  output += fmt::format("- Initial FAST threshold: {}\n", initThFAST_ );
  output += fmt::format("- Min FAST threshold: {}\n"    , minThFAST_  );
  // clang-format on

  return output;
}

}; // namespace ORB_SLAM3

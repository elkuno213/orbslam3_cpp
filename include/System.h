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

#ifndef SYSTEM_H
#define SYSTEM_H

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>
#include <spdlog/spdlog.h>
#include "ImuTypes.h"
#include "ORBVocabulary.h"

namespace ORB_SLAM3 {

class Atlas;
class FrameDrawer;
class KeyFrameDatabase;
class LocalMapping;
class LoopClosing;
class Map;
class MapDrawer;
class MapPoint;
class Settings;
class Tracking;
class Viewer;

class System {
public:
  // Input sensor
  enum eSensor {
    MONOCULAR     = 0,
    STEREO        = 1,
    RGBD          = 2,
    IMU_MONOCULAR = 3,
    IMU_STEREO    = 4,
    IMU_RGBD      = 5,
  };

  // File type
  enum FileType {
    TEXT_FILE   = 0,
    BINARY_FILE = 1,
  };

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
  System(
    const std::string& strVocFile,
    const std::string& strSettingsFile,
    const eSensor      sensor,
    const bool         bUseViewer  = true,
    const int          initFr      = 0,
    const std::string& strSequence = std::string()
  );

  // Proccess the given stereo frame. Images must be synchronized and rectified.
  // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
  // Returns the camera pose (empty if tracking fails).
  Sophus::SE3f TrackStereo(
    const cv::Mat&                 imLeft,
    const cv::Mat&                 imRight,
    const double&                  timestamp,
    const std::vector<IMU::Point>& vImuMeas = std::vector<IMU::Point>(),
    std::string                    filename = ""
  );

  // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
  // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
  // Input depthmap: Float (CV_32F).
  // Returns the camera pose (empty if tracking fails).
  Sophus::SE3f TrackRGBD(
    const cv::Mat&                 im,
    const cv::Mat&                 depthmap,
    const double&                  timestamp,
    const std::vector<IMU::Point>& vImuMeas = std::vector<IMU::Point>(),
    std::string                    filename = ""
  );

  // Proccess the given monocular frame and optionally imu data
  // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
  // Returns the camera pose (empty if tracking fails).
  Sophus::SE3f TrackMonocular(
    const cv::Mat&                 im,
    const double&                  timestamp,
    const std::vector<IMU::Point>& vImuMeas = std::vector<IMU::Point>(),
    std::string                    filename = ""
  );

  // This stops local mapping thread (map building) and performs only camera tracking.
  void ActivateLocalizationMode();
  // This resumes local mapping thread and performs SLAM again.
  void DeactivateLocalizationMode();

  // Returns true if there have been a big map change (loop closure, global BA)
  // since last call to this function
  bool MapChanged();

  // Reset the system (clear Atlas or the active map)
  void Reset();
  void ResetActiveMap();

  // All threads will be requested to finish.
  // It waits until all threads have finished.
  // This function must be called before saving the trajectory.
  void Shutdown();
  bool isShutDown();

  // Save camera trajectory in the TUM RGB-D dataset format.
  // Only for stereo and RGB-D. This method does not work for monocular.
  // Call first Shutdown()
  // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
  void SaveTrajectoryTUM(const std::string& filename);

  // Save keyframe poses in the TUM RGB-D dataset format.
  // This method works for all sensor input.
  // Call first Shutdown()
  // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
  void SaveKeyFrameTrajectoryTUM(const std::string& filename);

  void SaveTrajectoryEuRoC(const std::string& filename);
  void SaveKeyFrameTrajectoryEuRoC(const std::string& filename);

  void SaveTrajectoryEuRoC(const std::string& filename, Map* pMap);
  void SaveKeyFrameTrajectoryEuRoC(const std::string& filename, Map* pMap);

  // Save data used for initialization debug
  void SaveDebugData(const int& iniIdx);

  // Save camera trajectory in the KITTI dataset format.
  // Only for stereo and RGB-D. This method does not work for monocular.
  // Call first Shutdown()
  // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
  void SaveTrajectoryKITTI(const std::string& filename);

  // TODO: Save/Load functions
  // SaveMap(const std::string &filename);
  // LoadMap(const std::string &filename);

  // Information from most recent processed frame
  // You can call this right after TrackMonocular (or stereo or RGBD)
  int                       GetTrackingState();
  std::vector<MapPoint*>    GetTrackedMapPoints();
  std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

  // For debugging
  double GetTimeFromIMUInit();
  bool   isLost();
  bool   isFinished();

  void ChangeDataset();

  float GetImageScale();

#ifdef REGISTER_TIMES
  void InsertRectTime(double& time);
  void InsertResizeTime(double& time);
  void InsertTrackTime(double& time);
#endif

private:
  void SaveAtlas(int type);
  bool LoadAtlas(int type);

  std::string CalculateCheckSum(std::string filename, int type);

  // Input sensor
  eSensor mSensor;

  // ORB vocabulary used for place recognition and feature matching.
  ORBVocabulary* mpVocabulary;

  // KeyFrame database for place recognition (relocalization and loop detection).
  KeyFrameDatabase* mpKeyFrameDatabase;

  // Map structure that stores the pointers to all KeyFrames and MapPoints.
  // Map* mpMap;
  Atlas* mpAtlas;

  // Tracker. It receives a frame and computes the associated camera pose.
  // It also decides when to insert a new keyframe, create some new MapPoints and
  // performs relocalization if tracking fails.
  Tracking* mpTracker;

  // Local Mapper. It manages the local map and performs local bundle adjustment.
  LocalMapping* mpLocalMapper;

  // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
  // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
  LoopClosing* mpLoopCloser;

  // The viewer draws the map and the current camera pose. It uses Pangolin.
  Viewer* mpViewer;

  FrameDrawer* mpFrameDrawer;
  MapDrawer*   mpMapDrawer;

  // System threads: Local Mapping, Loop Closing, Viewer.
  // The Tracking thread "lives" in the main execution thread that creates the System object.
  std::thread* mptLocalMapping;
  std::thread* mptLoopClosing;
  std::thread* mptViewer;

  // Reset flag
  std::mutex mMutexReset;
  bool       mbReset;
  bool       mbResetActiveMap;

  // Change mode flags
  std::mutex mMutexMode;
  bool       mbActivateLocalizationMode;
  bool       mbDeactivateLocalizationMode;

  // Shutdown flag
  bool mbShutDown;

  // Tracking state
  int                       mTrackingState;
  std::vector<MapPoint*>    mTrackedMapPoints;
  std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
  std::mutex                mMutexState;

  //
  std::string mStrLoadAtlasFromFile;
  std::string mStrSaveAtlasToFile;

  std::string mStrVocabularyFilePath;

  Settings* settings_;

  std::shared_ptr<spdlog::logger> _logger;
};

} // namespace ORB_SLAM3

#endif // SYSTEM_H

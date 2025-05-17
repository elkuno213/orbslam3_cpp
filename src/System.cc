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

#include "System.h"
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <ios>
#include <iostream>
#include <thread>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <opencv2/imgproc.hpp>
#include <openssl/md5.h>
#include "Atlas.h"
#include "FrameDrawer.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "LoggingUtils.h"
#include "LoopClosing.h"
#include "Map.h"
#include "MapDrawer.h"
#include "MapPoint.h"
#include "Settings.h"
#include "Tracking.h"
#include "Viewer.h"

namespace ORB_SLAM3 {

System::System(
  const std::string& strVocFile,
  const std::string& strSettingsFile,
  const Sensor       sensor,
  const bool         bUseViewer,
  const int          initFr,
  const std::string& strSequence
)
  : mSensor(sensor)
  , mpViewer(static_cast<Viewer*>(NULL))
  , mbReset(false)
  , mbResetActiveMap(false)
  , mbActivateLocalizationMode(false)
  , mbDeactivateLocalizationMode(false)
  , mbShutDown(false)
  , _logger(logging::CreateModuleLogger("System")) {
  _logger->info(R"(
    ORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
    ORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
    This program comes with ABSOLUTELY NO WARRANTY;
    This is free software, and you are welcome to redistribute it
    under certain conditions. See LICENSE.txt.
  )");

  if (mSensor == Sensor::Monocular) {
    _logger->info("Input sensor: Monocular");
  } else if (mSensor == Sensor::Stereo) {
    _logger->info("Input sensor: Stereo");
  } else if (mSensor == Sensor::RGBD) {
    _logger->info("Input sensor: RGB-D");
  } else if (mSensor == Sensor::InertialMonocular) {
    _logger->info("Input sensor: Monocular-Inertial");
  } else if (mSensor == Sensor::InertialStereo) {
    _logger->info("Input sensor: Stereo-Inertial");
  } else if (mSensor == Sensor::InertialRGBD) {
    _logger->info("Input sensor: RGB-D Inertial");
  }

  // Check settings file
  cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    throw std::runtime_error(fmt::format("Failed to open settings file {}", strSettingsFile));
  }

  cv::FileNode node = fsSettings["File.version"];
  if (!node.empty() && node.isString() && node.string() == "1.0") {
    settings_ = new Settings(strSettingsFile, mSensor);

    mStrLoadAtlasFromFile = settings_->atlasLoadFile();
    mStrSaveAtlasToFile   = settings_->atlasSaveFile();

    _logger->info(settings_->Str());
  } else {
    settings_         = nullptr;
    cv::FileNode node = fsSettings["System.LoadAtlasFromFile"];
    if (!node.empty() && node.isString()) {
      mStrLoadAtlasFromFile = (std::string)node;
    }

    node = fsSettings["System.SaveAtlasToFile"];
    if (!node.empty() && node.isString()) {
      mStrSaveAtlasToFile = (std::string)node;
    }
  }

  node          = fsSettings["loopClosing"];
  bool activeLC = true;
  if (!node.empty()) {
    activeLC = static_cast<int>(fsSettings["loopClosing"]) != 0;
  }

  mStrVocabularyFilePath = strVocFile;

  bool loadedAtlas = false;

  if (mStrLoadAtlasFromFile.empty()) {
    // Load ORB Vocabulary
    _logger->info("Loading ORB vocabulary {}...", strVocFile);

    mpVocabulary  = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if (!bVocLoad) {
      throw std::runtime_error(fmt::format("Failed to open vocabulary file {}", strVocFile));
    }
    _logger->info("ORB vocabulary loaded");

    // Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    // Create the Atlas
    _logger->info("Initializing Atlas from scratch...");
    mpAtlas = new Atlas(0);
  } else {
    // Load ORB Vocabulary
    _logger->info("Loading ORB vocabulary {}...", strVocFile);

    mpVocabulary  = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if (!bVocLoad) {
      throw std::runtime_error(fmt::format("Failed to open vocabulary file {}", strVocFile));
    }
    _logger->info("ORB vocabulary loaded");

    // Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    // Load the file with an earlier session
    // clock_t start = clock();
    _logger->info("Loading Atlas from file {}", mStrLoadAtlasFromFile);
    bool isRead = LoadAtlas(FileType::BINARY_FILE);

    if (!isRead) {
      throw std::runtime_error(
        "Faild to load Atlas, try with another session file or vocabulary file"
      );
    }

    // mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);
    // _logger->info(
    //   "KF in DB: {} ; words: {}",
    //   mpKeyFrameDatabase->mnNumKFs,
    //   mpKeyFrameDatabase->mnNumWords
    // );

    loadedAtlas = true;

    mpAtlas->CreateNewMap();

    // clock_t timeElapsed = clock() - start;
    // unsigned msElapsed = timeElapsed / (CLOCKS_PER_SEC / 1000);
    // _logger->info("Binary file read in {} ms", msElapsed);

    // usleep(10*1000*1000);
  }

  if (mSensor == Sensor::InertialStereo || mSensor == Sensor::InertialMonocular || mSensor == Sensor::InertialRGBD) {
    mpAtlas->SetInertialSensor();
  }

  // Create Drawers. These are used by the Viewer
  mpFrameDrawer = new FrameDrawer(mpAtlas);
  mpMapDrawer   = new MapDrawer(mpAtlas, strSettingsFile, settings_);

  // Initialize the Tracking thread
  //(it will live in the main thread of execution, the one that called this constructor)
  _logger->info("Initializing tracking thread...");
  mpTracker = new Tracking(
    this,
    mpVocabulary,
    mpFrameDrawer,
    mpMapDrawer,
    mpAtlas,
    mpKeyFrameDatabase,
    strSettingsFile,
    mSensor,
    settings_,
    strSequence
  );

  // Initialize the Local Mapping thread and launch
  _logger->info("Initializing local mapping thread...");
  mpLocalMapper = new LocalMapping(
    this,
    mpAtlas,
    mSensor == Sensor::Monocular || mSensor == Sensor::InertialMonocular,
    mSensor == Sensor::InertialMonocular || mSensor == Sensor::InertialStereo
      || mSensor == Sensor::InertialRGBD,
    strSequence
  );
  mptLocalMapping        = new std::thread(&ORB_SLAM3::LocalMapping::Run, mpLocalMapper);
  mpLocalMapper->mInitFr = initFr;
  if (settings_) {
    mpLocalMapper->mThFarPoints = settings_->thFarPoints();
  } else {
    mpLocalMapper->mThFarPoints = fsSettings["thFarPoints"];
  }
  if (mpLocalMapper->mThFarPoints != 0) {
    _logger->info(
      "Discard points further than {} m from current camera",
      mpLocalMapper->mThFarPoints
    );
    mpLocalMapper->mbFarPoints = true;
  } else {
    mpLocalMapper->mbFarPoints = false;
  }

  // Initialize the Loop Closing thread and launch
  // mSensor!=Sensor::Monocular && mSensor!=Sensor::InertialMonocular
  _logger->info("Initializing loop closing thread...");
  mpLoopCloser = new LoopClosing(
    mpAtlas,
    mpKeyFrameDatabase,
    mpVocabulary,
    mSensor != Sensor::Monocular,
    activeLC
  ); // mSensor!=Sensor::Monocular);
  mptLoopClosing = new std::thread(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser);

  // Set pointers between threads
  mpTracker->SetLocalMapper(mpLocalMapper);
  mpTracker->SetLoopClosing(mpLoopCloser);

  mpLocalMapper->SetTracker(mpTracker);
  mpLocalMapper->SetLoopCloser(mpLoopCloser);

  mpLoopCloser->SetTracker(mpTracker);
  mpLoopCloser->SetLocalMapper(mpLocalMapper);

  // usleep(10*1000*1000);

  // Initialize the Viewer thread and launch
  if (bUseViewer) {
    // if(false) // TODO
    _logger->info("Initializing viewer thread...");
    mpViewer  = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile, settings_);
    mptViewer = new std::thread(&Viewer::Run, mpViewer);
    mpTracker->SetViewer(mpViewer);
    mpLoopCloser->mpViewer = mpViewer;
    mpViewer->both         = mpFrameDrawer->both;
  }
}

Sophus::SE3f System::TrackStereo(
  const cv::Mat&                 imLeft,
  const cv::Mat&                 imRight,
  const double&                  timestamp,
  const std::vector<IMU::Point>& vImuMeas,
  std::string                    filename
) {
  if (mSensor != Sensor::Stereo && mSensor != Sensor::InertialStereo) {
    throw std::runtime_error("Invalid sensor type, it should be Stereo or Stereo-Inertial");
  }

  cv::Mat imLeftToFeed, imRightToFeed;
  if (settings_ && settings_->needToRectify()) {
    _logger->debug("Rectifying both images...");

    cv::Mat M1l = settings_->M1l();
    cv::Mat M2l = settings_->M2l();
    cv::Mat M1r = settings_->M1r();
    cv::Mat M2r = settings_->M2r();

    cv::remap(imLeft, imLeftToFeed, M1l, M2l, cv::INTER_LINEAR);
    cv::remap(imRight, imRightToFeed, M1r, M2r, cv::INTER_LINEAR);
  } else if (settings_ && settings_->needToResize()) {
    _logger->debug("Resizing both images...");

    cv::resize(imLeft, imLeftToFeed, settings_->newImSize());
    cv::resize(imRight, imRightToFeed, settings_->newImSize());
  } else {
    _logger->debug("Cloning both image...");
    imLeftToFeed  = imLeft.clone();
    imRightToFeed = imRight.clone();
  }

  // Check mode change
  {
    std::unique_lock<std::mutex> lock(mMutexMode);
    if (mbActivateLocalizationMode) {
      _logger->debug("Localization mode activation requested");
      _logger->debug("Requesting local mapper to stop...");
      mpLocalMapper->RequestStop();
      while (!mpLocalMapper->isStopped()) {
        usleep(1000);
      }

      _logger->debug("Local mapper stopped. Switching to localization-only mode");
      mpTracker->InformOnlyTracking(true);
      mbActivateLocalizationMode = false;
      _logger->info("Localization-only mode activated");
    }
    if (mbDeactivateLocalizationMode) {
      _logger->debug("Localization mode deactivation requested");
      _logger->debug("Releasing local mapper...");
      mpTracker->InformOnlyTracking(false);
      mpLocalMapper->Release();
      mbDeactivateLocalizationMode = false;
      _logger->info("Localization-only mode deactivated. Resuming full SLAM mode");
    }
  }

  // Check reset
  {
    std::unique_lock<std::mutex> lock(mMutexReset);
    if (mbReset) {
      mpTracker->Reset();
      mbReset          = false;
      mbResetActiveMap = false;
      _logger->info("Full system reset completed");
    } else if (mbResetActiveMap) {
      mpTracker->ResetActiveMap();
      mbResetActiveMap = false;
      _logger->info("Active map reset completed");
    }
  }

  if (mSensor == Sensor::InertialStereo) {
    _logger->debug("Grabbing IMU data...");
    for (std::size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++) {
      mpTracker->GrabImuData(vImuMeas[i_imu]);
    }
  }

  _logger->debug("Grabbing Stereo images...");
  Sophus::SE3f Tcw = mpTracker->GrabImageStereo(imLeftToFeed, imRightToFeed, timestamp, filename);

  std::unique_lock<std::mutex> lock2(mMutexState);
  mTrackingState      = mpTracker->mState;
  mTrackedMapPoints   = mpTracker->mCurrentFrame.mvpMapPoints;
  mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
  _logger->debug(
    "Tracking states updated: state {} | {} map points | {} keypoints",
    mTrackingState,
    mTrackedMapPoints.size(),
    mTrackedKeyPointsUn.size()
  );

  return Tcw;
}

Sophus::SE3f System::TrackRGBD(
  const cv::Mat&                 im,
  const cv::Mat&                 depthmap,
  const double&                  timestamp,
  const std::vector<IMU::Point>& vImuMeas,
  std::string                    filename
) {
  if (mSensor != Sensor::RGBD && mSensor != Sensor::InertialRGBD) {
    throw std::runtime_error("Invalid sensor type, it should be RGB-D or RGB-D Inertial");
  }

  cv::Mat imToFeed      = im.clone();
  cv::Mat imDepthToFeed = depthmap.clone();
  if (settings_ && settings_->needToResize()) {
    _logger->debug("Resizing RGB-D image...");
    cv::Mat resizedIm;
    cv::resize(im, resizedIm, settings_->newImSize());
    imToFeed = resizedIm;
    cv::resize(depthmap, imDepthToFeed, settings_->newImSize());
  }

  // Check mode change
  {
    std::unique_lock<std::mutex> lock(mMutexMode);
    if (mbActivateLocalizationMode) {
      _logger->debug("Localization mode activation requested");
      _logger->debug("Requesting local mapper to stop...");
      mpLocalMapper->RequestStop();
      while (!mpLocalMapper->isStopped()) {
        usleep(1000);
      }

      _logger->debug("Local mapper stopped. Switching to localization-only mode");
      mpTracker->InformOnlyTracking(true);
      mbActivateLocalizationMode = false;
      _logger->info("Localization-only mode activated");
    }
    if (mbDeactivateLocalizationMode) {
      _logger->debug("Localization mode deactivation requested");
      _logger->debug("Releasing local mapper...");
      mpTracker->InformOnlyTracking(false);
      mpLocalMapper->Release();
      mbDeactivateLocalizationMode = false;
      _logger->info("Localization-only mode deactivated. Resuming full SLAM mode");
    }
  }

  // Check reset
  {
    std::unique_lock<std::mutex> lock(mMutexReset);
    if (mbReset) {
      mpTracker->Reset();
      mbReset          = false;
      mbResetActiveMap = false;
      _logger->info("Full system reset completed");
    } else if (mbResetActiveMap) {
      mpTracker->ResetActiveMap();
      mbResetActiveMap = false;
      _logger->info("Active map reset completed");
    }
  }

  if (mSensor == Sensor::InertialRGBD) {
    _logger->debug("Grabbing IMU data...");
    for (std::size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++) {
      mpTracker->GrabImuData(vImuMeas[i_imu]);
    }
  }

  _logger->debug("Grabbing RGB-D image...");
  Sophus::SE3f Tcw = mpTracker->GrabImageRGBD(imToFeed, imDepthToFeed, timestamp, filename);

  std::unique_lock<std::mutex> lock2(mMutexState);
  mTrackingState      = mpTracker->mState;
  mTrackedMapPoints   = mpTracker->mCurrentFrame.mvpMapPoints;
  mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
  _logger->debug(
    "Tracking states updated: state {} | {} map points | {} keypoints",
    mTrackingState,
    mTrackedMapPoints.size(),
    mTrackedKeyPointsUn.size()
  );

  return Tcw;
}

Sophus::SE3f System::TrackMonocular(
  const cv::Mat&                 im,
  const double&                  timestamp,
  const std::vector<IMU::Point>& vImuMeas,
  std::string                    filename
) {
  {
    std::unique_lock<std::mutex> lock(mMutexReset);
    if (mbShutDown) {
      _logger->info("Shutting down system...");
      return Sophus::SE3f();
    }
  }

  if (mSensor != Sensor::Monocular && mSensor != Sensor::InertialMonocular) {
    throw std::runtime_error("Invalid sensor type, it should be Monocular or Monocular-Inertial");
  }

  cv::Mat imToFeed = im.clone();
  if (settings_ && settings_->needToResize()) {
    _logger->debug("Resizing image...");
    cv::Mat resizedIm;
    cv::resize(im, resizedIm, settings_->newImSize());
    imToFeed = resizedIm;
  }

  // Check mode change
  {
    std::unique_lock<std::mutex> lock(mMutexMode);
    if (mbActivateLocalizationMode) {
      _logger->debug("Localization mode activation requested");
      _logger->debug("Requesting local mapper to stop...");
      mpLocalMapper->RequestStop();
      while (!mpLocalMapper->isStopped()) {
        usleep(1000);
      }

      _logger->debug("Local mapper stopped. Switching to localization-only mode");
      mpTracker->InformOnlyTracking(true);
      mbActivateLocalizationMode = false;
      _logger->info("Localization-only mode activated");
    }
    if (mbDeactivateLocalizationMode) {
      _logger->debug("Localization mode deactivation requested");
      _logger->debug("Releasing local mapper...");
      mpTracker->InformOnlyTracking(false);
      mpLocalMapper->Release();
      mbDeactivateLocalizationMode = false;
      _logger->info("Localization-only mode deactivated. Resuming full SLAM mode");
    }
  }

  // Check reset
  {
    std::unique_lock<std::mutex> lock(mMutexReset);
    if (mbReset) {
      mpTracker->Reset();
      mbReset          = false;
      mbResetActiveMap = false;
      _logger->info("Full system reset completed");
    } else if (mbResetActiveMap) {
      mpTracker->ResetActiveMap();
      mbResetActiveMap = false;
      _logger->info("Active map reset completed");
    }
  }

  if (mSensor == Sensor::InertialMonocular) {
    _logger->debug("Grabbing IMU data...");
    for (std::size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++) {
      mpTracker->GrabImuData(vImuMeas[i_imu]);
    }
  }

  _logger->debug("Grabbing image...");
  Sophus::SE3f Tcw = mpTracker->GrabImageMonocular(imToFeed, timestamp, filename);

  std::unique_lock<std::mutex> lock2(mMutexState);
  mTrackingState      = mpTracker->mState;
  mTrackedMapPoints   = mpTracker->mCurrentFrame.mvpMapPoints;
  mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
  _logger->debug(
    "Tracking states updated: state {} | {} map points | {} keypoints",
    mTrackingState,
    mTrackedMapPoints.size(),
    mTrackedKeyPointsUn.size()
  );

  return Tcw;
}

void System::ActivateLocalizationMode() {
  std::unique_lock<std::mutex> lock(mMutexMode);
  mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode() {
  std::unique_lock<std::mutex> lock(mMutexMode);
  mbDeactivateLocalizationMode = true;
}

bool System::MapChanged() {
  static int n    = 0;
  int        curn = mpAtlas->GetLastBigChangeIdx();
  if (n < curn) {
    n = curn;
    return true;
  } else {
    return false;
  }
}

void System::Reset() {
  std::unique_lock<std::mutex> lock(mMutexReset);
  mbReset = true;
}

void System::ResetActiveMap() {
  std::unique_lock<std::mutex> lock(mMutexReset);
  mbResetActiveMap = true;
}

void System::Shutdown() {
  {
    std::unique_lock<std::mutex> lock(mMutexReset);
    mbShutDown = true;
  }

  _logger->info("Shutting down system...");

  mpLocalMapper->RequestFinish();
  mpLoopCloser->RequestFinish();
  /*if(mpViewer)
  {
      mpViewer->RequestFinish();
      while(!mpViewer->isFinished())
          usleep(5000);
  }*/

  // Wait until all thread have effectively stopped
  /*while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() ||
  mpLoopCloser->isRunningGBA())
  {
      if(!mpLocalMapper->isFinished())
          _logger->info("mpLocalMapper is not finished");*/
  /*if(!mpLoopCloser->isFinished())
      _logger->info("mpLocalMapper is not finished");
  if(mpLoopCloser->isRunningGBA()){
      _logger->info("mpLoopCloser is running GBA, break anyway...");
      break;
  }*/
  /*usleep(5000);
}*/

  if (!mStrSaveAtlasToFile.empty()) {
    _logger->info("Saving Atlas to {}", mStrSaveAtlasToFile);
    SaveAtlas(FileType::BINARY_FILE);
  }

  /*if(mpViewer)
      pangolin::BindToContext("ORB-SLAM2: Map Viewer");*/

#ifdef REGISTER_TIMES
  mpTracker->PrintTimeStats();
#endif

  _logger->info("System stopped");
}

bool System::isShutDown() {
  std::unique_lock<std::mutex> lock(mMutexReset);
  return mbShutDown;
}

void System::SaveTrajectoryTUM(const std::string& filename) {
  if (mSensor == Sensor::Monocular) {
    _logger->error("SaveTrajectoryTUM cannot be used for Monocular sensor");
    return;
  }

  _logger->info("Saving camera trajectory to {}...", filename);

  std::vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
  std::sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  Sophus::SE3f Two = vpKFs[0]->GetPoseInverse();

  std::ofstream f;
  f.open(filename.c_str());
  f << std::fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose
  // graph). We need to get first the keyframe pose and then concatenate the relative
  // transformation. Frames not localized (tracking failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
  // which is true when tracking failed (lbL).
  std::list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
  std::list<double>::iterator               lT   = mpTracker->mlFrameTimes.begin();
  std::list<bool>::iterator                 lbL  = mpTracker->mlbLost.begin();
  for (std::list<Sophus::SE3f>::iterator lit  = mpTracker->mlRelativeFramePoses.begin(),
                                         lend = mpTracker->mlRelativeFramePoses.end();
       lit != lend;
       lit++, lRit++, lT++, lbL++) {
    if (*lbL) {
      continue;
    }

    KeyFrame* pKF = *lRit;

    Sophus::SE3f Trw;

    // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
    while (pKF->isBad()) {
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
    }

    Trw = Trw * pKF->GetPose() * Two;

    Sophus::SE3f Tcw = (*lit) * Trw;
    Sophus::SE3f Twc = Tcw.inverse();

    Eigen::Vector3f    twc = Twc.translation();
    Eigen::Quaternionf q   = Twc.unit_quaternion();

    f << std::setprecision(6) << *lT << " " << std::setprecision(9) << twc(0) << " " << twc(1)
      << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }
  f.close();
  _logger->info("Trajectory saved at {}", filename);
}

void System::SaveKeyFrameTrajectoryTUM(const std::string& filename) {
  _logger->info("Saving keyframe trajectory to {} ...", filename);

  std::vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
  std::sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  std::ofstream f;
  f.open(filename.c_str());
  f << std::fixed;

  for (std::size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame* pKF = vpKFs[i];

    // pKF->SetPose(pKF->GetPose()*Two);

    // If the reference keyframe was culled, skip.
    if (pKF->isBad()) {
      continue;
    }

    Sophus::SE3f       Twc = pKF->GetPoseInverse();
    Eigen::Quaternionf q   = Twc.unit_quaternion();
    Eigen::Vector3f    t   = Twc.translation();
    f << std::setprecision(6) << pKF->mTimeStamp << std::setprecision(7) << " " << t(0) << " "
      << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
      << std::endl;
  }

  f.close();
  _logger->info("Trajectory saved at {}", filename);
}

void System::SaveTrajectoryEuRoC(const std::string& filename) {
  /*if (mSensor == Sensor::Monocular) {
    _logger->error("SaveTrajectoryEuRoC cannot be used for monocular");
    return;
  }*/

  _logger->info("Saving trajectory to {}...", filename);

  std::vector<Map*> vpMaps    = mpAtlas->GetAllMaps();
  int               numMaxKFs = 0;
  Map*              pBiggerMap;

  _logger->info("There are {} maps in the atlas", vpMaps.size());
  for (Map* pMap : vpMaps) {
    _logger->info("Map {} has {} KFs", pMap->GetId(), pMap->GetAllKeyFrames().size());
    if (pMap->GetAllKeyFrames().size() > numMaxKFs) {
      numMaxKFs  = pMap->GetAllKeyFrames().size();
      pBiggerMap = pMap;
    }
  }

  std::vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
  std::sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  Sophus::SE3f Twb; // Can be word to cam0 or world to b depending on IMU or not.
  if (mSensor == Sensor::InertialMonocular || mSensor == Sensor::InertialStereo || mSensor == Sensor::InertialRGBD) {
    Twb = vpKFs[0]->GetImuPose();
  } else {
    Twb = vpKFs[0]->GetPoseInverse();
  }

  std::ofstream f;
  f.open(filename.c_str());
  f << std::fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose
  // graph). We need to get first the keyframe pose and then concatenate the relative
  // transformation. Frames not localized (tracking failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
  // which is true when tracking failed (lbL).
  std::list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
  std::list<double>::iterator               lT   = mpTracker->mlFrameTimes.begin();
  std::list<bool>::iterator                 lbL  = mpTracker->mlbLost.begin();

  // _logger->info("Size of mlpReferences: {}", mpTracker->mlpReferences.size());
  // _logger->info("Size of mlRelativeFramePoses: {}", mpTracker->mlRelativeFramePoses.size());
  // _logger->info("Size of mpTracker->mlFrameTimes: {}", mpTracker->mlFrameTimes.size());
  // _logger->info("Size of mpTracker->mlbLost: {}", mpTracker->mlbLost.size());

  for (auto lit  = mpTracker->mlRelativeFramePoses.begin(),
            lend = mpTracker->mlRelativeFramePoses.end();
       lit != lend;
       lit++, lRit++, lT++, lbL++) {
    if (*lbL) {
      continue;
    }

    KeyFrame* pKF = *lRit;

    Sophus::SE3f Trw;

    // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
    if (!pKF) {
      continue;
    }

    // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
    while (pKF->isBad()) {
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
    }

    if (!pKF || pKF->GetMap() != pBiggerMap) {
      _logger->warn("Parent KeyFrame is from another map, skipping...");
      continue;
    }

    Trw = Trw * pKF->GetPose() * Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

    if (mSensor == Sensor::InertialMonocular || mSensor == Sensor::InertialStereo || mSensor == Sensor::InertialRGBD) {
      Sophus::SE3f       Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
      Eigen::Quaternionf q   = Twb.unit_quaternion();
      Eigen::Vector3f    twb = Twb.translation();
      f << std::setprecision(6) << 1e9 * (*lT) << " " << std::setprecision(9) << twb(0) << " "
        << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
        << std::endl;
    } else {
      Sophus::SE3f       Twc = ((*lit) * Trw).inverse();
      Eigen::Quaternionf q   = Twc.unit_quaternion();
      Eigen::Vector3f    twc = Twc.translation();
      f << std::setprecision(6) << 1e9 * (*lT) << " " << std::setprecision(9) << twc(0) << " "
        << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
        << std::endl;
    }
  }
  f.close();
  _logger->info("Trajectory saved at {}", filename);
}

void System::SaveTrajectoryEuRoC(const std::string& filename, Map* pMap) {
  /*if (mSensor == Sensor::Monocular) {
    _logger->error("SaveTrajectoryEuRoC cannot be used for monocular");
    return;
  }*/

  _logger->info("Saving trajectory of map {} to {}...", pMap->GetId(), filename);

  int numMaxKFs = 0;

  std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
  std::sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  Sophus::SE3f Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
  if (mSensor == Sensor::InertialMonocular || mSensor == Sensor::InertialStereo || mSensor == Sensor::InertialRGBD) {
    Twb = vpKFs[0]->GetImuPose();
  } else {
    Twb = vpKFs[0]->GetPoseInverse();
  }

  std::ofstream f;
  f.open(filename.c_str());
  f << std::fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose
  // graph). We need to get first the keyframe pose and then concatenate the relative
  // transformation. Frames not localized (tracking failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
  // which is true when tracking failed (lbL).
  std::list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
  std::list<double>::iterator               lT   = mpTracker->mlFrameTimes.begin();
  std::list<bool>::iterator                 lbL  = mpTracker->mlbLost.begin();

  // _logger->info("Size of mlpReferences: {}", mpTracker->mlpReferences.size());
  // _logger->info("Size of mlRelativeFramePoses: {}", mpTracker->mlRelativeFramePoses.size());
  // _logger->info("Size of mpTracker->mlFrameTimes: {}", mpTracker->mlFrameTimes.size());
  // _logger->info("Size of mpTracker->mlbLost: {}", mpTracker->mlbLost.size());

  for (auto lit  = mpTracker->mlRelativeFramePoses.begin(),
            lend = mpTracker->mlRelativeFramePoses.end();
       lit != lend;
       lit++, lRit++, lT++, lbL++) {
    if (*lbL) {
      continue;
    }

    KeyFrame* pKF = *lRit;

    Sophus::SE3f Trw;

    if (!pKF) {
      continue;
    }

    // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
    while (pKF->isBad()) {
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
    }

    if (!pKF || pKF->GetMap() != pMap) {
      _logger->warn("Parent KF is from another map, skipping...");
      continue;
    }

    Trw = Trw * pKF->GetPose() * Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

    if (mSensor == Sensor::InertialMonocular || mSensor == Sensor::InertialStereo || mSensor == Sensor::InertialRGBD) {
      Sophus::SE3f       Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
      Eigen::Quaternionf q   = Twb.unit_quaternion();
      Eigen::Vector3f    twb = Twb.translation();
      f << std::setprecision(6) << 1e9 * (*lT) << " " << std::setprecision(9) << twb(0) << " "
        << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
        << std::endl;
    } else {
      Sophus::SE3f       Twc = ((*lit) * Trw).inverse();
      Eigen::Quaternionf q   = Twc.unit_quaternion();
      Eigen::Vector3f    twc = Twc.translation();
      f << std::setprecision(6) << 1e9 * (*lT) << " " << std::setprecision(9) << twc(0) << " "
        << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
        << std::endl;
    }
  }
  f.close();
  _logger->info("Trajectory saved at {}", filename);
}

/*void System::SaveTrajectoryEuRoC(const std::string &filename) {

    std::cout << std::endl << "Saving trajectory to " << filename << " ..." << std::endl;
    if(mSensor==Sensor::Monocular)
    {
        std::cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << std::endl;
        return;
    }

    std::vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    std::vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    std::sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
    if (mSensor==Sensor::InertialMonocular || mSensor==Sensor::InertialStereo ||
mSensor==Sensor::InertialRGBD) Twb = vpKFs[0]->GetImuPose_(); else Twb =
vpKFs[0]->GetPoseInverse_();

    std::ofstream f;
    f.open(filename.c_str());
    // std::cout << "file open" << std::endl;
    f << std::fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose
graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    std::list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    std::list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    std::list<bool>::iterator lbL = mpTracker->mlbLost.begin();

    // std::cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << std::endl;
    // std::cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() <<
std::endl;
    // std::cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << std::endl;
    // std::cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << std::endl;


    for(std::list<Sophus::SE3f>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        // std::cout << "1" << std::endl;
        if(*lbL)
            continue;


        KeyFrame* pKF = *lRit;
        // std::cout << "KF: " << pKF->mnId << std::endl;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable
keyframe. if (!pKF) continue;

        // std::cout << "2.5" << std::endl;

        while(pKF->isBad())
        {
            // std::cout << " 2.bad" << std::endl;
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
            // std::cout << "--Parent KF: " << pKF->mnId << std::endl;
        }

        if(!pKF || pKF->GetMap() != pBiggerMap)
        {
            // std::cout << "--Parent KF is from another map" << std::endl;
            continue;
        }

        // std::cout << "3" << std::endl;

        Trw = Trw * pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        // std::cout << "4" << std::endl;


        if (mSensor == Sensor::InertialMonocular || mSensor == Sensor::InertialStereo ||
mSensor==Sensor::InertialRGBD)
        {
            Sophus::SE3f Tbw = pKF->mImuCalib.Tbc_ * (*lit) * Trw;
            Sophus::SE3f Twb = Tbw.inverse();

            Eigen::Vector3f twb = Twb.translation();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            f << std::setprecision(6) << 1e9*(*lT) << " " <<  std::setprecision(9) << twb(0) << " "
<< twb(1)
<< " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        }
        else
        {
            Sophus::SE3f Tcw = (*lit) * Trw;
            Sophus::SE3f Twc = Tcw.inverse();

            Eigen::Vector3f twc = Twc.translation();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            f << std::setprecision(6) << 1e9*(*lT) << " " <<  std::setprecision(9) << twc(0) << " "
<< twc(1)
<< " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        }

        // std::cout << "5" << std::endl;
    }
    // std::cout << "end saving trajectory" << std::endl;
    f.close();
    std::cout << std::endl << "End of saving trajectory to " << filename << " ..." << std::endl;
}*/

/*void System::SaveKeyFrameTrajectoryEuRoC_old(const std::string &filename)
{
    std::cout << std::endl << "Saving keyframe trajectory to " << filename << " ..." << std::endl;

    std::vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    std::vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    std::sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

    for(std::size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;
        if (mSensor == Sensor::InertialMonocular || mSensor == Sensor::InertialStereo ||
mSensor==Sensor::InertialRGBD)
        {
            cv::Mat R = pKF->GetImuRotation().t();
            std::vector<float> q = Converter::toQuaternion(R);
            cv::Mat twb = pKF->GetImuPosition();
            f << std::setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<  std::setprecision(9) <<
twb.at<float>(0) << " " << twb.at<float>(1) << " " << twb.at<float>(2) << " " << q[0] << " " << q[1]
<< " " << q[2] << " " << q[3] << std::endl;

        }
        else
        {
            cv::Mat R = pKF->GetRotation();
            std::vector<float> q = Converter::toQuaternion(R);
            cv::Mat t = pKF->GetCameraCenter();
            f << std::setprecision(6) << 1e9*pKF->mTimeStamp << " " <<  std::setprecision(9) <<
t.at<float>(0)
<< " " << t.at<float>(1) << " " << t.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << "
" << q[3] << std::endl;
        }
    }
    f.close();
}*/

void System::SaveKeyFrameTrajectoryEuRoC(const std::string& filename) {
  _logger->info("Saving keyframe trajectory to {}...", filename);

  std::vector<Map*> vpMaps = mpAtlas->GetAllMaps();
  Map*              pBiggerMap;
  int               numMaxKFs = 0;
  for (Map* pMap : vpMaps) {
    if (pMap && pMap->GetAllKeyFrames().size() > numMaxKFs) {
      numMaxKFs  = pMap->GetAllKeyFrames().size();
      pBiggerMap = pMap;
    }
  }

  if (!pBiggerMap) {
    _logger->warn("There is not a map");
    return;
  }

  std::vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
  std::sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  std::ofstream f;
  f.open(filename.c_str());
  f << std::fixed;

  for (std::size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame* pKF = vpKFs[i];

    // pKF->SetPose(pKF->GetPose()*Two);

    if (!pKF || pKF->isBad()) {
      continue;
    }
    if (mSensor == Sensor::InertialMonocular || mSensor == Sensor::InertialStereo || mSensor == Sensor::InertialRGBD) {
      Sophus::SE3f       Twb = pKF->GetImuPose();
      Eigen::Quaternionf q   = Twb.unit_quaternion();
      Eigen::Vector3f    twb = Twb.translation();
      f << std::setprecision(6) << 1e9 * pKF->mTimeStamp << " " << std::setprecision(9) << twb(0)
        << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " "
        << q.w() << std::endl;
    } else {
      Sophus::SE3f       Twc = pKF->GetPoseInverse();
      Eigen::Quaternionf q   = Twc.unit_quaternion();
      Eigen::Vector3f    t   = Twc.translation();
      f << std::setprecision(6) << 1e9 * pKF->mTimeStamp << " " << std::setprecision(9) << t(0)
        << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " "
        << q.w() << std::endl;
    }
  }
  f.close();
  _logger->info("Trajectory saved at {}", filename);
}

void System::SaveKeyFrameTrajectoryEuRoC(const std::string& filename, Map* pMap) {
  _logger->info("Saving keyframe trajectory of map {} to {}...", pMap->GetId(), filename);

  std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
  std::sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  std::ofstream f;
  f.open(filename.c_str());
  f << std::fixed;

  for (std::size_t i = 0; i < vpKFs.size(); i++) {
    KeyFrame* pKF = vpKFs[i];

    if (!pKF || pKF->isBad()) {
      continue;
    }
    if (mSensor == Sensor::InertialMonocular || mSensor == Sensor::InertialStereo || mSensor == Sensor::InertialRGBD) {
      Sophus::SE3f       Twb = pKF->GetImuPose();
      Eigen::Quaternionf q   = Twb.unit_quaternion();
      Eigen::Vector3f    twb = Twb.translation();
      f << std::setprecision(6) << 1e9 * pKF->mTimeStamp << " " << std::setprecision(9) << twb(0)
        << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " "
        << q.w() << std::endl;

    } else {
      Sophus::SE3f       Twc = pKF->GetPoseInverse();
      Eigen::Quaternionf q   = Twc.unit_quaternion();
      Eigen::Vector3f    t   = Twc.translation();
      f << std::setprecision(6) << 1e9 * pKF->mTimeStamp << " " << std::setprecision(9) << t(0)
        << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " "
        << q.w() << std::endl;
    }
  }
  f.close();
  _logger->info("Trajectory saved at {}", filename);
}

/*void System::SaveTrajectoryKITTI(const std::string &filename)
{
    std::cout << std::endl << "Saving camera trajectory to " << filename << " ..." << std::endl;
    if(mSensor==Sensor::Monocular)
    {
        std::cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << std::endl;
        return;
    }

    std::vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    std::sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose
graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    std::list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    std::list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(std::list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM3::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
            Trw = Trw * Converter::toCvMat(pKF->mTcp.matrix());
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPoseCv() * Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << std::setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " <<
Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " << Rwc.at<float>(1,0) << " " <<
Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " " <<
twc.at<float>(2) << std::endl;
    }
    f.close();
}*/

void System::SaveTrajectoryKITTI(const std::string& filename) {
  if (mSensor == Sensor::Monocular) {
    _logger->error("SaveTrajectoryKITTI cannot be used for Monocular case");
    return;
  }

  _logger->info("Saving camera trajectory to {}...", filename);

  std::vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
  std::sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  Sophus::SE3f Tow = vpKFs[0]->GetPoseInverse();

  std::ofstream f;
  f.open(filename.c_str());
  f << std::fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose
  // graph). We need to get first the keyframe pose and then concatenate the relative
  // transformation. Frames not localized (tracking failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
  // which is true when tracking failed (lbL).
  std::list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
  std::list<double>::iterator               lT   = mpTracker->mlFrameTimes.begin();
  for (std::list<Sophus::SE3f>::iterator lit  = mpTracker->mlRelativeFramePoses.begin(),
                                         lend = mpTracker->mlRelativeFramePoses.end();
       lit != lend;
       lit++, lRit++, lT++) {
    ORB_SLAM3::KeyFrame* pKF = *lRit;

    Sophus::SE3f Trw;

    if (!pKF) {
      continue;
    }

    // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
    while (pKF->isBad()) {
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
    }

    Trw = Trw * pKF->GetPose() * Tow;

    Sophus::SE3f    Tcw = (*lit) * Trw;
    Sophus::SE3f    Twc = Tcw.inverse();
    Eigen::Matrix3f Rwc = Twc.rotationMatrix();
    Eigen::Vector3f twc = Twc.translation();

    f << std::setprecision(9) << Rwc(0, 0) << " " << Rwc(0, 1) << " " << Rwc(0, 2) << " " << twc(0)
      << " " << Rwc(1, 0) << " " << Rwc(1, 1) << " " << Rwc(1, 2) << " " << twc(1) << " "
      << Rwc(2, 0) << " " << Rwc(2, 1) << " " << Rwc(2, 2) << " " << twc(2) << std::endl;
  }
  f.close();
  _logger->info("Trajectory saved at {}", filename);
}

void System::SaveDebugData(const int& initIdx) {
  // 0. Save initialization trajectory
  SaveTrajectoryEuRoC(
    "init_FrameTrajectoy_" + std::to_string(mpLocalMapper->mInitSect) + "_"
    + std::to_string(initIdx) + ".txt"
  );

  // 1. Save scale
  std::ofstream f;
  f.open("init_Scale_" + std::to_string(mpLocalMapper->mInitSect) + ".txt", std::ios_base::app);
  f << std::fixed;
  f << mpLocalMapper->mScale << std::endl;
  f.close();

  // 2. Save gravity direction
  f.open("init_GDir_" + std::to_string(mpLocalMapper->mInitSect) + ".txt", std::ios_base::app);
  f << std::fixed;
  f << mpLocalMapper->mRwg(0, 0) << "," << mpLocalMapper->mRwg(0, 1) << ","
    << mpLocalMapper->mRwg(0, 2) << std::endl;
  f << mpLocalMapper->mRwg(1, 0) << "," << mpLocalMapper->mRwg(1, 1) << ","
    << mpLocalMapper->mRwg(1, 2) << std::endl;
  f << mpLocalMapper->mRwg(2, 0) << "," << mpLocalMapper->mRwg(2, 1) << ","
    << mpLocalMapper->mRwg(2, 2) << std::endl;
  f.close();

  // 3. Save computational cost
  f.open("init_CompCost_" + std::to_string(mpLocalMapper->mInitSect) + ".txt", std::ios_base::app);
  f << std::fixed;
  f << mpLocalMapper->mCostTime << std::endl;
  f.close();

  // 4. Save biases
  f.open("init_Biases_" + std::to_string(mpLocalMapper->mInitSect) + ".txt", std::ios_base::app);
  f << std::fixed;
  f << mpLocalMapper->mbg(0) << "," << mpLocalMapper->mbg(1) << "," << mpLocalMapper->mbg(2)
    << std::endl;
  f << mpLocalMapper->mba(0) << "," << mpLocalMapper->mba(1) << "," << mpLocalMapper->mba(2)
    << std::endl;
  f.close();

  // 5. Save covariance matrix
  f.open(
    "init_CovMatrix_" + std::to_string(mpLocalMapper->mInitSect) + "_" + std::to_string(initIdx)
      + ".txt",
    std::ios_base::app
  );
  f << std::fixed;
  for (int i = 0; i < mpLocalMapper->mcovInertial.rows(); i++) {
    for (int j = 0; j < mpLocalMapper->mcovInertial.cols(); j++) {
      if (j != 0) {
        f << ",";
      }
      f << std::setprecision(15) << mpLocalMapper->mcovInertial(i, j);
    }
    f << std::endl;
  }
  f.close();

  // 6. Save initialization time
  f.open("init_Time_" + std::to_string(mpLocalMapper->mInitSect) + ".txt", std::ios_base::app);
  f << std::fixed;
  f << mpLocalMapper->mInitTime << std::endl;
  f.close();
}

int System::GetTrackingState() {
  std::unique_lock<std::mutex> lock(mMutexState);
  return mTrackingState;
}

std::vector<MapPoint*> System::GetTrackedMapPoints() {
  std::unique_lock<std::mutex> lock(mMutexState);
  return mTrackedMapPoints;
}

std::vector<cv::KeyPoint> System::GetTrackedKeyPointsUn() {
  std::unique_lock<std::mutex> lock(mMutexState);
  return mTrackedKeyPointsUn;
}

double System::GetTimeFromIMUInit() {
  double aux = mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
  if ((aux > 0.) && mpAtlas->isImuInitialized()) {
    return mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
  } else {
    return 0.f;
  }
}

bool System::isLost() {
  if (!mpAtlas->isImuInitialized()) {
    return false;
  } else {
    if ((mpTracker->mState == Tracking::LOST)) { //||(mpTracker->mState==Tracking::RECENTLY_LOST))
      return true;
    } else {
      return false;
    }
  }
}

bool System::isFinished() {
  return (GetTimeFromIMUInit() > 0.1);
}

void System::ChangeDataset() {
  if (mpAtlas->GetCurrentMap()->KeyFramesInMap() < 12) {
    mpTracker->ResetActiveMap();
  } else {
    mpTracker->CreateMapInAtlas();
  }

  mpTracker->NewDataset();
}

float System::GetImageScale() {
  return mpTracker->GetImageScale();
}

#ifdef REGISTER_TIMES
void System::InsertRectTime(double& time) {
  mpTracker->vdRectStereo_ms.push_back(time);
}

void System::InsertResizeTime(double& time) {
  mpTracker->vdResizeImage_ms.push_back(time);
}

void System::InsertTrackTime(double& time) {
  mpTracker->vdTrackTotal_ms.push_back(time);
}
#endif

void System::SaveAtlas(int type) {
  if (!mStrSaveAtlasToFile.empty()) {
    // clock_t start = clock();

    // Save the current session
    mpAtlas->PreSave();

    std::string pathSaveFileName = "./";
    pathSaveFileName             = pathSaveFileName.append(mStrSaveAtlasToFile);
    pathSaveFileName             = pathSaveFileName.append(".osa");

    std::string strVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);
    std::size_t found                 = mStrVocabularyFilePath.find_last_of("/\\");
    std::string strVocabularyName     = mStrVocabularyFilePath.substr(found + 1);

    if (type == TEXT_FILE) { // File text
      _logger->info("Starting to write the save text file {}...", pathSaveFileName);
      std::remove(pathSaveFileName.c_str());
      std::ofstream                 ofs(pathSaveFileName, std::ios::binary);
      boost::archive::text_oarchive oa(ofs);
      oa << strVocabularyName;
      oa << strVocabularyChecksum;
      oa << mpAtlas;
      _logger->info("File saved at {}", pathSaveFileName);
    } else if (type == BINARY_FILE) { // File binary
      _logger->info("Starting to write the save binary file {}...", pathSaveFileName);
      std::remove(pathSaveFileName.c_str());
      std::ofstream                   ofs(pathSaveFileName, std::ios::binary);
      boost::archive::binary_oarchive oa(ofs);
      oa << strVocabularyName;
      oa << strVocabularyChecksum;
      oa << mpAtlas;
      _logger->info("File saved at {}", pathSaveFileName);
    }
  }
}

bool System::LoadAtlas(int type) {
  std::string strFileVoc, strVocChecksum;
  bool        isRead = false;

  std::string pathLoadFileName = "./";
  pathLoadFileName             = pathLoadFileName.append(mStrLoadAtlasFromFile);
  pathLoadFileName             = pathLoadFileName.append(".osa");

  if (type == TEXT_FILE) { // File text
    _logger->info("Starting to read the save text file {}...", pathLoadFileName);
    std::ifstream ifs(pathLoadFileName, std::ios::binary);
    if (!ifs.good()) {
      _logger->error("Failed to load file");
      return false;
    }
    boost::archive::text_iarchive ia(ifs);
    ia >> strFileVoc;
    ia >> strVocChecksum;
    ia >> mpAtlas;
    isRead = true;
    _logger->info("File saved at {}", pathLoadFileName);
  } else if (type == BINARY_FILE) { // File binary
    _logger->info("Starting to read the save binary file {}...", pathLoadFileName);
    std::ifstream ifs(pathLoadFileName, std::ios::binary);
    if (!ifs.good()) {
      _logger->error("Failed to load file");
      return false;
    }
    boost::archive::binary_iarchive ia(ifs);
    ia >> strFileVoc;
    ia >> strVocChecksum;
    ia >> mpAtlas;
    isRead = true;
    _logger->info("File saved at {}", pathLoadFileName);
  }

  if (isRead) {
    // Check if the vocabulary is the same
    std::string strInputVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath, TEXT_FILE);

    if (strInputVocabularyChecksum.compare(strVocChecksum) != 0) {
      _logger->warn("The vocabulary loaded isn't the same as the one used to create the session");
      _logger->warn("Vocabulary file: {}", strFileVoc);
      return false; // Both are differents
    }

    mpAtlas->SetKeyFrameDababase(mpKeyFrameDatabase);
    mpAtlas->SetORBVocabulary(mpVocabulary);
    mpAtlas->PostLoad();

    return true;
  }
  return false;
}

std::string System::CalculateCheckSum(std::string filename, int type) {
  std::string checksum = "";

  unsigned char c[MD5_DIGEST_LENGTH];

  std::ios_base::openmode flags = std::ios::in;
  if (type == BINARY_FILE) { // Binary file
    flags = std::ios::in | std::ios::binary;
  }

  std::ifstream f(filename.c_str(), flags);
  if (!f.is_open()) {
    _logger->error("Unable to open the in file {} for Md5 hash", filename);
    return checksum;
  }

  MD5_CTX md5Context;
  char    buffer[1024];

  MD5_Init(&md5Context);
  while (int count = f.readsome(buffer, sizeof(buffer))) {
    MD5_Update(&md5Context, buffer, count);
  }

  f.close();

  MD5_Final(c, &md5Context);

  for (int i = 0; i < MD5_DIGEST_LENGTH; i++) {
    char aux[10];
    std::sprintf(aux, "%02x", c[i]);
    checksum = checksum + aux;
  }

  return checksum;
}

} // namespace ORB_SLAM3

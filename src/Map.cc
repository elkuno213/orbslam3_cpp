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

#include "Map.h"
#include <ranges>
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "LoggingUtils.h"
#include "MapPoint.h"

namespace ORB_SLAM3 {

long unsigned int Map::nNextId = 0;

Map::Map()
  : mnMaxKFid(0)
  , mnBigChangeIdx(0)
  , mbImuInitialized(false)
  , mnMapChange(0)
  , mpFirstRegionKF(static_cast<KeyFrame*>(NULL))
  , mbFail(false)
  , mIsInUse(false)
  , mHasTumbnail(false)
  , mbBad(false)
  , mnMapChangeNotified(0)
  , mbIsInertial(false)
  , mbIMU_BA1(false)
  , mbIMU_BA2(false)
  , _logger(logging::CreateModuleLogger("Map")) {
  mnId       = nNextId++;
  mThumbnail = static_cast<GLubyte*>(NULL);
}

Map::Map(int initKFid)
  : mnInitKFid(initKFid)
  , mnMaxKFid(initKFid)
  , /*mnLastLoopKFid(initKFid),*/ mnBigChangeIdx(0)
  , mIsInUse(false)
  , mHasTumbnail(false)
  , mbBad(false)
  , mbImuInitialized(false)
  , mpFirstRegionKF(static_cast<KeyFrame*>(NULL))
  , mnMapChange(0)
  , mbFail(false)
  , mnMapChangeNotified(0)
  , mbIsInertial(false)
  , mbIMU_BA1(false)
  , mbIMU_BA2(false)
  , _logger(logging::CreateModuleLogger("Map")) {
  mnId       = nNextId++;
  mThumbnail = static_cast<GLubyte*>(NULL);
}

Map::~Map() {
  _logger->info("Destructing by releasing all resources (map points, keyframes, thumbnail, etc.)");

  // TODO: erase all points from memory
  mspMapPoints.clear();

  // TODO: erase all keyframes from memory
  mspKeyFrames.clear();

  if (mThumbnail) {
    delete mThumbnail;
  }
  mThumbnail = static_cast<GLubyte*>(NULL);

  mvpReferenceMapPoints.clear();
  mvpKeyFrameOrigins.clear();
}

void Map::AddKeyFrame(KeyFrame* pKF) {
  std::unique_lock<std::mutex> lock(mMutexMap);
  if (mspKeyFrames.empty()) {
    _logger->info("First key frame {} | Initial key frame of map {}", pKF->mnId, mnInitKFid);
    mnInitKFid  = pKF->mnId;
    mpKFinitial = pKF;
    mpKFlowerID = pKF;
  }
  mspKeyFrames.insert(pKF);
  _logger->debug("Key frame {} added to map", pKF->mnId);
  if (pKF->mnId > mnMaxKFid) {
    mnMaxKFid = pKF->mnId;
  }
  if (pKF->mnId < mpKFlowerID->mnId) {
    mpKFlowerID = pKF;
  }
}

void Map::AddMapPoint(MapPoint* pMP) {
  std::unique_lock<std::mutex> lock(mMutexMap);
  _logger->debug("Map point {} added to map", pMP->mnId);
  mspMapPoints.insert(pMP);
}

void Map::SetImuInitialized() {
  std::unique_lock<std::mutex> lock(mMutexMap);
  mbImuInitialized = true;
}

bool Map::isImuInitialized() {
  std::unique_lock<std::mutex> lock(mMutexMap);
  return mbImuInitialized;
}

void Map::EraseMapPoint(MapPoint* pMP) {
  std::unique_lock<std::mutex> lock(mMutexMap);
  _logger->debug("Erasing map point {}...", pMP->mnId);
  mspMapPoints.erase(pMP);

  // TODO: This only erase the pointer.
  // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame* pKF) {
  std::unique_lock<std::mutex> lock(mMutexMap);
  _logger->debug("Erasing key frame {}...", pKF->mnId);
  mspKeyFrames.erase(pKF);
  if (mspKeyFrames.size() > 0) {
    if (pKF->mnId == mpKFlowerID->mnId) {
      std::vector<KeyFrame*> vpKFs
        = std::vector<KeyFrame*>(mspKeyFrames.begin(), mspKeyFrames.end());
      std::sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
      mpKFlowerID = vpKFs[0];
    }
  } else {
    mpKFlowerID = 0;
  }

  // TODO: This only erase the pointer.
  // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const std::vector<MapPoint*>& vpMPs) {
  std::unique_lock<std::mutex> lock(mMutexMap);
  mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange() {
  std::unique_lock<std::mutex> lock(mMutexMap);
  mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx() {
  std::unique_lock<std::mutex> lock(mMutexMap);
  return mnBigChangeIdx;
}

std::vector<KeyFrame*> Map::GetAllKeyFrames() {
  std::unique_lock<std::mutex> lock(mMutexMap);
  return std::vector<KeyFrame*>(mspKeyFrames.begin(), mspKeyFrames.end());
}

std::vector<MapPoint*> Map::GetAllMapPoints() {
  std::unique_lock<std::mutex> lock(mMutexMap);
  return std::vector<MapPoint*>(mspMapPoints.begin(), mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap() {
  std::unique_lock<std::mutex> lock(mMutexMap);
  return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap() {
  std::unique_lock<std::mutex> lock(mMutexMap);
  return mspKeyFrames.size();
}

std::vector<MapPoint*> Map::GetReferenceMapPoints() {
  std::unique_lock<std::mutex> lock(mMutexMap);
  return mvpReferenceMapPoints;
}

long unsigned int Map::GetId() {
  return mnId;
}

long unsigned int Map::GetInitKFid() {
  std::unique_lock<std::mutex> lock(mMutexMap);
  return mnInitKFid;
}

void Map::SetInitKFid(long unsigned int initKFif) {
  std::unique_lock<std::mutex> lock(mMutexMap);
  mnInitKFid = initKFif;
}

long unsigned int Map::GetMaxKFid() {
  std::unique_lock<std::mutex> lock(mMutexMap);
  return mnMaxKFid;
}

KeyFrame* Map::GetOriginKF() {
  return mpKFinitial;
}

void Map::SetCurrentMap() {
  mIsInUse = true;
}

void Map::SetStoredMap() {
  mIsInUse = false;
}

void Map::clear() {
  _logger->info("Clearing map: releasing all keyframes, map points, and resetting state");

  // for (auto sit = mspMapPoints.begin(); sit != mspMapPoints.end(); sit++) {
  //   delete *sit;
  // }

  std::ranges::for_each(mspKeyFrames, [](KeyFrame* const kf) {
    kf->UpdateMap(static_cast<Map*>(NULL));
    // delete *sit;
  });

  mspMapPoints.clear();
  mspKeyFrames.clear();
  mnMaxKFid        = mnInitKFid;
  mbImuInitialized = false;
  mvpReferenceMapPoints.clear();
  mvpKeyFrameOrigins.clear();
  mbIMU_BA1 = false;
  mbIMU_BA2 = false;
}

bool Map::IsInUse() {
  return mIsInUse;
}

void Map::SetBad() {
  mbBad = true;
}

bool Map::IsBad() {
  return mbBad;
}

void Map::ApplyScaledRotation(const Sophus::SE3f& T, const float s, const bool bScaledVel) {
  std::unique_lock<std::mutex> lock(mMutexMap);

  _logger->info("Applying scale of {:.6f} to rotation{}...", s, bScaledVel ? " and velocity" : "");

  // Body position (IMU) of first keyframe is fixed to (0,0,0)
  Sophus::SE3f    Tyw = T;
  Eigen::Matrix3f Ryw = Tyw.rotationMatrix();
  Eigen::Vector3f tyw = Tyw.translation();

  for (KeyFrame* const kf : mspKeyFrames) {
    Sophus::SE3f Twc   = kf->GetPoseInverse();
    Twc.translation() *= s;
    Sophus::SE3f Tyc   = Tyw * Twc;
    Sophus::SE3f Tcy   = Tyc.inverse();
    kf->SetPose(Tcy);
    Eigen::Vector3f Vw = kf->GetVelocity();
    if (!bScaledVel) {
      kf->SetVelocity(Ryw * Vw);
    } else {
      kf->SetVelocity(Ryw * Vw * s);
    }
  }
  for (MapPoint* const mp : mspMapPoints) {
    mp->SetWorldPos(s * Ryw * mp->GetWorldPos() + tyw);
    mp->UpdateNormalAndDepth();
  }
  mnMapChange++;
}

void Map::SetInertialSensor() {
  std::unique_lock<std::mutex> lock(mMutexMap);
  mbIsInertial = true;
}

bool Map::IsInertial() {
  std::unique_lock<std::mutex> lock(mMutexMap);
  return mbIsInertial;
}

void Map::SetIniertialBA1() {
  std::unique_lock<std::mutex> lock(mMutexMap);
  mbIMU_BA1 = true;
}

void Map::SetIniertialBA2() {
  std::unique_lock<std::mutex> lock(mMutexMap);
  mbIMU_BA2 = true;
}

bool Map::GetIniertialBA1() {
  std::unique_lock<std::mutex> lock(mMutexMap);
  return mbIMU_BA1;
}

bool Map::GetIniertialBA2() {
  std::unique_lock<std::mutex> lock(mMutexMap);
  return mbIMU_BA2;
}

void Map::ChangeId(long unsigned int nId) {
  mnId = nId;
}

unsigned int Map::GetLowerKFID() {
  std::unique_lock<std::mutex> lock(mMutexMap);
  if (mpKFlowerID) {
    return mpKFlowerID->mnId;
  }
  return 0;
}

int Map::GetMapChangeIndex() {
  std::unique_lock<std::mutex> lock(mMutexMap);
  return mnMapChange;
}

void Map::IncreaseChangeIndex() {
  std::unique_lock<std::mutex> lock(mMutexMap);
  mnMapChange++;
}

int Map::GetLastMapChange() {
  std::unique_lock<std::mutex> lock(mMutexMap);
  return mnMapChangeNotified;
}

void Map::SetLastMapChange(int currentChangeId) {
  std::unique_lock<std::mutex> lock(mMutexMap);
  mnMapChangeNotified = currentChangeId;
}

void Map::PreSave(std::set<GeometricCamera*>& spCams) {
  _logger->info("PreSave: cleaning up observations and backing up map data.");

  int nMPWithoutObs = 0;
  for (MapPoint* const mp : mspMapPoints) {
    if (!mp || mp->isBad()) {
      continue;
    }

    if (mp->GetObservations().size() == 0) {
      nMPWithoutObs++;
    }
    // TODO(VuHoi): get reference to observations instead of copying.
    // TODO(VuHoi): consider 2 loops:
    // - 1st loop: gather to-erase observations
    // - 2nd loop: erase them
    auto mpObs = mp->GetObservations();
    for (KeyFrame* const kf : mpObs | std::views::keys) {
      if (kf->GetMap() != this || kf->isBad()) {
        mp->EraseObservation(kf);
      }
    }
  }

  // Saves the id of KF origins
  mvBackupKeyFrameOriginsId.clear();
  mvBackupKeyFrameOriginsId.reserve(mvpKeyFrameOrigins.size());
  std::ranges::copy(
    mvpKeyFrameOrigins | std::views::transform([](KeyFrame* const kf) {
      return kf->mnId;
    }),
    std::back_inserter(mvBackupKeyFrameOriginsId)
  );

  // Backup of MapPoints
  mvpBackupMapPoints.clear();
  for (MapPoint* const mp : mspMapPoints) {
    if (!mp || mp->isBad()) {
      continue;
    }

    mvpBackupMapPoints.push_back(mp);
    mp->PreSave(mspKeyFrames, mspMapPoints);
  }

  // Backup of KeyFrames
  mvpBackupKeyFrames.clear();
  for (KeyFrame* const kf : mspKeyFrames) {
    if (!kf || kf->isBad()) {
      continue;
    }

    mvpBackupKeyFrames.push_back(kf);
    kf->PreSave(mspKeyFrames, mspMapPoints, spCams);
  }

  mnBackupKFinitialID = -1;
  if (mpKFinitial) {
    mnBackupKFinitialID = mpKFinitial->mnId;
  }

  mnBackupKFlowerID = -1;
  if (mpKFlowerID) {
    mnBackupKFlowerID = mpKFlowerID->mnId;
  }
}

void Map::PostLoad(
  KeyFrameDatabase* pKFDB,
  ORBVocabulary*    pORBVoc /*, std::map<long unsigned int, KeyFrame*>& mpKeyFrameId*/,
  std::map<unsigned int, GeometricCamera*>& mpCams
) {
  _logger->info(
    "PostLoad: restoring map from backup (key frames: {}, map points: {})",
    mvpBackupKeyFrames.size(),
    mvpBackupMapPoints.size()
  );

  std::copy(
    mvpBackupMapPoints.begin(),
    mvpBackupMapPoints.end(),
    std::inserter(mspMapPoints, mspMapPoints.begin())
  );
  std::copy(
    mvpBackupKeyFrames.begin(),
    mvpBackupKeyFrames.end(),
    std::inserter(mspKeyFrames, mspKeyFrames.begin())
  );

  std::map<long unsigned int, MapPoint*> mpMapPointId;
  for (MapPoint* const mp : mspMapPoints) {
    if (!mp || mp->isBad()) {
      continue;
    }

    mp->UpdateMap(this);
    mpMapPointId[mp->mnId] = mp;
  }

  std::map<long unsigned int, KeyFrame*> mpKeyFrameId;
  for (KeyFrame* const kf : mspKeyFrames) {
    if (!kf || kf->isBad()) {
      continue;
    }

    kf->UpdateMap(this);
    kf->SetORBVocabulary(pORBVoc);
    kf->SetKeyFrameDatabase(pKFDB);
    mpKeyFrameId[kf->mnId] = kf;
  }

  // References reconstruction between different instances
  for (MapPoint* const mp : mspMapPoints) {
    if (!mp || mp->isBad()) {
      continue;
    }

    mp->PostLoad(mpKeyFrameId, mpMapPointId);
  }

  for (KeyFrame* const kf : mspKeyFrames) {
    if (!kf || kf->isBad()) {
      continue;
    }

    kf->PostLoad(mpKeyFrameId, mpMapPointId, mpCams);
    pKFDB->add(kf);
  }

  if (mnBackupKFinitialID != -1) {
    mpKFinitial = mpKeyFrameId[mnBackupKFinitialID];
  }

  if (mnBackupKFlowerID != -1) {
    mpKFlowerID = mpKeyFrameId[mnBackupKFlowerID];
  }

  mvpKeyFrameOrigins.clear();
  mvpKeyFrameOrigins.reserve(mvBackupKeyFrameOriginsId.size());
  std::ranges::copy(
    mvBackupKeyFrameOriginsId | std::views::transform([&](const unsigned long id) {
      return mpKeyFrameId[id];
    }),
    std::back_inserter(mvpKeyFrameOrigins)
  );

  mvpBackupMapPoints.clear();
}

} // namespace ORB_SLAM3

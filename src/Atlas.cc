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

#include "Atlas.h"
#include <algorithm>
#include <numeric>
#include "KeyFrameDatabase.h"
#include "LoggingUtils.h"
#include "MapPoint.h"
#include "Viewer.h"

namespace ORB_SLAM3 {

Atlas::Atlas() : _logger(logging::CreateModuleLogger("Atlas")) {
  mpCurrentMap = static_cast<Map*>(NULL);
}

Atlas::Atlas(int initKFid)
  : mnLastInitKFidMap(initKFid), mHasViewer(false), _logger(logging::CreateModuleLogger("Atlas")) {
  mpCurrentMap = static_cast<Map*>(NULL);
  CreateNewMap();
}

Atlas::~Atlas() {
  // TODO(VuHoi): use nullptr
  std::erase_if(mspMaps, [](Map* map) {
    if (map) {
      delete map;
      map = static_cast<Map*>(NULL);
      return true;
    }
    return false;
  });
}

void Atlas::CreateNewMap() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  _logger->info("New map created with ID {}", Map::nNextId);
  if (mpCurrentMap) {
    if (!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid()) {
      mnLastInitKFidMap
        = mpCurrentMap->GetMaxKFid() + 1; // The init KF is the next of current maximum
    }

    mpCurrentMap->SetStoredMap();
    _logger->info("Map stored with ID {}", mpCurrentMap->GetId());

    // if(mHasViewer)
    //     mpViewer->AddMapToCreateThumbnail(mpCurrentMap);
  }
  _logger->info("New map created with last KeyFrame ID: {}", mnLastInitKFidMap);

  mpCurrentMap = new Map(mnLastInitKFidMap);
  mpCurrentMap->SetCurrentMap();
  mspMaps.insert(mpCurrentMap);
}

void Atlas::ChangeMap(Map* pMap) {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  _logger->info("Change to map with ID {}", pMap->GetId());
  if (mpCurrentMap) {
    mpCurrentMap->SetStoredMap();
  }

  mpCurrentMap = pMap;
  mpCurrentMap->SetCurrentMap();
}

unsigned long int Atlas::GetLastInitKFid() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mnLastInitKFidMap;
}

void Atlas::SetViewer(Viewer* pViewer) {
  mpViewer   = pViewer;
  mHasViewer = true;
}

void Atlas::AddKeyFrame(KeyFrame* pKF) {
  Map* pMapKF = pKF->GetMap();
  pMapKF->AddKeyFrame(pKF);
}

void Atlas::AddMapPoint(MapPoint* pMP) {
  Map* pMapMP = pMP->GetMap();
  pMapMP->AddMapPoint(pMP);
}

GeometricCamera* Atlas::AddCamera(GeometricCamera* pCam) {
  // Check if the camera already exists
  bool bAlreadyInMap = false;
  int  index_cam     = -1;
  // TODO(VuHoi): check pCam existing
  // TODO(VuHoi): use std::views::enumerate in C++23
  for (std::size_t i = 0; i < mvpCameras.size(); ++i) {
    GeometricCamera* pCam_i = mvpCameras[i];
    if (!pCam) {
      _logger->warn("Camera not existing");
    }
    if (!pCam_i) {
      _logger->warn("Camera (i) not existing");
    }

    // Skips comparison if cameras are different types
    if (pCam->GetType() != pCam_i->GetType()) {
      continue;
    }

    if (pCam->GetType() == GeometricCamera::CAM_PINHOLE) {
      if (((Pinhole*)pCam_i)->IsEqual(pCam)) {
        bAlreadyInMap = true;
        index_cam     = i;
      }
    } else if (pCam->GetType() == GeometricCamera::CAM_FISHEYE) {
      if (((KannalaBrandt8*)pCam_i)->IsEqual(pCam)) {
        bAlreadyInMap = true;
        index_cam     = i;
      }
    }
  }

  if (bAlreadyInMap) {
    return mvpCameras[index_cam];
  } else {
    mvpCameras.push_back(pCam);
    return pCam;
  }
}

std::vector<GeometricCamera*> Atlas::GetAllCameras() {
  return mvpCameras;
}

void Atlas::SetReferenceMapPoints(const std::vector<MapPoint*>& vpMPs) {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  mpCurrentMap->SetReferenceMapPoints(vpMPs);
}

void Atlas::InformNewBigChange() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  mpCurrentMap->InformNewBigChange();
}

int Atlas::GetLastBigChangeIdx() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetLastBigChangeIdx();
}

long unsigned int Atlas::MapPointsInMap() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mpCurrentMap->MapPointsInMap();
}

long unsigned Atlas::KeyFramesInMap() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mpCurrentMap->KeyFramesInMap();
}

std::vector<KeyFrame*> Atlas::GetAllKeyFrames() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetAllKeyFrames();
}

std::vector<MapPoint*> Atlas::GetAllMapPoints() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetAllMapPoints();
}

std::vector<MapPoint*> Atlas::GetReferenceMapPoints() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mpCurrentMap->GetReferenceMapPoints();
}

std::vector<Map*> Atlas::GetAllMaps() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  struct compFunctor {
    inline bool operator()(Map* elem1, Map* elem2) {
      return elem1->GetId() < elem2->GetId();
    }
  };
  std::vector<Map*> vMaps(mspMaps.begin(), mspMaps.end());
  std::sort(vMaps.begin(), vMaps.end(), compFunctor());
  return vMaps;
}

int Atlas::CountMaps() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mspMaps.size();
}

void Atlas::clearMap() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  mpCurrentMap->clear();
}

void Atlas::clearAtlas() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  /*for (auto it=mspMaps.begin(), send=mspMaps.end(); it!=send; it++) {
    (*it)->clear();
    delete *it;
  }*/
  mspMaps.clear();
  mpCurrentMap      = static_cast<Map*>(NULL);
  mnLastInitKFidMap = 0;
}

Map* Atlas::GetCurrentMap() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  if (!mpCurrentMap) {
    CreateNewMap();
  }
  while (mpCurrentMap->IsBad()) {
    usleep(3000);
  }

  return mpCurrentMap;
}

void Atlas::SetMapBad(Map* pMap) {
  mspMaps.erase(pMap);
  pMap->SetBad();

  mspBadMaps.insert(pMap);
}

void Atlas::RemoveBadMaps() {
  /*for(Map* pMap : mspBadMaps)
  {
      delete pMap;
      pMap = static_cast<Map*>(NULL);
  }*/
  mspBadMaps.clear();
}

bool Atlas::isInertial() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mpCurrentMap->IsInertial();
}

void Atlas::SetInertialSensor() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  mpCurrentMap->SetInertialSensor();
}

void Atlas::SetImuInitialized() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  mpCurrentMap->SetImuInitialized();
}

bool Atlas::isImuInitialized() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);
  return mpCurrentMap->isImuInitialized();
}

void Atlas::PreSave() {
  if (mpCurrentMap) {
    if (!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid()) {
      mnLastInitKFidMap
        = mpCurrentMap->GetMaxKFid() + 1; // The init KF is the next of current maximum
    }
  }

  struct compFunctor {
    inline bool operator()(Map* elem1, Map* elem2) {
      return elem1->GetId() < elem2->GetId();
    }
  };
  std::copy(mspMaps.begin(), mspMaps.end(), std::back_inserter(mvpBackupMaps));
  std::sort(mvpBackupMaps.begin(), mvpBackupMaps.end(), compFunctor());

  std::set<GeometricCamera*> spCams(mvpCameras.begin(), mvpCameras.end());
  for (Map* const map : mvpBackupMaps) {
    if (!map || map->IsBad()) {
      continue;
    }

    // TODO(VuHoi): check empty instead
    if (map->GetAllKeyFrames().size() == 0) {
      // Empty map, erase before of save it.
      SetMapBad(map);
      continue;
    }
    map->PreSave(spCams);
  }
  RemoveBadMaps();
}

void Atlas::PostLoad() {
  std::map<unsigned int, GeometricCamera*> mpCams;
  for (GeometricCamera* const camera : mvpCameras) {
    mpCams[camera->GetId()] = camera;
  }

  mspMaps.clear();
  unsigned long int numKF = 0, numMP = 0;
  for (Map* const map : mvpBackupMaps) {
    mspMaps.insert(map);
    map->PostLoad(mpKeyFrameDB, mpORBVocabulary, mpCams);
    numKF += map->GetAllKeyFrames().size();
    numMP += map->GetAllMapPoints().size();
  }
  mvpBackupMaps.clear();
}

void Atlas::SetKeyFrameDababase(KeyFrameDatabase* pKFDB) {
  mpKeyFrameDB = pKFDB;
}

KeyFrameDatabase* Atlas::GetKeyFrameDatabase() {
  return mpKeyFrameDB;
}

void Atlas::SetORBVocabulary(ORBVocabulary* pORBVoc) {
  mpORBVocabulary = pORBVoc;
}

ORBVocabulary* Atlas::GetORBVocabulary() {
  return mpORBVocabulary;
}

long unsigned int Atlas::GetNumLivedKF() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);

  // Return the total number of keyframes across all maps in atlas.
  return std::transform_reduce(
    mspMaps.begin(),
    mspMaps.end(),
    0UL,
    std::plus<>(),
    [](Map* const map) {
      return map->GetAllKeyFrames().size();
    }
  );
}

long unsigned int Atlas::GetNumLivedMP() {
  std::unique_lock<std::mutex> lock(mMutexAtlas);

  // Return the total number of mappoints across all maps in atlas.
  return std::transform_reduce(
    mspMaps.begin(),
    mspMaps.end(),
    0UL,
    std::plus<>(),
    [](Map* const map) {
      return map->GetAllMapPoints().size();
    }
  );
}

std::map<long unsigned int, KeyFrame*> Atlas::GetAtlasKeyframes() {
  std::map<long unsigned int, KeyFrame*> mpIdKFs;

  // Construct a map of keyframe IDs and keyframes.
  for (Map* const map : mvpBackupMaps) {
    // TODO(VuHoi): add nullptr check for map.
    for (KeyFrame* const kf : map->GetAllKeyFrames()) {
      // TODO(VuHoi): add nullptr check for keyframe.
      mpIdKFs[kf->mnId] = kf;
    }
  }
  return mpIdKFs;
}

} // namespace ORB_SLAM3

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

#include "LoopClosing.h"
#include <algorithm>
#include <ranges>
#include "Atlas.h"
#include "Converter.h"
#include "G2oTypes.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "LoggingUtils.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Sim3Solver.h"
#include "System.h"
#include "Tracking.h"

namespace ORB_SLAM3 {

LoopClosing::LoopClosing(
  Atlas*            pAtlas,
  KeyFrameDatabase* pDB,
  ORBVocabulary*    pVoc,
  const bool        bFixScale,
  const bool        bActiveLC
)
  : mbResetRequested(false)
  , mbResetActiveMapRequested(false)
  , mbFinishRequested(false)
  , mbFinished(true)
  , mpAtlas(pAtlas)
  , mpKeyFrameDB(pDB)
  , mpORBVocabulary(pVoc)
  , mpMatchedKF(NULL)
  , mLastLoopKFid(0)
  , mbRunningGBA(false)
  , mbFinishedGBA(true)
  , mbStopGBA(false)
  , mpThreadGBA(NULL)
  , mbFixScale(bFixScale)
  , mnFullBAIdx(0)
  , mnLoopNumCoincidences(0)
  , mnMergeNumCoincidences(0)
  , mbLoopDetected(false)
  , mbMergeDetected(false)
  , mnLoopNumNotFound(0)
  , mnMergeNumNotFound(0)
  , mbActiveLC(bActiveLC)
  , _logger(logging::CreateModuleLogger("LoopClosing")) {
  mnCovisibilityConsistencyTh = 3;
  mpLastCurrentKF             = static_cast<KeyFrame*>(NULL);

#ifdef REGISTER_TIMES
  vdDataQuery_ms.clear();
  vdEstSim3_ms.clear();
  vdPRTotal_ms.clear();

  vdMergeMaps_ms.clear();
  vdWeldingBA_ms.clear();
  vdMergeOptEss_ms.clear();
  vdMergeTotal_ms.clear();
  vnMergeKFs.clear();
  vnMergeMPs.clear();
  nMerges = 0;

  vdLoopFusion_ms.clear();
  vdLoopOptEss_ms.clear();
  vdLoopTotal_ms.clear();
  vnLoopKFs.clear();
  nLoop = 0;

  vdGBA_ms.clear();
  vdUpdateMap_ms.clear();
  vdFGBATotal_ms.clear();
  vnGBAKFs.clear();
  vnGBAMPs.clear();
  nFGBA_exec  = 0;
  nFGBA_abort = 0;
#endif

  mstrFolderSubTraj = "SubTrajectories/";
  mnNumCorrection   = 0;
  mnCorrectionGBA   = 0;
}

void LoopClosing::SetTracker(Tracking* pTracker) {
  mpTracker = pTracker;
}

void LoopClosing::SetLocalMapper(LocalMapping* pLocalMapper) {
  mpLocalMapper = pLocalMapper;
}

void LoopClosing::Run() {
  mbFinished = false;

  while (1) {
    // NEW LOOP AND MERGE DETECTION ALGORITHM
    //----------------------------

    if (CheckNewKeyFrames()) {
      if (mpLastCurrentKF) {
        mpLastCurrentKF->mvpLoopCandKFs.clear();
        mpLastCurrentKF->mvpMergeCandKFs.clear();
      }
#ifdef REGISTER_TIMES
      std::chrono::steady_clock::time_point time_StartPR = std::chrono::steady_clock::now();
#endif

      bool bFindedRegion = NewDetectCommonRegions();

#ifdef REGISTER_TIMES
      std::chrono::steady_clock::time_point time_EndPR = std::chrono::steady_clock::now();

      double timePRTotal = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                             time_EndPR - time_StartPR
      )
                             .count();
      vdPRTotal_ms.push_back(timePRTotal);
#endif
      if (bFindedRegion) {
        if (mbMergeDetected) {
          if (IsInertialBased(mpTracker->mSensor) && (!mpCurrentKF->GetMap()->isImuInitialized())) {
            _logger->warn("Merge aborted: IMU is not initialized");
          } else {
            Sophus::SE3d mTmw = mpMergeMatchedKF->GetPose().cast<double>();
            g2o::Sim3    gSmw2(mTmw.unit_quaternion(), mTmw.translation(), 1.0);
            Sophus::SE3d mTcw = mpCurrentKF->GetPose().cast<double>();
            g2o::Sim3    gScw1(mTcw.unit_quaternion(), mTcw.translation(), 1.0);
            g2o::Sim3    gSw2c = mg2oMergeSlw.inverse();
            g2o::Sim3    gSw1m = mg2oMergeSlw;

            mSold_new = (gSw2c * gScw1);

            if (mpCurrentKF->GetMap()->IsInertial() && mpMergeMatchedKF->GetMap()->IsInertial()) {
              _logger->info("Performing transformation check using IMU data");
              if (mSold_new.scale() < 0.90 || mSold_new.scale() > 1.1) {
                mpMergeLastCurrentKF->SetErase();
                mpMergeMatchedKF->SetErase();
                mnMergeNumCoincidences = 0;
                mvpMergeMatchedMPs.clear();
                mvpMergeMPs.clear();
                mnMergeNumNotFound = 0;
                mbMergeDetected    = false;
                _logger->warn(
                  "Estimated scale out of acceptable range [0.9, 1.1]. Aborting merge..."
                );
                continue;
              }
              // If inertial, force only yaw
              if (IsInertialBased(mpTracker->mSensor) && mpCurrentKF->GetMap()->GetIniertialBA1()) {
                Eigen::Vector3d phi = LogSO3(mSold_new.rotation().toRotationMatrix());
                phi(0)              = 0;
                phi(1)              = 0;
                mSold_new           = g2o::Sim3(ExpSO3(phi), mSold_new.translation(), 1.0);
              }
            }

            mg2oMergeSmw = gSmw2 * gSw2c * gScw1;

            mg2oMergeScw = mg2oMergeSlw;

            // mpTracker->SetStepByStep(true);

            _logger->info("Map merge successfully detected");

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartMerge
              = std::chrono::steady_clock::now();
            nMerges += 1;
#endif
            // TODO UNCOMMENT
            if (IsInertialBased(mpTracker->mSensor)) {
              MergeLocal2();
            } else {
              MergeLocal();
            }

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndMerge = std::chrono::steady_clock::now();
            double                                timeMergeTotal
              = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                  time_EndMerge - time_StartMerge
              )
                  .count();
            vdMergeTotal_ms.push_back(timeMergeTotal);
#endif

            _logger->info("Map merge completed successfully");
          }

          vdPR_CurrentTime.push_back(mpCurrentKF->mTimeStamp);
          vdPR_MatchedTime.push_back(mpMergeMatchedKF->mTimeStamp);
          vnPR_TypeRecogn.push_back(1);

          // Reset all variables
          mpMergeLastCurrentKF->SetErase();
          mpMergeMatchedKF->SetErase();
          mnMergeNumCoincidences = 0;
          mvpMergeMatchedMPs.clear();
          mvpMergeMPs.clear();
          mnMergeNumNotFound = 0;
          mbMergeDetected    = false;

          if (mbLoopDetected) {
            // Reset Loop variables
            mpLoopLastCurrentKF->SetErase();
            mpLoopMatchedKF->SetErase();
            mnLoopNumCoincidences = 0;
            mvpLoopMatchedMPs.clear();
            mvpLoopMPs.clear();
            mnLoopNumNotFound = 0;
            mbLoopDetected    = false;
          }
        }

        if (mbLoopDetected) {
          bool bGoodLoop = true;
          vdPR_CurrentTime.push_back(mpCurrentKF->mTimeStamp);
          vdPR_MatchedTime.push_back(mpLoopMatchedKF->mTimeStamp);
          vnPR_TypeRecogn.push_back(0);

          _logger->info("Loop closure detected");

          mg2oLoopScw = mg2oLoopSlw; //*mvg2oSim3LoopTcw[nCurrentIndex];
          if (mpCurrentKF->GetMap()->IsInertial()) {
            Sophus::SE3d Twc = mpCurrentKF->GetPoseInverse().cast<double>();
            g2o::Sim3    g2oTwc(Twc.unit_quaternion(), Twc.translation(), 1.0);
            g2o::Sim3    g2oSww_new = g2oTwc * mg2oLoopScw;

            Eigen::Vector3d phi = LogSO3(g2oSww_new.rotation().toRotationMatrix());
            {
              std::ostringstream oss;
              oss << phi.transpose();
              _logger->info("phi: {}", oss.str());
            }

            if (std::fabs(phi(0)) < 0.008f && std::fabs(phi(1)) < 0.008f && std::fabs(phi(2)) < 0.349f) {
              if (mpCurrentKF->GetMap()->IsInertial()) {
                // If inertial, force only yaw
                if (IsInertialBased(mpTracker->mSensor) && mpCurrentKF->GetMap()->GetIniertialBA2()) {
                  phi(0)      = 0;
                  phi(1)      = 0;
                  g2oSww_new  = g2o::Sim3(ExpSO3(phi), g2oSww_new.translation(), 1.0);
                  mg2oLoopScw = g2oTwc.inverse() * g2oSww_new;
                }
              }
            } else {
              _logger->warn("Bad loop detected");
              bGoodLoop = false;
            }
          }

          if (bGoodLoop) {
            mvpLoopMapPoints = mvpLoopMPs;

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_StartLoop = std::chrono::steady_clock::now();
            nLoop += 1;
#endif
            CorrectLoop();
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndLoop = std::chrono::steady_clock::now();
            double                                timeLoopTotal
              = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                  time_EndLoop - time_StartLoop
              )
                  .count();
            vdLoopTotal_ms.push_back(timeLoopTotal);
#endif

            mnNumCorrection += 1;
          }

          // Reset all variables
          mpLoopLastCurrentKF->SetErase();
          mpLoopMatchedKF->SetErase();
          mnLoopNumCoincidences = 0;
          mvpLoopMatchedMPs.clear();
          mvpLoopMPs.clear();
          mnLoopNumNotFound = 0;
          mbLoopDetected    = false;
        }
      }
      mpLastCurrentKF = mpCurrentKF;
    }

    ResetIfRequested();

    if (CheckFinish()) {
      break;
    }

    usleep(5000);
  }

  SetFinish();
}

void LoopClosing::InsertKeyFrame(KeyFrame* pKF) {
  std::unique_lock<std::mutex> lock(mMutexLoopQueue);
  if (pKF->mnId != 0) {
    mlpLoopKeyFrameQueue.push_back(pKF);
  }
}

bool LoopClosing::CheckNewKeyFrames() {
  std::unique_lock<std::mutex> lock(mMutexLoopQueue);
  return (!mlpLoopKeyFrameQueue.empty());
}

bool LoopClosing::NewDetectCommonRegions() {
  // To deactivate placerecognition. No loopclosing nor merging will be performed
  if (!mbActiveLC) {
    return false;
  }

  {
    std::unique_lock<std::mutex> lock(mMutexLoopQueue);
    mpCurrentKF = mlpLoopKeyFrameQueue.front();
    mlpLoopKeyFrameQueue.pop_front();
    // Avoid that a keyframe can be erased while it is being process by this thread
    mpCurrentKF->SetNotErase();
    mpCurrentKF->mbCurrentPlaceRecognition = true;

    mpLastMap = mpCurrentKF->GetMap();
  }

  if (mpLastMap->IsInertial() && !mpLastMap->GetIniertialBA2()) {
    mpKeyFrameDB->add(mpCurrentKF);
    mpCurrentKF->SetErase();
    return false;
  }

  if (mpTracker->mSensor == Sensor::Stereo && mpLastMap->GetAllKeyFrames().size() < 5) { // 12
    _logger->info(
      "Stereo key frame {} inserted without check; map is small (fewer than 5 key frames)",
      mpCurrentKF->mnId
    );
    mpKeyFrameDB->add(mpCurrentKF);
    mpCurrentKF->SetErase();
    return false;
  }

  if (mpLastMap->GetAllKeyFrames().size() < 12) {
    _logger->info(
      "Key frame {} inserted without check; map is small (fewer than 12 key frames)",
      mpCurrentKF->mnId
    );
    mpKeyFrameDB->add(mpCurrentKF);
    mpCurrentKF->SetErase();
    return false;
  }

  _logger->info("Checking key frame {}", mpCurrentKF->mnId);

  // Check the last candidates with geometric validation
  //  Loop candidates
  bool bLoopDetectedInKF = false;
  bool bCheckSpatial     = false;

#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_StartEstSim3_1 = std::chrono::steady_clock::now();
#endif
  if (mnLoopNumCoincidences > 0) {
    bCheckSpatial = true;
    // Find from the last KF candidates
    Sophus::SE3d mTcl
      = (mpCurrentKF->GetPose() * mpLoopLastCurrentKF->GetPoseInverse()).cast<double>();
    g2o::Sim3              gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);
    g2o::Sim3              gScw           = gScl * mg2oLoopSlw;
    int                    numProjMatches = 0;
    std::vector<MapPoint*> vpMatchedMPs;
    bool                   bCommonRegion = DetectAndReffineSim3FromLastKF(
      mpCurrentKF,
      mpLoopMatchedKF,
      gScw,
      numProjMatches,
      mvpLoopMPs,
      vpMatchedMPs
    );
    if (bCommonRegion) {
      bLoopDetectedInKF = true;

      mnLoopNumCoincidences++;
      mpLoopLastCurrentKF->SetErase();
      mpLoopLastCurrentKF = mpCurrentKF;
      mg2oLoopSlw         = gScw;
      mvpLoopMatchedMPs   = vpMatchedMPs;

      mbLoopDetected    = mnLoopNumCoincidences >= 3;
      mnLoopNumNotFound = 0;
    } else {
      bLoopDetectedInKF = false;

      mnLoopNumNotFound++;
      if (mnLoopNumNotFound >= 2) {
        _logger->warn("Loop detection reset, {} consecutive failures", mnLoopNumNotFound);
        mpLoopLastCurrentKF->SetErase();
        mpLoopMatchedKF->SetErase();
        mnLoopNumCoincidences = 0;
        mvpLoopMatchedMPs.clear();
        mvpLoopMPs.clear();
        mnLoopNumNotFound = 0;
      }
    }
  }

  // Merge candidates
  bool bMergeDetectedInKF = false;
  if (mnMergeNumCoincidences > 0) {
    // Find from the last KF candidates
    Sophus::SE3d mTcl
      = (mpCurrentKF->GetPose() * mpMergeLastCurrentKF->GetPoseInverse()).cast<double>();

    g2o::Sim3              gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);
    g2o::Sim3              gScw           = gScl * mg2oMergeSlw;
    int                    numProjMatches = 0;
    std::vector<MapPoint*> vpMatchedMPs;
    bool                   bCommonRegion = DetectAndReffineSim3FromLastKF(
      mpCurrentKF,
      mpMergeMatchedKF,
      gScw,
      numProjMatches,
      mvpMergeMPs,
      vpMatchedMPs
    );
    if (bCommonRegion) {
      bMergeDetectedInKF = true;

      mnMergeNumCoincidences++;
      mpMergeLastCurrentKF->SetErase();
      mpMergeLastCurrentKF = mpCurrentKF;
      mg2oMergeSlw         = gScw;
      mvpMergeMatchedMPs   = vpMatchedMPs;

      mbMergeDetected = mnMergeNumCoincidences >= 3;
    } else {
      mbMergeDetected    = false;
      bMergeDetectedInKF = false;

      mnMergeNumNotFound++;
      if (mnMergeNumNotFound >= 2) {
        _logger->warn("Merge detection reset, {} consecutive failures", mnLoopNumNotFound);
        mpMergeLastCurrentKF->SetErase();
        mpMergeMatchedKF->SetErase();
        mnMergeNumCoincidences = 0;
        mvpMergeMatchedMPs.clear();
        mvpMergeMPs.clear();
        mnMergeNumNotFound = 0;
      }
    }
  }
#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_EndEstSim3_1 = std::chrono::steady_clock::now();
  double timeEstSim3 = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                         time_EndEstSim3_1 - time_StartEstSim3_1
  )
                         .count();
#endif

  if (mbMergeDetected || mbLoopDetected) {
    _logger->info("Loop or merge detected, adding key frame {} to database", mpCurrentKF->mnId);
#ifdef REGISTER_TIMES
    vdEstSim3_ms.push_back(timeEstSim3);
#endif
    mpKeyFrameDB->add(mpCurrentKF);
    return true;
  }

  // TODO: This is only necessary if we use a minimun score for pick the best candidates
  const std::vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();

  // Extract candidates from the bag of words
  std::vector<KeyFrame*> vpMergeBowCand, vpLoopBowCand;
  if (!bMergeDetectedInKF || !bLoopDetectedInKF) {
    // Search in BoW
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartQuery = std::chrono::steady_clock::now();
#endif
    mpKeyFrameDB->DetectNBestCandidates(mpCurrentKF, vpLoopBowCand, vpMergeBowCand, 3);
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndQuery = std::chrono::steady_clock::now();
    double timeDataQuery = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                             time_EndQuery - time_StartQuery
    )
                             .count();
    vdDataQuery_ms.push_back(timeDataQuery);
#endif
    _logger->info(
      "Loop or merge not detected in key frame, so detect in database and find {} candidates",
      vpLoopBowCand.size()
    );
  }

#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_StartEstSim3_2 = std::chrono::steady_clock::now();
#endif
  // Check the BoW candidates if the geometric candidate list is empty
  // Loop candidates
  if (!bLoopDetectedInKF && !vpLoopBowCand.empty()) {
    _logger->info("Attempting loop detection from {} BoW candidates...", vpLoopBowCand.size());
    mbLoopDetected = DetectCommonRegionsFromBoW(
      vpLoopBowCand,
      mpLoopMatchedKF,
      mpLoopLastCurrentKF,
      mg2oLoopSlw,
      mnLoopNumCoincidences,
      mvpLoopMPs,
      mvpLoopMatchedMPs
    );
    _logger->info("Loop detection {}", mbLoopDetected ? "succeeded" : "failed");
  }
  // Merge candidates
  if (!bMergeDetectedInKF && !vpMergeBowCand.empty()) {
    _logger->info("Attempting merge detection from {} BoW candidates...", vpLoopBowCand.size());
    mbMergeDetected = DetectCommonRegionsFromBoW(
      vpMergeBowCand,
      mpMergeMatchedKF,
      mpMergeLastCurrentKF,
      mg2oMergeSlw,
      mnMergeNumCoincidences,
      mvpMergeMPs,
      mvpMergeMatchedMPs
    );
    _logger->info("Merge detection {}", mbMergeDetected ? "succeeded" : "failed");
  }

#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_EndEstSim3_2 = std::chrono::steady_clock::now();
  timeEstSim3 += std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                   time_EndEstSim3_2 - time_StartEstSim3_2
  )
                   .count();
  vdEstSim3_ms.push_back(timeEstSim3);
#endif

  mpKeyFrameDB->add(mpCurrentKF);
  _logger->info("Key frame {} added to database", mpCurrentKF->mnId);

  if (mbMergeDetected || mbLoopDetected) {
    return true;
  }

  mpCurrentKF->SetErase();
  mpCurrentKF->mbCurrentPlaceRecognition = false;

  return false;
}

bool LoopClosing::DetectAndReffineSim3FromLastKF(
  KeyFrame*               pCurrentKF,
  KeyFrame*               pMatchedKF,
  g2o::Sim3&              gScw,
  int&                    nNumProjMatches,
  std::vector<MapPoint*>& vpMPs,
  std::vector<MapPoint*>& vpMatchedMPs
) {
  std::set<MapPoint*> spAlreadyMatchedMPs;
  nNumProjMatches = FindMatchesByProjection(
    pCurrentKF,
    pMatchedKF,
    gScw,
    spAlreadyMatchedMPs,
    vpMPs,
    vpMatchedMPs
  );

  int nProjMatches    = 30;
  int nProjOptMatches = 50;
  int nProjMatchesRep = 100;

  if (nNumProjMatches >= nProjMatches) {
    Sophus::SE3d                mTwm = pMatchedKF->GetPoseInverse().cast<double>();
    g2o::Sim3                   gSwm(mTwm.unit_quaternion(), mTwm.translation(), 1.0);
    g2o::Sim3                   gScm = gScw * gSwm;
    Eigen::Matrix<double, 7, 7> mHessian7x7;

    bool bFixedScale = mbFixScale; // TODO CHECK; Solo para el monocular inertial
    if (mpTracker->mSensor == Sensor::InertialMonocular && !pCurrentKF->GetMap()->GetIniertialBA2()) {
      bFixedScale = false;
    }
    int numOptMatches = Optimizer::OptimizeSim3(
      mpCurrentKF,
      pMatchedKF,
      vpMatchedMPs,
      gScm,
      10,
      bFixedScale,
      mHessian7x7,
      true
    );

    if (numOptMatches > nProjOptMatches) {
      g2o::Sim3 gScw_estimation(gScw.rotation(), gScw.translation(), 1.0);

      std::vector<MapPoint*> vpMatchedMP;
      vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));

      nNumProjMatches = FindMatchesByProjection(
        pCurrentKF,
        pMatchedKF,
        gScw_estimation,
        spAlreadyMatchedMPs,
        vpMPs,
        vpMatchedMPs
      );
      if (nNumProjMatches >= nProjMatchesRep) {
        gScw = gScw_estimation;
        return true;
      }
    }
  }
  return false;
}

bool LoopClosing::DetectCommonRegionsFromBoW(
  std::vector<KeyFrame*>& vpBowCand,
  KeyFrame*&              pMatchedKF2,
  KeyFrame*&              pLastCurrentKF,
  g2o::Sim3&              g2oScw,
  int&                    nNumCoincidences,
  std::vector<MapPoint*>& vpMPs,
  std::vector<MapPoint*>& vpMatchedMPs
) {
  int nBoWMatches     = 20;
  int nBoWInliers     = 15;
  int nSim3Inliers    = 20;
  int nProjMatches    = 50;
  int nProjOptMatches = 80;

  std::set<KeyFrame*> spConnectedKeyFrames = mpCurrentKF->GetConnectedKeyFrames();

  int nNumCovisibles = 10;

  ORBmatcher matcherBoW(0.9, true);
  ORBmatcher matcher(0.75, true);

  // Varibles to select the best numbe
  KeyFrame*              pBestMatchedKF;
  int                    nBestMatchesReproj   = 0;
  int                    nBestNumCoindicendes = 0;
  g2o::Sim3              g2oBestScw;
  std::vector<MapPoint*> vpBestMapPoints;
  std::vector<MapPoint*> vpBestMatchedMapPoints;

  int              numCandidates = vpBowCand.size();
  std::vector<int> vnStage(numCandidates, 0);
  std::vector<int> vnMatchesStage(numCandidates, 0);

  int index = 0;

  for (KeyFrame* const candidate : vpBowCand) {
    if (!candidate || candidate->isBad()) {
      continue;
    }

    // Current KF against KF with covisibles version
    std::vector<KeyFrame*> vpCovKFi = candidate->GetBestCovisibilityKeyFrames(nNumCovisibles);
    if (vpCovKFi.empty()) {
      _logger->debug("Empty covisible list");
      vpCovKFi.push_back(candidate);
    } else {
      vpCovKFi.push_back(vpCovKFi[0]);
      vpCovKFi[0] = candidate;
    }

    bool bAbortByNearKF = std::ranges::any_of(vpCovKFi, [&](KeyFrame* const neighbor) {
      return spConnectedKeyFrames.find(neighbor) != spConnectedKeyFrames.end();
    });
    if (bAbortByNearKF) {
      // Skip due to close/near key frame
      continue;
    }

    std::vector<std::vector<MapPoint*>> vvpMatchedMPs;
    vvpMatchedMPs.resize(vpCovKFi.size());
    std::set<MapPoint*> spMatchedMPi;
    int                 numBoWMatches = 0;

    KeyFrame* pMostBoWMatchesKF  = candidate;
    int       nMostBoWNumMatches = 0;

    std::vector<MapPoint*> vpMatchedPoints = std::vector<MapPoint*>(
      mpCurrentKF->GetMapPointMatches().size(),
      static_cast<MapPoint*>(NULL)
    );
    std::vector<KeyFrame*> vpKeyFrameMatchedMP = std::vector<KeyFrame*>(
      mpCurrentKF->GetMapPointMatches().size(),
      static_cast<KeyFrame*>(NULL)
    );

    int nIndexMostBoWMatchesKF = 0;
    for (int j = 0; j < vpCovKFi.size(); ++j) {
      if (!vpCovKFi[j] || vpCovKFi[j]->isBad()) {
        continue;
      }

      int num = matcherBoW.SearchByBoW(mpCurrentKF, vpCovKFi[j], vvpMatchedMPs[j]);
      if (num > nMostBoWNumMatches) {
        nMostBoWNumMatches     = num;
        nIndexMostBoWMatchesKF = j;
      }
    }

    for (int j = 0; j < vpCovKFi.size(); ++j) {
      for (int k = 0; k < vvpMatchedMPs[j].size(); ++k) {
        MapPoint* pMPi_j = vvpMatchedMPs[j][k];
        if (!pMPi_j || pMPi_j->isBad()) {
          continue;
        }

        if (spMatchedMPi.find(pMPi_j) == spMatchedMPi.end()) {
          spMatchedMPi.insert(pMPi_j);
          numBoWMatches++;

          vpMatchedPoints[k]     = pMPi_j;
          vpKeyFrameMatchedMP[k] = vpCovKFi[j];
        }
      }
    }

    // pMostBoWMatchesKF = vpCovKFi[pMostBoWMatchesKF];

    if (numBoWMatches >= nBoWMatches) { // TODO pick a good threshold
      // Geometric validation
      bool bFixedScale = mbFixScale;
      if (mpTracker->mSensor == Sensor::InertialMonocular && !mpCurrentKF->GetMap()->GetIniertialBA2()) {
        bFixedScale = false;
      }

      Sim3Solver solver = Sim3Solver(
        mpCurrentKF,
        pMostBoWMatchesKF,
        vpMatchedPoints,
        bFixedScale,
        vpKeyFrameMatchedMP
      );
      solver.SetRansacParameters(0.99, nBoWInliers, 300); // at least 15 inliers

      bool              bNoMore = false;
      std::vector<bool> vbInliers;
      int               nInliers;
      bool              bConverge = false;
      Eigen::Matrix4f   mTcm;
      while (!bConverge && !bNoMore) {
        mTcm = solver.iterate(20, bNoMore, vbInliers, nInliers, bConverge);
        _logger->debug(
          "Solver achieve {} geometrical inliers among {} BoW matches",
          nInliers,
          nBoWInliers
        );
      }

      if (bConverge) {
        // Match by reprojection
        vpCovKFi.clear();
        vpCovKFi = pMostBoWMatchesKF->GetBestCovisibilityKeyFrames(nNumCovisibles);
        vpCovKFi.push_back(pMostBoWMatchesKF);
        std::set<KeyFrame*> spCheckKFs(vpCovKFi.begin(), vpCovKFi.end());

        _logger->debug("There are {} near key frames", vpCovKFi.size());

        std::set<MapPoint*>    spMapPoints;
        std::vector<MapPoint*> vpMapPoints;
        std::vector<KeyFrame*> vpKeyFrames;
        for (KeyFrame* const neighbor : vpCovKFi) {
          for (MapPoint* const mp : neighbor->GetMapPointMatches()) {
            if (!mp || mp->isBad()) {
              continue;
            }

            if (spMapPoints.find(mp) == spMapPoints.end()) {
              spMapPoints.insert(mp);
              vpMapPoints.push_back(mp);
              vpKeyFrames.push_back(neighbor);
            }
          }
        }

        _logger->debug("There are {} key frames which view all the map points", vpKeyFrames.size());

        g2o::Sim3 gScm(
          solver.GetEstimatedRotation().cast<double>(),
          solver.GetEstimatedTranslation().cast<double>(),
          (double)solver.GetEstimatedScale()
        );
        g2o::Sim3 gSmw(
          pMostBoWMatchesKF->GetRotation().cast<double>(),
          pMostBoWMatchesKF->GetTranslation().cast<double>(),
          1.0
        );
        g2o::Sim3     gScw = gScm * gSmw; // Similarity matrix of current from the world position
        Sophus::Sim3f mScw = Converter::toSophus(gScw);

        std::vector<MapPoint*> vpMatchedMP;
        vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));
        std::vector<KeyFrame*> vpMatchedKF;
        vpMatchedKF.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<KeyFrame*>(NULL));
        int numProjMatches = matcher.SearchByProjection(
          mpCurrentKF,
          mScw,
          vpMapPoints,
          vpKeyFrames,
          vpMatchedMP,
          vpMatchedKF,
          8,
          1.5
        );
        _logger->debug(
          "There are {} BoW matches between {} points with coarse Sim3",
          numProjMatches,
          vpMapPoints.size()
        );

        if (numProjMatches >= nProjMatches) {
          // Optimize Sim3 transformation with every matches
          Eigen::Matrix<double, 7, 7> mHessian7x7;

          bool bFixedScale = mbFixScale;
          if (mpTracker->mSensor == Sensor::InertialMonocular && !mpCurrentKF->GetMap()->GetIniertialBA2()) {
            bFixedScale = false;
          }

          int numOptMatches = Optimizer::OptimizeSim3(
            mpCurrentKF,
            candidate,
            vpMatchedMP,
            gScm,
            10,
            mbFixScale,
            mHessian7x7,
            true
          );

          if (numOptMatches >= nSim3Inliers) {
            g2o::Sim3 gSmw(
              pMostBoWMatchesKF->GetRotation().cast<double>(),
              pMostBoWMatchesKF->GetTranslation().cast<double>(),
              1.0
            );
            g2o::Sim3 gScw = gScm * gSmw; // Similarity matrix of current from the world position
            Sophus::Sim3f mScw = Converter::toSophus(gScw);

            std::vector<MapPoint*> vpMatchedMP;
            vpMatchedMP.resize(
              mpCurrentKF->GetMapPointMatches().size(),
              static_cast<MapPoint*>(NULL)
            );
            int numProjOptMatches
              = matcher.SearchByProjection(mpCurrentKF, mScw, vpMapPoints, vpMatchedMP, 5, 1.0);

            if (numProjOptMatches >= nProjOptMatches) {
              int max_x = -1, min_x = 1000000;
              int max_y = -1, min_y = 1000000;
              for (MapPoint* const mp : vpMatchedMP) {
                if (!mp || mp->isBad()) {
                  continue;
                }

                const auto& [index, _] = mp->GetIndexInKeyFrame(candidate);
                if (index >= 0) {
                  int coord_x = candidate->mvKeysUn[index].pt.x;
                  if (coord_x < min_x) {
                    min_x = coord_x;
                  }
                  if (coord_x > max_x) {
                    max_x = coord_x;
                  }
                  int coord_y = candidate->mvKeysUn[index].pt.y;
                  if (coord_y < min_y) {
                    min_y = coord_y;
                  }
                  if (coord_y > max_y) {
                    max_y = coord_y;
                  }
                }
              }

              int nNumKFs = 0;
              // vpMatchedMPs = vpMatchedMP;
              // vpMPs = vpMapPoints;
              //  Check the Sim3 transformation with the current KeyFrame covisibles
              std::vector<KeyFrame*> vpCurrentCovKFs
                = mpCurrentKF->GetBestCovisibilityKeyFrames(nNumCovisibles);

              int j = 0;
              while (nNumKFs < 3 && j < vpCurrentCovKFs.size()) {
                KeyFrame*    pKFj = vpCurrentCovKFs[j];
                Sophus::SE3d mTjc
                  = (pKFj->GetPose() * mpCurrentKF->GetPoseInverse()).cast<double>();
                g2o::Sim3              gSjc(mTjc.unit_quaternion(), mTjc.translation(), 1.0);
                g2o::Sim3              gSjw             = gSjc * gScw;
                int                    numProjMatches_j = 0;
                std::vector<MapPoint*> vpMatchedMPs_j;
                bool                   bValid = DetectCommonRegionsFromLastKF(
                  pKFj,
                  pMostBoWMatchesKF,
                  gSjw,
                  numProjMatches_j,
                  vpMapPoints,
                  vpMatchedMPs_j
                );

                if (bValid) {
                  Sophus::SE3f    Tc_w        = mpCurrentKF->GetPose();
                  Sophus::SE3f    Tw_cj       = pKFj->GetPoseInverse();
                  Sophus::SE3f    Tc_cj       = Tc_w * Tw_cj;
                  Eigen::Vector3f vector_dist = Tc_cj.translation();
                  nNumKFs++;
                }
                j++;
              }

              if (nNumKFs < 3) {
                vnStage[index]        = 8;
                vnMatchesStage[index] = nNumKFs;
              }

              if (nBestMatchesReproj < numProjOptMatches) {
                nBestMatchesReproj     = numProjOptMatches;
                nBestNumCoindicendes   = nNumKFs;
                pBestMatchedKF         = pMostBoWMatchesKF;
                g2oBestScw             = gScw;
                vpBestMapPoints        = vpMapPoints;
                vpBestMatchedMapPoints = vpMatchedMP;
              }
            }
          }
        }
      }
    }
    index++;
  }

  if (nBestMatchesReproj > 0) {
    pLastCurrentKF   = mpCurrentKF;
    nNumCoincidences = nBestNumCoindicendes;
    pMatchedKF2      = pBestMatchedKF;
    pMatchedKF2->SetNotErase();
    g2oScw       = g2oBestScw;
    vpMPs        = vpBestMapPoints;
    vpMatchedMPs = vpBestMatchedMapPoints;

    return nNumCoincidences >= 3;
  } else {
    int maxStage = -1;
    int maxMatched;
    for (int i = 0; i < vnStage.size(); ++i) {
      if (vnStage[i] > maxStage) {
        maxStage   = vnStage[i];
        maxMatched = vnMatchesStage[i];
      }
    }
  }
  return false;
}

bool LoopClosing::DetectCommonRegionsFromLastKF(
  KeyFrame*               pCurrentKF,
  KeyFrame*               pMatchedKF,
  g2o::Sim3&              gScw,
  int&                    nNumProjMatches,
  std::vector<MapPoint*>& vpMPs,
  std::vector<MapPoint*>& vpMatchedMPs
) {
  std::set<MapPoint*> spAlreadyMatchedMPs(vpMatchedMPs.begin(), vpMatchedMPs.end());
  nNumProjMatches = FindMatchesByProjection(
    pCurrentKF,
    pMatchedKF,
    gScw,
    spAlreadyMatchedMPs,
    vpMPs,
    vpMatchedMPs
  );

  int nProjMatches = 30;
  if (nNumProjMatches >= nProjMatches) {
    return true;
  }

  return false;
}

int LoopClosing::FindMatchesByProjection(
  KeyFrame*               pCurrentKF,
  KeyFrame*               pMatchedKFw,
  g2o::Sim3&              g2oScw,
  std::set<MapPoint*>&    spMatchedMPinOrigin,
  std::vector<MapPoint*>& vpMapPoints,
  std::vector<MapPoint*>& vpMatchedMapPoints
) {
  int                    nNumCovisibles = 10;
  std::vector<KeyFrame*> vpCovKFm       = pMatchedKFw->GetBestCovisibilityKeyFrames(nNumCovisibles);
  int                    nInitialCov    = vpCovKFm.size();
  vpCovKFm.push_back(pMatchedKFw);
  std::set<KeyFrame*> spCheckKFs(vpCovKFm.begin(), vpCovKFm.end());
  std::set<KeyFrame*> spCurrentCovisbles = pCurrentKF->GetConnectedKeyFrames();
  if (nInitialCov < nNumCovisibles) {
    for (int i = 0; i < nInitialCov; ++i) {
      std::vector<KeyFrame*> vpKFs     = vpCovKFm[i]->GetBestCovisibilityKeyFrames(nNumCovisibles);
      int                    nInserted = 0;
      int                    j         = 0;
      while (j < vpKFs.size() && nInserted < nNumCovisibles) {
        if(spCheckKFs.find(vpKFs[j]) == spCheckKFs.end() && spCurrentCovisbles.find(vpKFs[j]) == spCurrentCovisbles.end())
                {
          spCheckKFs.insert(vpKFs[j]);
          ++nInserted;
        }
        ++j;
      }
      vpCovKFm.insert(vpCovKFm.end(), vpKFs.begin(), vpKFs.end());
    }
  }
  std::set<MapPoint*> spMapPoints;
  vpMapPoints.clear();
  vpMatchedMapPoints.clear();
  for (KeyFrame* const neighbor : vpCovKFm) {
    for (MapPoint* const mp : neighbor->GetMapPointMatches()) {
      if (!mp || mp->isBad()) {
        continue;
      }

      if (spMapPoints.find(mp) == spMapPoints.end()) {
        spMapPoints.insert(mp);
        vpMapPoints.push_back(mp);
      }
    }
  }

  Sophus::Sim3f mScw = Converter::toSophus(g2oScw);
  ORBmatcher    matcher(0.9, true);

  vpMatchedMapPoints.resize(pCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));
  int num_matches
    = matcher.SearchByProjection(pCurrentKF, mScw, vpMapPoints, vpMatchedMapPoints, 3, 1.5);

  return num_matches;
}

void LoopClosing::CorrectLoop() {
  _logger->info("Loop detected");

  // Send a stop signal to Local Mapping
  // Avoid new keyframes are inserted while correcting the loop
  _logger->info("CorrectLoop: requesting to stop local mapping...");
  mpLocalMapper->RequestStop();
  mpLocalMapper->EmptyQueue(); // Proccess keyframes in the queue

  // If a Global Bundle Adjustment is running, abort it
  if (isRunningGBA()) {
    _logger->info("CorrectLoop: global bundle adjustment is running, stop it now");
    std::unique_lock<std::mutex> lock(mMutexGBA);
    mbStopGBA = true;

    mnFullBAIdx++;

    if (mpThreadGBA) {
      mpThreadGBA->detach();
      delete mpThreadGBA;
    }
    _logger->info("CorrectLoop: global bundle adjustment stopped");
  }

  // Wait until Local Mapping has effectively stopped
  while (!mpLocalMapper->isStopped()) {
    usleep(1000);
  }
  _logger->info("Local mapping stopped");

  // Ensure current keyframe is updated
  // assert(mpCurrentKF->GetMap()->CheckEssentialGraph());
  mpCurrentKF->UpdateConnections();
  // assert(mpCurrentKF->GetMap()->CheckEssentialGraph());

  // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by
  // propagation
  mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
  mvpCurrentConnectedKFs.push_back(mpCurrentKF);

  _logger->info("CorrectLoop: there are {} connected key frames", mvpCurrentConnectedKFs.size());

  KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
  CorrectedSim3[mpCurrentKF] = mg2oLoopScw;
  Sophus::SE3f Twc           = mpCurrentKF->GetPoseInverse();
  Sophus::SE3f Tcw           = mpCurrentKF->GetPose();
  g2o::Sim3    g2oScw(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>(), 1.0);
  NonCorrectedSim3[mpCurrentKF] = g2oScw;

  // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
  Sophus::SE3d correctedTcw(
    mg2oLoopScw.rotation(),
    mg2oLoopScw.translation() / mg2oLoopScw.scale()
  );
  mpCurrentKF->SetPose(correctedTcw.cast<float>());
  _logger->info("CorrectLoop: key frame {} pose updated", mpCurrentKF->mnId);

  Map* pLoopMap = mpCurrentKF->GetMap();

#ifdef REGISTER_TIMES
  /*KeyFrame* pKF = mpCurrentKF;
  int numKFinLoop = 0;
  while (pKF && pKF->mnId > mpLoopMatchedKF->mnId) {
    pKF = pKF->GetParent();
    numKFinLoop += 1;
  }
  vnLoopKFs.push_back(numKFinLoop);*/

  std::chrono::steady_clock::time_point time_StartFusion = std::chrono::steady_clock::now();
#endif

  {
    // Get Map Mutex
    std::unique_lock<std::mutex> lock(pLoopMap->mMutexMapUpdate);

    const bool bImuInit = pLoopMap->isImuInitialized();

    _logger->info(
      "CorrectLoop: starting pose correction for {} connected keyframes",
      mvpCurrentConnectedKFs.size() - 1
    ); // -1 to exclude current key frame
    for (KeyFrame* const connected : mvpCurrentConnectedKFs) {
      if (connected != mpCurrentKF) {
        Sophus::SE3f Tiw = connected->GetPose();
        Sophus::SE3d Tic = (Tiw * Twc).cast<double>();
        g2o::Sim3    g2oSic(Tic.unit_quaternion(), Tic.translation(), 1.0);
        g2o::Sim3    g2oCorrectedSiw = g2oSic * mg2oLoopScw;
        // Pose corrected with the Sim3 of the loop closure
        CorrectedSim3[connected] = g2oCorrectedSiw;

        // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
        Sophus::SE3d correctedTiw(
          g2oCorrectedSiw.rotation(),
          g2oCorrectedSiw.translation() / g2oCorrectedSiw.scale()
        );
        connected->SetPose(correctedTiw.cast<float>());

        // Pose without correction
        g2o::Sim3 g2oSiw(
          Tiw.unit_quaternion().cast<double>(),
          Tiw.translation().cast<double>(),
          1.0
        );
        NonCorrectedSim3[connected] = g2oSiw;
      }
    }

    // Correct all MapPoints observed by current keyframe and neighbors, so that they align with the
    // other side of the loop
    _logger->info(
      "CorrectLoop: correcting map points observed by current key frames and it neighbors..."
    );
    for (const auto& [connected, g2oCorrectedSiw] : CorrectedSim3) {
      g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

      g2o::Sim3 g2oSiw = NonCorrectedSim3[connected];

      // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
      /*Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(),g2oCorrectedSiw.translation() /
      g2oCorrectedSiw.scale()); pKFi->SetPose(correctedTiw.cast<float>());*/

      for (MapPoint* const mp : connected->GetMapPointMatches()) {
        if (!mp) {
          continue;
        }
        if (mp->isBad()) {
          continue;
        }
        if (mp->mnCorrectedByKF == mpCurrentKF->mnId) {
          continue;
        }

        // Project with non-corrected pose and project back with corrected pose
        Eigen::Vector3d P3Dw             = mp->GetWorldPos().cast<double>();
        Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(P3Dw));

        mp->SetWorldPos(eigCorrectedP3Dw.cast<float>());
        mp->mnCorrectedByKF      = mpCurrentKF->mnId;
        mp->mnCorrectedReference = connected->mnId;
        mp->UpdateNormalAndDepth();
      }

      // Correct velocity according to orientation correction
      if (bImuInit) {
        Eigen::Quaternionf Rcor
          = (g2oCorrectedSiw.rotation().inverse() * g2oSiw.rotation()).cast<float>();
        connected->SetVelocity(Rcor * connected->GetVelocity());
      }

      // Make sure connections are updated
      connected->UpdateConnections();
    }
    // TODO Check this index increasement
    mpAtlas->GetCurrentMap()->IncreaseChangeIndex();

    // Start Loop Fusion
    // Update matched map points and replace if duplicated
    _logger->info("CorrectLoop: updating these map points by replacing duplicates...");
    for (std::size_t i = 0; i < mvpLoopMatchedMPs.size(); i++) {
      if (mvpLoopMatchedMPs[i]) {
        MapPoint* pLoopMP = mvpLoopMatchedMPs[i];
        MapPoint* pCurMP  = mpCurrentKF->GetMapPoint(i);
        if (pCurMP) {
          pCurMP->Replace(pLoopMP);
        } else {
          mpCurrentKF->AddMapPoint(pLoopMP, i);
          pLoopMP->AddObservation(mpCurrentKF, i);
          pLoopMP->ComputeDistinctiveDescriptors();
        }
      }
    }
  }

  // Project MapPoints observed in the neighborhood of the loop keyframe
  // into the current keyframe and neighbors using corrected poses.
  // Fuse duplications.
  _logger->info(
    "CorrectLoop: fusing map points observed by neighborhood of loop key frame into current key "
    "frames and its neighbors..."
  );
  SearchAndFuse(CorrectedSim3, mvpLoopMapPoints);

  // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides
  // of the loop
  std::map<KeyFrame*, std::set<KeyFrame*>> LoopConnections;

  _logger->info("CorrectLoop: updating connections in covisibility graph...");
  for (KeyFrame* const connected : mvpCurrentConnectedKFs) {
    std::vector<KeyFrame*> vpPreviousNeighbors = connected->GetVectorCovisibleKeyFrames();

    // Update connections. Detect new links.
    connected->UpdateConnections();
    LoopConnections[connected] = connected->GetConnectedKeyFrames();
    for (KeyFrame* const kf : vpPreviousNeighbors) {
      LoopConnections[connected].erase(kf);
    }
    for (KeyFrame* const kf : mvpCurrentConnectedKFs) {
      LoopConnections[connected].erase(kf);
    }
  }

  // Optimize graph
  bool bFixedScale = mbFixScale;
  // TODO CHECK; Solo para el monocular inertial
  if (mpTracker->mSensor == Sensor::InertialMonocular && !mpCurrentKF->GetMap()->GetIniertialBA2()) {
    bFixedScale = false;
  }

#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_EndFusion = std::chrono::steady_clock::now();

  double timeFusion = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                        time_EndFusion - time_StartFusion
  )
                        .count();
  vdLoopFusion_ms.push_back(timeFusion);
#endif
  if (pLoopMap->IsInertial() && pLoopMap->isImuInitialized()) {
    _logger->info("Optimizing essential graph 4DoF...");
    Optimizer::OptimizeEssentialGraph4DoF(
      pLoopMap,
      mpLoopMatchedKF,
      mpCurrentKF,
      NonCorrectedSim3,
      CorrectedSim3,
      LoopConnections
    );
  } else {
    _logger->info("Optimizing essential graph...");
    Optimizer::OptimizeEssentialGraph(
      pLoopMap,
      mpLoopMatchedKF,
      mpCurrentKF,
      NonCorrectedSim3,
      CorrectedSim3,
      LoopConnections,
      bFixedScale
    );
  }
#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_EndOpt = std::chrono::steady_clock::now();
  double timeOptEss = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                        time_EndOpt - time_EndFusion
  )
                        .count();
  vdLoopOptEss_ms.push_back(timeOptEss);
#endif

  mpAtlas->InformNewBigChange();

  // Add loop edge
  mpLoopMatchedKF->AddLoopEdge(mpCurrentKF);
  mpCurrentKF->AddLoopEdge(mpLoopMatchedKF);

  // Launch a new thread to perform Global Bundle Adjustment (Only if few keyframes, if not it would
  // take too much time)
  if (!pLoopMap->isImuInitialized() || (pLoopMap->KeyFramesInMap() < 200 && mpAtlas->CountMaps() == 1)) {
    mbRunningGBA    = true;
    mbFinishedGBA   = false;
    mbStopGBA       = false;
    mnCorrectionGBA = mnNumCorrection;

    mpThreadGBA
      = new std::thread(&LoopClosing::RunGlobalBundleAdjustment, this, pLoopMap, mpCurrentKF->mnId);
    _logger->info("Global bundle adjustment thread running back");
  }

  _logger->info("CorrectLoop: loop corrected, releasing local mapping...");
  mpLocalMapper->Release();

  mLastLoopKFid = mpCurrentKF->mnId; // TODO old varible, it is not use in the new algorithm
}

void LoopClosing::MergeLocal() {
  _logger->info("MergeLocal: starting to merge in local without IMU data");

  int numTemporalKFs = 25; // Temporal KFs in the local window if the map is inertial.

  // Relationship to rebuild the essential graph, it is used two times, first in the local window
  // and later in the rest of the map
  KeyFrame* pNewChild;
  KeyFrame* pNewParent;

  std::vector<KeyFrame*> vpLocalCurrentWindowKFs;
  std::vector<KeyFrame*> vpMergeConnectedKFs;

  // Flag that is true only when we stopped a running BA, in this case we need relaunch at the end
  // of the merge
  bool bRelaunchBA = false;

  //  If a Global Bundle Adjustment is running, abort it
  if (isRunningGBA()) {
    _logger->info("MergeLocal: stopping global bundle adjustment...");
    std::unique_lock<std::mutex> lock(mMutexGBA);
    mbStopGBA = true;

    mnFullBAIdx++;

    if (mpThreadGBA) {
      mpThreadGBA->detach();
      delete mpThreadGBA;
    }
    bRelaunchBA = true;
  }

  _logger->info("MergeLocal: requesting to stop local mapping...");
  mpLocalMapper->RequestStop();
  // Wait until Local Mapping has effectively stopped
  while (!mpLocalMapper->isStopped()) {
    usleep(1000);
  }

  mpLocalMapper->EmptyQueue();

  // Merge map will become in the new active map with the local window of KFs and MPs from the
  // current map. Later, the elements of the current map will be transform to the new active map
  // reference, in order to keep real time tracking
  Map* pCurrentMap = mpCurrentKF->GetMap();
  Map* pMergeMap   = mpMergeMatchedKF->GetMap();
  _logger->info(
    "MergeLocal: merging local active map {} and non-active map {}",
    pCurrentMap->GetId(),
    pMergeMap->GetId()
  );

#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_StartMerge = std::chrono::steady_clock::now();
#endif

  // Ensure current keyframe is updated
  mpCurrentKF->UpdateConnections();

  // Get the current KF and its neighbors(visual->covisibles; inertial->temporal+covisibles)
  std::set<KeyFrame*> spLocalWindowKFs;
  // Get MPs in the welding area from the current map
  std::set<MapPoint*> spLocalWindowMPs;
  if (pCurrentMap->IsInertial() && pMergeMap->IsInertial()) { // TODO Check the correct
                                                              // initialization
    KeyFrame* pKFi      = mpCurrentKF;
    int       nInserted = 0;
    while (pKFi && nInserted < numTemporalKFs) {
      spLocalWindowKFs.insert(pKFi);
      pKFi = mpCurrentKF->mPrevKF;
      nInserted++;

      std::set<MapPoint*> spMPi = pKFi->GetMapPoints();
      spLocalWindowMPs.insert(spMPi.begin(), spMPi.end());
    }

    pKFi = mpCurrentKF->mNextKF;
    while (pKFi) {
      spLocalWindowKFs.insert(pKFi);

      std::set<MapPoint*> spMPi = pKFi->GetMapPoints();
      spLocalWindowMPs.insert(spMPi.begin(), spMPi.end());

      pKFi = mpCurrentKF->mNextKF;
    }
  } else {
    spLocalWindowKFs.insert(mpCurrentKF);
  }

  std::vector<KeyFrame*> vpCovisibleKFs = mpCurrentKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
  spLocalWindowKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());
  spLocalWindowKFs.insert(mpCurrentKF);
  const int nMaxTries = 5;
  int       nNumTries = 0;
  while (spLocalWindowKFs.size() < numTemporalKFs && nNumTries < nMaxTries) {
    std::vector<KeyFrame*> vpNewCovKFs;
    vpNewCovKFs.empty();
    for (KeyFrame* const local : spLocalWindowKFs) {
      for (KeyFrame* const neighbor : local->GetBestCovisibilityKeyFrames(numTemporalKFs / 2)) {
        if (neighbor && !neighbor->isBad() && spLocalWindowKFs.find(neighbor) == spLocalWindowKFs.end()) {
          vpNewCovKFs.push_back(neighbor);
        }
      }
    }

    spLocalWindowKFs.insert(vpNewCovKFs.begin(), vpNewCovKFs.end());
    nNumTries++;
  }

  for (KeyFrame* const local : spLocalWindowKFs) {
    if (!local || local->isBad()) {
      continue;
    }

    std::set<MapPoint*> spMPs = local->GetMapPoints();
    spLocalWindowMPs.insert(spMPs.begin(), spMPs.end());
  }

  std::set<KeyFrame*> spMergeConnectedKFs;
  if (pCurrentMap->IsInertial() && pMergeMap->IsInertial()) { // TODO Check the correct
                                                              // initialization
    KeyFrame* pKFi      = mpMergeMatchedKF;
    int       nInserted = 0;
    while (pKFi && nInserted < numTemporalKFs / 2) {
      spMergeConnectedKFs.insert(pKFi);
      pKFi = mpCurrentKF->mPrevKF;
      nInserted++;
    }

    pKFi = mpMergeMatchedKF->mNextKF;
    while (pKFi && nInserted < numTemporalKFs) {
      spMergeConnectedKFs.insert(pKFi);
      pKFi = mpCurrentKF->mNextKF;
    }
  } else {
    spMergeConnectedKFs.insert(mpMergeMatchedKF);
  }
  vpCovisibleKFs = mpMergeMatchedKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
  spMergeConnectedKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());
  spMergeConnectedKFs.insert(mpMergeMatchedKF);
  nNumTries = 0;
  while (spMergeConnectedKFs.size() < numTemporalKFs && nNumTries < nMaxTries) {
    std::vector<KeyFrame*> vpNewCovKFs;
    for (KeyFrame* const merged : spMergeConnectedKFs) {
      for (KeyFrame* const neighbor : merged->GetBestCovisibilityKeyFrames(numTemporalKFs / 2)) {
        if (neighbor && !neighbor->isBad() && spMergeConnectedKFs.find(neighbor) == spMergeConnectedKFs.end()) {
          vpNewCovKFs.push_back(neighbor);
        }
      }
    }

    spMergeConnectedKFs.insert(vpNewCovKFs.begin(), vpNewCovKFs.end());
    nNumTries++;
  }

  std::set<MapPoint*> spMapPointMerge;
  for (KeyFrame* const merged : spMergeConnectedKFs) {
    std::set<MapPoint*> vpMPs = merged->GetMapPoints();
    spMapPointMerge.insert(vpMPs.begin(), vpMPs.end());
  }

  std::vector<MapPoint*> vpCheckFuseMapPoint;
  vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
  std::copy(
    spMapPointMerge.begin(),
    spMapPointMerge.end(),
    std::back_inserter(vpCheckFuseMapPoint)
  );

  Sophus::SE3d Twc = mpCurrentKF->GetPoseInverse().cast<double>();
  g2o::Sim3    g2oNonCorrectedSwc(Twc.unit_quaternion(), Twc.translation(), 1.0);
  g2o::Sim3    g2oNonCorrectedScw = g2oNonCorrectedSwc.inverse();
  g2o::Sim3    g2oCorrectedScw    = mg2oMergeScw; // TODO Check the transformation

  KeyFrameAndPose vCorrectedSim3, vNonCorrectedSim3;
  vCorrectedSim3[mpCurrentKF]    = g2oCorrectedScw;
  vNonCorrectedSim3[mpCurrentKF] = g2oNonCorrectedScw;

#ifdef REGISTER_TIMES
  vnMergeKFs.push_back(spLocalWindowKFs.size() + spMergeConnectedKFs.size());
  vnMergeMPs.push_back(spLocalWindowMPs.size() + spMapPointMerge.size());
#endif
  for (KeyFrame* const local : spLocalWindowKFs) {
    if (!local || local->isBad()) {
      continue;
    }

    if (local->GetMap() != pCurrentMap) {
      _logger->warn("MergeLocal: key frame {} belongs to unexpected map", local->mnId);
    }

    g2o::Sim3 g2oCorrectedSiw;

    if (local != mpCurrentKF) {
      Sophus::SE3d Tiw = (local->GetPose()).cast<double>();
      g2o::Sim3    g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
      // Pose without correction
      vNonCorrectedSim3[local] = g2oSiw;

      Sophus::SE3d Tic = Tiw * Twc;
      g2o::Sim3    g2oSic(Tic.unit_quaternion(), Tic.translation(), 1.0);
      g2oCorrectedSiw       = g2oSic * mg2oMergeScw;
      vCorrectedSim3[local] = g2oCorrectedSiw;
    } else {
      g2oCorrectedSiw = g2oCorrectedScw;
    }
    local->mTcwMerge = local->GetPose();

    // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
    double s       = g2oCorrectedSiw.scale();
    local->mfScale = s;
    Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(), g2oCorrectedSiw.translation() / s);

    local->mTcwMerge = correctedTiw.cast<float>();

    if (pCurrentMap->isImuInitialized()) {
      Eigen::Quaternionf Rcor
        = (g2oCorrectedSiw.rotation().inverse() * vNonCorrectedSim3[local].rotation())
            .cast<float>();
      local->mVwbMerge = Rcor * local->GetVelocity();
    }

    // TODO DEBUG to know which are the KFs that had been moved to the other map
  }

  int numPointsWithCorrection = 0;

  // for(MapPoint* pMPi : spLocalWindowMPs)
  auto itMP = spLocalWindowMPs.begin();
  while (itMP != spLocalWindowMPs.end()) {
    MapPoint* pMPi = *itMP;
    if (!pMPi || pMPi->isBad()) {
      itMP = spLocalWindowMPs.erase(itMP);
      continue;
    }

    KeyFrame* pKFref = pMPi->GetReferenceKeyFrame();
    if (vCorrectedSim3.find(pKFref) == vCorrectedSim3.end()) {
      itMP = spLocalWindowMPs.erase(itMP);
      numPointsWithCorrection++;
      continue;
    }
    g2o::Sim3 g2oCorrectedSwi    = vCorrectedSim3[pKFref].inverse();
    g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

    // Project with non-corrected pose and project back with corrected pose
    Eigen::Vector3d    P3Dw             = pMPi->GetWorldPos().cast<double>();
    Eigen::Vector3d    eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(P3Dw));
    Eigen::Quaterniond Rcor = g2oCorrectedSwi.rotation() * g2oNonCorrectedSiw.rotation();

    pMPi->mPosMerge          = eigCorrectedP3Dw.cast<float>();
    pMPi->mNormalVectorMerge = Rcor.cast<float>() * pMPi->GetNormal();

    itMP++;
  }
  /*if (numPointsWithCorrection > 0) {
    _logger->info(
      "[Merge]: Removed {} points from Ma (reference KF not in welding area)",
      numPointsWithCorrection
    );
    _logger->info("[Merge]: Ma now contains {} points after correction", spLocalWindowMPs.size());
  }*/

  _logger->info("MergeLocal: updating current map...");
  {
    std::unique_lock<std::mutex> currentLock(pCurrentMap->mMutexMapUpdate
    ); // We update the current map with the Merge information
    std::unique_lock<std::mutex> mergeLock(pMergeMap->mMutexMapUpdate
    ); // We remove the Kfs and MPs in the merged area from the old map

    for (KeyFrame* const local : spLocalWindowKFs) {
      if (!local || local->isBad()) {
        continue;
      }

      local->mTcwBefMerge = local->GetPose();
      local->mTwcBefMerge = local->GetPoseInverse();
      local->SetPose(local->mTcwMerge);

      // Make sure connections are updated
      local->UpdateMap(pMergeMap);
      local->mnMergeCorrectedForKF = mpCurrentKF->mnId;
      pMergeMap->AddKeyFrame(local);
      pCurrentMap->EraseKeyFrame(local);

      if (pCurrentMap->isImuInitialized()) {
        local->SetVelocity(local->mVwbMerge);
      }
    }

    for (MapPoint* const local : spLocalWindowMPs) {
      if (!local || local->isBad()) {
        continue;
      }

      local->SetWorldPos(local->mPosMerge);
      local->SetNormalVector(local->mNormalVectorMerge);
      local->UpdateMap(pMergeMap);
      pMergeMap->AddMapPoint(local);
      pCurrentMap->EraseMapPoint(local);
    }

    mpAtlas->ChangeMap(pMergeMap);
    mpAtlas->SetMapBad(pCurrentMap);
    pMergeMap->IncreaseChangeIndex();
    // TODO for debug
    pMergeMap->ChangeId(pCurrentMap->GetId());
  }

  // Rebuild the essential graph in the local window
  _logger->info("MergeLocal: updating essential graph...");
  pCurrentMap->GetOriginKF()->SetFirstConnection(false);
  pNewChild  = mpCurrentKF->GetParent(); // Old parent, it will be the new child of this KF
  pNewParent = mpCurrentKF; // Old child, now it will be the parent of its own parent(we need
                            // eliminate this KF from children list in its old parent)
  mpCurrentKF->ChangeParent(mpMergeMatchedKF);
  while (pNewChild) {
    pNewChild->EraseChild(pNewParent
    ); // We remove the relation between the old parent and the new for avoid loop
    KeyFrame* pOldParent = pNewChild->GetParent();

    pNewChild->ChangeParent(pNewParent);

    pNewParent = pNewChild;
    pNewChild  = pOldParent;
  }

  // Update the connections between the local window
  mpMergeMatchedKF->UpdateConnections();

  vpMergeConnectedKFs = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
  vpMergeConnectedKFs.push_back(mpMergeMatchedKF);
  // vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
  // std::copy(spMapPointMerge.begin(), spMapPointMerge.end(),
  // std::back_inserter(vpCheckFuseMapPoint));

  // Project MapPoints observed in the neighborhood of the merge keyframe
  // into the current keyframe and neighbors using corrected poses.
  // Fuse duplications.
  SearchAndFuse(vCorrectedSim3, vpCheckFuseMapPoint);

  // Update connectivity
  std::ranges::for_each(
    spLocalWindowKFs | std::views::filter([](KeyFrame* const kf) {
      return kf && !kf->isBad();
    }),
    [](KeyFrame* const kf) {
      kf->UpdateConnections();
    }
  );
  std::ranges::for_each(
    spMergeConnectedKFs | std::views::filter([](KeyFrame* const kf) {
      return kf && !kf->isBad();
    }),
    [](KeyFrame* const kf) {
      kf->UpdateConnections();
    }
  );

  _logger->info("MergeLocal: starting welding bundle adjustment...");

#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_StartWeldingBA = std::chrono::steady_clock::now();
  double timeMergeMaps = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                           time_StartWeldingBA - time_StartMerge
  )
                           .count();
  vdMergeMaps_ms.push_back(timeMergeMaps);
#endif

  bool bStop = false;
  vpLocalCurrentWindowKFs.clear();
  vpMergeConnectedKFs.clear();
  std::copy(
    spLocalWindowKFs.begin(),
    spLocalWindowKFs.end(),
    std::back_inserter(vpLocalCurrentWindowKFs)
  );
  std::copy(
    spMergeConnectedKFs.begin(),
    spMergeConnectedKFs.end(),
    std::back_inserter(vpMergeConnectedKFs)
  );
  if (IsInertialBased(mpTracker->mSensor)) {
    Optimizer::MergeInertialBA(mpCurrentKF, mpMergeMatchedKF, &bStop, pCurrentMap, vCorrectedSim3);
  } else {
    Optimizer::LocalBundleAdjustment(
      mpCurrentKF,
      vpLocalCurrentWindowKFs,
      vpMergeConnectedKFs,
      &bStop
    );
  }

#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_EndWeldingBA = std::chrono::steady_clock::now();
  double timeWeldingBA = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                           time_EndWeldingBA - time_StartWeldingBA
  )
                           .count();
  vdWeldingBA_ms.push_back(timeWeldingBA);
#endif
  _logger->info("MergeLocal: welding bundle adjustment finished");

  // Loop closed. Release Local Mapping.
  mpLocalMapper->Release();

  // Update the non critical area from the current map to the merged map
  std::vector<KeyFrame*> vpCurrentMapKFs = pCurrentMap->GetAllKeyFrames();
  std::vector<MapPoint*> vpCurrentMapMPs = pCurrentMap->GetAllMapPoints();

  if (vpCurrentMapKFs.size() == 0) {
  } else {
    if (mpTracker->mSensor == Sensor::Monocular) {
      std::unique_lock<std::mutex> currentLock(pCurrentMap->mMutexMapUpdate
      ); // We update the current map with the Merge information

      for (KeyFrame* const current : vpCurrentMapKFs) {
        if (!current || current->isBad() || current->GetMap() != pCurrentMap) {
          continue;
        }

        g2o::Sim3 g2oCorrectedSiw;

        Sophus::SE3d Tiw = (current->GetPose()).cast<double>();
        g2o::Sim3    g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
        // Pose without correction
        vNonCorrectedSim3[current] = g2oSiw;

        Sophus::SE3d Tic = Tiw * Twc;
        g2o::Sim3    g2oSim(Tic.unit_quaternion(), Tic.translation(), 1.0);
        g2oCorrectedSiw         = g2oSim * mg2oMergeScw;
        vCorrectedSim3[current] = g2oCorrectedSiw;

        // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
        double s = g2oCorrectedSiw.scale();

        current->mfScale = s;

        Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(), g2oCorrectedSiw.translation() / s);

        current->mTcwBefMerge = current->GetPose();
        current->mTwcBefMerge = current->GetPoseInverse();

        current->SetPose(correctedTiw.cast<float>());

        if (pCurrentMap->isImuInitialized()) {
          Eigen::Quaternionf Rcor
            = (g2oCorrectedSiw.rotation().inverse() * vNonCorrectedSim3[current].rotation())
                .cast<float>();
          current->SetVelocity(Rcor * current->GetVelocity()); // TODO: should add here scale s
        }
      }
      for (MapPoint* const current : vpCurrentMapMPs) {
        if (!current || current->isBad() || current->GetMap() != pCurrentMap) {
          continue;
        }

        KeyFrame* pKFref             = current->GetReferenceKeyFrame();
        g2o::Sim3 g2oCorrectedSwi    = vCorrectedSim3[pKFref].inverse();
        g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

        // Project with non-corrected pose and project back with corrected pose
        Eigen::Vector3d P3Dw             = current->GetWorldPos().cast<double>();
        Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(P3Dw));
        current->SetWorldPos(eigCorrectedP3Dw.cast<float>());

        current->UpdateNormalAndDepth();
      }
    }

    _logger->info("MergeLocal: requesting to stop local mapping...");
    mpLocalMapper->RequestStop();
    // Wait until Local Mapping has effectively stopped
    while (!mpLocalMapper->isStopped()) {
      usleep(1000);
    }
    _logger->info("MergeLocal: local mapping stopped");

    // Optimize graph (and update the loop position for each element form the begining to the end)
    if (mpTracker->mSensor != Sensor::Monocular) {
      Optimizer::OptimizeEssentialGraph(
        mpCurrentKF,
        vpMergeConnectedKFs,
        vpLocalCurrentWindowKFs,
        vpCurrentMapKFs,
        vpCurrentMapMPs
      );
    }

    {
      // Get Merge Map Mutex
      std::unique_lock<std::mutex> currentLock(pCurrentMap->mMutexMapUpdate
      ); // We update the current map with the Merge information
      std::unique_lock<std::mutex> mergeLock(pMergeMap->mMutexMapUpdate
      ); // We remove the Kfs and MPs in the merged area from the old map

      // Merge outside key frames
      for (KeyFrame* const current : vpCurrentMapKFs) {
        if (!current || current->isBad() || current->GetMap() != pCurrentMap) {
          continue;
        }

        // Make sure connections are updated
        current->UpdateMap(pMergeMap);
        pMergeMap->AddKeyFrame(current);
        pCurrentMap->EraseKeyFrame(current);
      }

      for (MapPoint* const current : vpCurrentMapMPs) {
        if (!current || current->isBad()) {
          continue;
        }

        current->UpdateMap(pMergeMap);
        pMergeMap->AddMapPoint(current);
        pCurrentMap->EraseMapPoint(current);
      }
    }
  }

#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_EndOptEss = std::chrono::steady_clock::now();
  double timeOptEss = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                        time_EndOptEss - time_EndWeldingBA
  )
                        .count();
  vdMergeOptEss_ms.push_back(timeOptEss);
#endif

  _logger->info("MergeLocal: releasing local mapping...");
  mpLocalMapper->Release();

  if (bRelaunchBA && (!pCurrentMap->isImuInitialized() || (pCurrentMap->KeyFramesInMap() < 200 && mpAtlas->CountMaps() == 1))) {
    // Launch a new thread to perform Global Bundle Adjustment
    mbRunningGBA  = true;
    mbFinishedGBA = false;
    mbStopGBA     = false;
    mpThreadGBA   = new std::thread(
      &LoopClosing::RunGlobalBundleAdjustment,
      this,
      pMergeMap,
      mpCurrentKF->mnId
    );
    _logger->info("MergeLocal: global bundle ajustment thread running back");
  }

  mpMergeMatchedKF->AddMergeEdge(mpCurrentKF);
  mpCurrentKF->AddMergeEdge(mpMergeMatchedKF);

  pCurrentMap->IncreaseChangeIndex();
  pMergeMap->IncreaseChangeIndex();

  mpAtlas->RemoveBadMaps();
}

void LoopClosing::MergeLocal2() {
  _logger->info("MergeLocal: starting to merge in local with IMU data");

  int numTemporalKFs
    = 11; // TODO (set by parameter): Temporal KFs in the local window if the map is inertial.

  // Relationship to rebuild the essential graph, it is used two times, first in the local window
  // and later in the rest of the map
  KeyFrame* pNewChild;
  KeyFrame* pNewParent;

  std::vector<KeyFrame*> vpLocalCurrentWindowKFs;
  std::vector<KeyFrame*> vpMergeConnectedKFs;

  KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
  // NonCorrectedSim3[mpCurrentKF]=mg2oLoopScw;

  // Flag that is true only when we stopped a running BA, in this case we need relaunch at the end
  // of the merge
  bool bRelaunchBA = false;

  //  If a Global Bundle Adjustment is running, abort it
  if (isRunningGBA()) {
    _logger->info("MergeLocal: stopping global bundle adjustment...");
    std::unique_lock<std::mutex> lock(mMutexGBA);
    mbStopGBA = true;

    mnFullBAIdx++;

    if (mpThreadGBA) {
      mpThreadGBA->detach();
      delete mpThreadGBA;
    }
    bRelaunchBA = true;
  }

  _logger->info("MergeLocal: requesting to stop local mapping...");
  mpLocalMapper->RequestStop();
  // Wait until Local Mapping has effectively stopped
  while (!mpLocalMapper->isStopped()) {
    usleep(1000);
  }

  Map* pCurrentMap = mpCurrentKF->GetMap();
  Map* pMergeMap   = mpMergeMatchedKF->GetMap();
  _logger->info(
    "MergeLocal: merging local active map {} and non-active map {}",
    pCurrentMap->GetId(),
    pMergeMap->GetId()
  );

  {
    float        s_on = mSold_new.scale();
    Sophus::SE3f T_on(mSold_new.rotation().cast<float>(), mSold_new.translation().cast<float>());

    std::unique_lock<std::mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);

    mpLocalMapper->EmptyQueue();

    std::chrono::steady_clock::time_point t2        = std::chrono::steady_clock::now();
    bool                                  bScaleVel = false;
    if (s_on != 1) {
      bScaleVel = true;
    }
    mpAtlas->GetCurrentMap()->ApplyScaledRotation(T_on, s_on, bScaleVel);
    mpTracker->UpdateFrameIMU(s_on, mpCurrentKF->GetImuBias(), mpTracker->GetLastKeyFrame());
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
  }

  const int numKFnew = pCurrentMap->KeyFramesInMap();

  if (IsInertialBased(mpTracker->mSensor) && !pCurrentMap->GetIniertialBA2()) {
    // Map is not completly initialized
    Eigen::Vector3d bg, ba;
    bg << 0., 0., 0.;
    ba << 0., 0., 0.;
    Optimizer::InertialOptimization(pCurrentMap, bg, ba);
    IMU::Bias                    b(ba[0], ba[1], ba[2], bg[0], bg[1], bg[2]);
    std::unique_lock<std::mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
    mpTracker->UpdateFrameIMU(1.0f, b, mpTracker->GetLastKeyFrame());

    // Set map initialized
    pCurrentMap->SetIniertialBA2();
    pCurrentMap->SetIniertialBA1();
    pCurrentMap->SetImuInitialized();
  }

  // Load KFs and MPs from merge map
  _logger->info("MergeLocal: updating current map...");
  {
    // Get Merge Map Mutex (This section stops tracking!!)
    std::unique_lock<std::mutex> currentLock(pCurrentMap->mMutexMapUpdate
    ); // We update the current map with the Merge information
    std::unique_lock<std::mutex> mergeLock(pMergeMap->mMutexMapUpdate
    ); // We remove the Kfs and MPs in the merged area from the old map

    // TODO(VuHoi): remove auxilary variables.
    std::vector<KeyFrame*> vpMergeMapKFs = pMergeMap->GetAllKeyFrames();
    std::vector<MapPoint*> vpMergeMapMPs = pMergeMap->GetAllMapPoints();

    for (KeyFrame* const merged : vpMergeMapKFs) {
      if (!merged || merged->isBad() || merged->GetMap() != pMergeMap) {
        continue;
      }

      // Make sure connections are updated
      merged->UpdateMap(pCurrentMap);
      pCurrentMap->AddKeyFrame(merged);
      pMergeMap->EraseKeyFrame(merged);
    }

    for (MapPoint* const merged : vpMergeMapMPs) {
      if (!merged || merged->isBad() || merged->GetMap() != pMergeMap) {
        continue;
      }

      merged->UpdateMap(pCurrentMap);
      pCurrentMap->AddMapPoint(merged);
      pMergeMap->EraseMapPoint(merged);
    }

    // Save non corrected poses (already merged maps)
    for (KeyFrame* const current : pCurrentMap->GetAllKeyFrames()) {
      Sophus::SE3d Tiw = (current->GetPose()).cast<double>();
      g2o::Sim3    g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
      NonCorrectedSim3[current] = g2oSiw;
    }
  }

  _logger->info("MergeLocal: updating essential graph...");
  //  mpCurrentKF->UpdateConnections(); // to put at false mbFirstConnection
  pMergeMap->GetOriginKF()->SetFirstConnection(false);
  pNewChild  = mpMergeMatchedKF->GetParent(); // Old parent, it will be the new child of this KF
  pNewParent = mpMergeMatchedKF; // Old child, now it will be the parent of its own parent(we need
                                 // eliminate this KF from children list in its old parent)
  mpMergeMatchedKF->ChangeParent(mpCurrentKF);
  while (pNewChild) {
    pNewChild->EraseChild(pNewParent
    ); // We remove the relation between the old parent and the new for avoid loop
    KeyFrame* pOldParent = pNewChild->GetParent();
    pNewChild->ChangeParent(pNewParent);
    pNewParent = pNewChild;
    pNewChild  = pOldParent;
  }

  _logger->info("MergeLocal: updating relationship between key frames...");
  std::vector<MapPoint*> vpCheckFuseMapPoint; // MapPoint vector from current map to allow to fuse
                                              // duplicated points with the old map (merge)
  std::vector<KeyFrame*> vpCurrentConnectedKFs;

  mvpMergeConnectedKFs.push_back(mpMergeMatchedKF);
  std::vector<KeyFrame*> aux = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
  mvpMergeConnectedKFs.insert(mvpMergeConnectedKFs.end(), aux.begin(), aux.end());
  if (mvpMergeConnectedKFs.size() > 6) {
    mvpMergeConnectedKFs.erase(mvpMergeConnectedKFs.begin() + 6, mvpMergeConnectedKFs.end());
  }
  /*mvpMergeConnectedKFs = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
  mvpMergeConnectedKFs.push_back(mpMergeMatchedKF);*/

  mpCurrentKF->UpdateConnections();
  vpCurrentConnectedKFs.push_back(mpCurrentKF);
  /*vpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
  vpCurrentConnectedKFs.push_back(mpCurrentKF);*/
  aux = mpCurrentKF->GetVectorCovisibleKeyFrames();
  vpCurrentConnectedKFs.insert(vpCurrentConnectedKFs.end(), aux.begin(), aux.end());
  if (vpCurrentConnectedKFs.size() > 6) {
    vpCurrentConnectedKFs.erase(vpCurrentConnectedKFs.begin() + 6, vpCurrentConnectedKFs.end());
  }

  std::set<MapPoint*> spMapPointMerge;
  for (KeyFrame* const merged : mvpMergeConnectedKFs) {
    std::set<MapPoint*> vpMPs = merged->GetMapPoints();
    spMapPointMerge.insert(vpMPs.begin(), vpMPs.end());
    if (spMapPointMerge.size() > 1000) {
      break;
    }
  }

  vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
  std::copy(
    spMapPointMerge.begin(),
    spMapPointMerge.end(),
    std::back_inserter(vpCheckFuseMapPoint)
  );

  SearchAndFuse(vpCurrentConnectedKFs, vpCheckFuseMapPoint);

  std::ranges::for_each(
    vpCurrentConnectedKFs | std::views::filter([](KeyFrame* const kf) {
      return kf && !kf->isBad();
    }),
    [](KeyFrame* const kf) {
      kf->UpdateConnections();
    }
  );
  std::ranges::for_each(
    mvpMergeConnectedKFs | std::views::filter([](KeyFrame* const kf) {
      return kf && !kf->isBad();
    }),
    [](KeyFrame* const kf) {
      kf->UpdateConnections();
    }
  );

  // TODO Check: If new map is too small, we suppose that not informaiton can be propagated from new
  // to old map
  if (numKFnew < 10) {
    _logger->info("MergeLocal: merge aborted due to new small map, releasing local mapping...");
    mpLocalMapper->Release();
    return;
  }

  _logger->info("MergeLocal: performing bundle adjustment...");
  bool      bStopFlag = false;
  KeyFrame* pCurrKF   = mpTracker->GetLastKeyFrame();
  Optimizer::MergeInertialBA(pCurrKF, mpMergeMatchedKF, &bStopFlag, pCurrentMap, CorrectedSim3);

  _logger->info("MergeLocal: releasing local mapping...");
  mpLocalMapper->Release();

  return;
}

void LoopClosing::CheckObservations(
  std::set<KeyFrame*>& spKFsMap1, std::set<KeyFrame*>& spKFsMap2
) {
  for (KeyFrame* const kf1 : spKFsMap1) {
    std::map<KeyFrame*, int> mMatchedMP;
    for (MapPoint* const mp1 : kf1->GetMapPoints()) {
      if (!mp1 || mp1->isBad()) {
        continue;
      }

      std::map<KeyFrame*, std::tuple<int, int>> mMPijObs = mp1->GetObservations();
      for (KeyFrame* const kf2 : spKFsMap2) {
        if (mMPijObs.find(kf2) != mMPijObs.end()) {
          if (mMatchedMP.find(kf2) != mMatchedMP.end()) {
            mMatchedMP[kf2] = mMatchedMP[kf2] + 1;
          } else {
            mMatchedMP[kf2] = 1;
          }
        }
      }
    }

    if (mMatchedMP.size() == 0) {
      _logger->info(
        "CheckObservations: key frame {} has no matched map point with the other map",
        kf1->mnId
      );
    } else {
      _logger->info(
        "CheckObservations: key frame {} has {} matched map points from the other map",
        kf1->mnId,
        mMatchedMP.size()
      );
      for (const auto& [kf, num_matches] : mMatchedMP) {
        _logger->debug("Key frame {}: {} matches", kf->mnId, num_matches);
      }
    }
  }
}

void LoopClosing::SearchAndFuse(
  const KeyFrameAndPose& CorrectedPosesMap, std::vector<MapPoint*>& vpMapPoints
) {
  ORBmatcher matcher(0.8);

  int total_replaces = 0;

  for (const auto& [kf, g2oScw] : CorrectedPosesMap) {
    int           num_replaces = 0;
    Map*          pMap         = kf->GetMap();
    Sophus::Sim3f Scw          = Converter::toSophus(g2oScw);

    std::vector<MapPoint*> vpReplacePoints(vpMapPoints.size(), static_cast<MapPoint*>(NULL));
    int                    numFused = matcher.Fuse(kf, Scw, vpMapPoints, 4, vpReplacePoints);

    // Get Map Mutex
    std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);
    const int                    nLP = vpMapPoints.size();
    for (int i = 0; i < nLP; i++) {
      MapPoint* pRep = vpReplacePoints[i];
      if (pRep) {
        num_replaces += 1;
        pRep->Replace(vpMapPoints[i]);
      }
    }

    total_replaces += num_replaces;
  }
}

void LoopClosing::SearchAndFuse(
  const std::vector<KeyFrame*>& vConectedKFs, std::vector<MapPoint*>& vpMapPoints
) {
  ORBmatcher matcher(0.8);

  int total_replaces = 0;

  for (KeyFrame* const kf : vConectedKFs) {
    int           num_replaces = 0;
    Map*          pMap         = kf->GetMap();
    Sophus::SE3f  Tcw          = kf->GetPose();
    Sophus::Sim3f Scw(Tcw.unit_quaternion(), Tcw.translation());
    Scw.setScale(1.f);
    // TODO(VuHoi): assert Scw.rotationMatrix() = Tcw.rotationMatrix()
    //              Scw.translation()    = Tcw.translation()
    //              Scw.scale()          = 1
    std::vector<MapPoint*> vpReplacePoints(vpMapPoints.size(), static_cast<MapPoint*>(NULL));
    matcher.Fuse(kf, Scw, vpMapPoints, 4, vpReplacePoints);

    // Get Map Mutex
    std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);
    const int                    nLP = vpMapPoints.size();
    for (int i = 0; i < nLP; i++) {
      MapPoint* pRep = vpReplacePoints[i];
      if (pRep) {
        num_replaces += 1;
        pRep->Replace(vpMapPoints[i]);
      }
    }
  }
}

void LoopClosing::RequestReset() {
  {
    std::unique_lock<std::mutex> lock(mMutexReset);
    mbResetRequested = true;
  }

  while (1) {
    {
      std::unique_lock<std::mutex> lock2(mMutexReset);
      if (!mbResetRequested) {
        break;
      }
    }
    usleep(5000);
  }
  _logger->info("Reset completed successfully");
}

void LoopClosing::RequestResetActiveMap(Map* pMap) {
  {
    std::unique_lock<std::mutex> lock(mMutexReset);
    mbResetActiveMapRequested = true;
    mpMapToReset              = pMap;
  }

  while (1) {
    {
      std::unique_lock<std::mutex> lock2(mMutexReset);
      if (!mbResetActiveMapRequested) {
        break;
      }
    }
    usleep(3000);
  }
  _logger->info("Active map reset completed successfully");
}

void LoopClosing::ResetIfRequested() {
  std::unique_lock<std::mutex> lock(mMutexReset);
  if (mbResetRequested) {
    _logger->info("Requesting to reset loop closing...");
    mlpLoopKeyFrameQueue.clear();
    mLastLoopKFid             = 0; // TODO old variable, it is not use in the new algorithm
    mbResetRequested          = false;
    mbResetActiveMapRequested = false;
  } else if (mbResetActiveMapRequested) {
    std::erase_if(mlpLoopKeyFrameQueue, [this](KeyFrame* const kf) {
      return kf->GetMap() == mpMapToReset;
    });

    mLastLoopKFid
      = mpAtlas->GetLastInitKFid(); // TODO old variable, it is not use in the new algorithm
    mbResetActiveMapRequested = false;
  }
}

void LoopClosing::RunGlobalBundleAdjustment(Map* pActiveMap, unsigned long nLoopKF) {
  _logger->info("Starting global bundle adjustment...");

#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_StartFGBA  = std::chrono::steady_clock::now();
  nFGBA_exec                                           += 1;
  vnGBAKFs.push_back(pActiveMap->GetAllKeyFrames().size());
  vnGBAMPs.push_back(pActiveMap->GetAllMapPoints().size());
#endif

  const bool bImuInit = pActiveMap->isImuInitialized();

  if (!bImuInit) {
    Optimizer::GlobalBundleAdjustemnt(pActiveMap, 10, &mbStopGBA, nLoopKF, false);
  } else {
    Optimizer::FullInertialBA(pActiveMap, 7, false, nLoopKF, &mbStopGBA);
  }

#ifdef REGISTER_TIMES
  std::chrono::steady_clock::time_point time_EndGBA = std::chrono::steady_clock::now();
  double timeGBA = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                     time_EndGBA - time_StartFGBA
  )
                     .count();
  vdGBA_ms.push_back(timeGBA);

  if (mbStopGBA) {
    nFGBA_abort += 1;
  }
#endif

  int idx = mnFullBAIdx;
  // Optimizer::GlobalBundleAdjustemnt(mpMap,10,&mbStopGBA,nLoopKF,false);

  // Update all MapPoints and KeyFrames
  // Local Mapping was active during BA, that means that there might be new keyframes
  // not included in the Global BA and they are not consistent with the updated map.
  // We need to propagate the correction through the spanning tree
  {
    std::unique_lock<std::mutex> lock(mMutexGBA);
    if (idx != mnFullBAIdx) {
      return;
    }

    if (!bImuInit && pActiveMap->isImuInitialized()) {
      return;
    }

    if (!mbStopGBA) {
      _logger->info("Global Bundle Adjustment finished, updating map...");

      mpLocalMapper->RequestStop();
      // Wait until Local Mapping has effectively stopped

      while (!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished()) {
        usleep(1000);
      }

      // Get Map Mutex
      std::unique_lock<std::mutex> lock(pActiveMap->mMutexMapUpdate);

      // pActiveMap->PrintEssentialGraph();
      //  Correct keyframes starting at map first keyframe
      std::list<KeyFrame*> lpKFtoCheck(
        pActiveMap->mvpKeyFrameOrigins.begin(),
        pActiveMap->mvpKeyFrameOrigins.end()
      );

      while (!lpKFtoCheck.empty()) {
        KeyFrame*    pKF = lpKFtoCheck.front();
        Sophus::SE3f Twc = pKF->GetPoseInverse();
        // Correct key frames
        for (KeyFrame* const child : pKF->GetChilds()) {
          if (!child || child->isBad()) {
            continue;
          }

          if (child->mnBAGlobalForKF != nLoopKF) {
            Sophus::SE3f Tchildc = child->GetPose() * Twc;
            child->mTcwGBA       = Tchildc * pKF->mTcwGBA; //*Tcorc*pKF->mTcwGBA;

            Sophus::SO3f Rcor = child->mTcwGBA.so3().inverse() * child->GetPose().so3();
            if (child->isVelocitySet()) {
              child->mVwbGBA = Rcor * child->GetVelocity();
            } else {
              _logger->info("Empty child velocity");
            }

            child->mBiasGBA = child->GetImuBias();

            child->mnBAGlobalForKF = nLoopKF;
          }
          lpKFtoCheck.push_back(child);
        }

        // Update pose
        pKF->mTcwBefGBA = pKF->GetPose();
        pKF->SetPose(pKF->mTcwGBA);
        /*cv::Mat Tco_cn = pKF->mTcwBefGBA * pKF->mTcwGBA.inv();
        cv::Vec3d trasl = Tco_cn.rowRange(0,3).col(3);
        double dist = cv::norm(trasl);
        std::cout << "GBA: KF " << pKF->mnId << " had been moved " << dist << " meters" <<
        std::endl; double desvX = 0; double desvY = 0; double desvZ = 0; if(pKF->mbHasHessian)
        {
            cv::Mat hessianInv = pKF->mHessianPose.inv();

            double covX = hessianInv.at<double>(3,3);
            desvX = std::sqrt(covX);
            double covY = hessianInv.at<double>(4,4);
            desvY = std::sqrt(covY);
            double covZ = hessianInv.at<double>(5,5);
            desvZ = std::sqrt(covZ);
            pKF->mbHasHessian = false;
        }
        if(dist > 1)
        {
            std::cout << "--To much distance correction: It has " <<
        pKF->GetConnectedKeyFrames().size()
        << " connected KFs" << std::endl; std::cout << "--It has " <<
        pKF->GetCovisiblesByWeight(80).size() << " connected KF with 80 common matches or more" <<
        std::endl; std::cout << "--It has " << pKF->GetCovisiblesByWeight(50).size() << " connected
        KF with 50 common matches or more" << std::endl; std::cout << "--It has " <<
        pKF->GetCovisiblesByWeight(20).size() << " connected KF with 20 common matches or more" <<
        std::endl;

            std::cout << "--STD in meters(x, y, z): " << desvX << ", " << desvY << ", " << desvZ <<
        std::endl;


            std::string strNameFile = pKF->mNameFile;
            cv::Mat imLeft = cv::imread(strNameFile, CV_LOAD_IMAGE_UNCHANGED);

            cv::cvtColor(imLeft, imLeft, CV_GRAY2BGR);

            std:vector<MapPoint*> vpMapPointsKF = pKF->GetMapPointMatches();
            int num_MPs = 0;
            for(int i=0; i<vpMapPointsKF.size(); ++i)
            {
                if(!vpMapPointsKF[i] || vpMapPointsKF[i]->isBad())
                {
                    continue;
                }
                num_MPs += 1;
                std::string strNumOBs = std::to_string(vpMapPointsKF[i]->Observations());
                cv::circle(imLeft, pKF->mvKeys[i].pt, 2, cv::Scalar(0, 255, 0));
                cv::putText(imLeft, strNumOBs, pKF->mvKeys[i].pt, CV_FONT_HERSHEY_DUPLEX, 1,
        cv::Scalar(255, 0, 0));
            }
            _logger->info("--It has " << num_MPs << " MPs matched in the map");

            std::string namefile = "./test_GBA/GBA_" + std::to_string(nLoopKF) + "_KF" +
        std::to_string(pKF->mnId)
        +"_D" + std::to_string(dist) +".png"; cv::imwrite(namefile, imLeft);
        }*/

        if (pKF->bImu) {
          // Update inertial values
          pKF->mVwbBefGBA = pKF->GetVelocity();
          // if (pKF->mVwbGBA.empty())
          //   _logger->info("pKF->mVwbGBA is empty");

          // assert(!pKF->mVwbGBA.empty());
          pKF->SetVelocity(pKF->mVwbGBA);
          pKF->SetNewBias(pKF->mBiasGBA);
        }

        lpKFtoCheck.pop_front();
      }

      // Correct MapPoints
      for (MapPoint* const pMP : pActiveMap->GetAllMapPoints()) {
        if (pMP->isBad()) {
          continue;
        }

        if (pMP->mnBAGlobalForKF == nLoopKF) {
          // If optimized by Global BA, just update
          pMP->SetWorldPos(pMP->mPosGBA);
        } else {
          // Update according to the correction of its reference keyframe
          KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

          if (pRefKF->mnBAGlobalForKF != nLoopKF) {
            continue;
          }

          /*if(pRefKF->mTcwBefGBA.empty())
              continue;*/

          // Map to non-corrected camera
          // cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
          // cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
          Eigen::Vector3f Xc = pRefKF->mTcwBefGBA * pMP->GetWorldPos();

          // Backproject using corrected camera
          pMP->SetWorldPos(pRefKF->GetPoseInverse() * Xc);
        }
      }

      pActiveMap->InformNewBigChange();
      pActiveMap->IncreaseChangeIndex();

      // TODO Check this update
      // mpTracker->UpdateFrameIMU(1.0f, mpTracker->GetLastKeyFrame()->GetImuBias(),
      // mpTracker->GetLastKeyFrame());

      mpLocalMapper->Release();

#ifdef REGISTER_TIMES
      std::chrono::steady_clock::time_point time_EndUpdateMap = std::chrono::steady_clock::now();
      double timeUpdateMap = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                               time_EndUpdateMap - time_EndGBA
      )
                               .count();
      vdUpdateMap_ms.push_back(timeUpdateMap);
      double timeFGBA = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                          time_EndUpdateMap - time_StartFGBA
      )
                          .count();
      vdFGBATotal_ms.push_back(timeFGBA);
#endif
      _logger->info("Map updated");
    }

    mbFinishedGBA = true;
    mbRunningGBA  = false;
  }
}

void LoopClosing::RequestFinish() {
  std::unique_lock<std::mutex> lock(mMutexFinish);
  mbFinishRequested = true;
}

bool LoopClosing::CheckFinish() {
  std::unique_lock<std::mutex> lock(mMutexFinish);
  return mbFinishRequested;
}

void LoopClosing::SetFinish() {
  std::unique_lock<std::mutex> lock(mMutexFinish);
  mbFinished = true;
}

bool LoopClosing::isFinished() {
  std::unique_lock<std::mutex> lock(mMutexFinish);
  return mbFinished;
}

} // namespace ORB_SLAM3

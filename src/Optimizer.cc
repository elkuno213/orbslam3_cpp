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

#include "Optimizer.h"
#include <cmath>
#include <list>
#include <mutex>
#include <ranges>
#include <tuple>
#include <utility>
#include <Thirdparty/g2o/g2o/core/block_solver.h>
#include <Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h>
#include <Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h>
#include <Thirdparty/g2o/g2o/core/robust_kernel_impl.h>
#include <Thirdparty/g2o/g2o/core/sparse_optimizer.h>
#include <Thirdparty/g2o/g2o/solvers/linear_solver_dense.h>
#include <Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h>
#include <Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h>
#include "Frame.h"
#include "G2oTypes.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include "OptimizableTypes.h"

namespace ORB_SLAM3 {
bool sortByVal(const std::pair<MapPoint*, int>& a, const std::pair<MapPoint*, int>& b) {
  return (a.second < b.second);
}

void Optimizer::GlobalBundleAdjustemnt(
  Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust
) {
  std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
  std::vector<MapPoint*> vpMP  = pMap->GetAllMapPoints();
  BundleAdjustment(vpKFs, vpMP, nIterations, pbStopFlag, nLoopKF, bRobust);
}

void Optimizer::BundleAdjustment(
  const std::vector<KeyFrame*>& vpKFs,
  const std::vector<MapPoint*>& vpMP,
  int                           nIterations,
  bool*                         pbStopFlag,
  const unsigned long           nLoopKF,
  const bool                    bRobust
) {
  std::vector<bool> vbNotIncludedMP;
  vbNotIncludedMP.resize(vpMP.size());

  Map* pMap = vpKFs[0]->GetMap();

  g2o::SparseOptimizer                    optimizer;
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver;

  linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  if (pbStopFlag) {
    optimizer.setForceStopFlag(pbStopFlag);
  }

  long unsigned int maxKFid = 0;

  const int nExpectedSize = (vpKFs.size()) * vpMP.size();

  std::vector<ORB_SLAM3::EdgeSE3ProjectXYZ*> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);

  std::vector<ORB_SLAM3::EdgeSE3ProjectXYZToBody*> vpEdgesBody;
  vpEdgesBody.reserve(nExpectedSize);

  std::vector<KeyFrame*> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);

  std::vector<KeyFrame*> vpEdgeKFBody;
  vpEdgeKFBody.reserve(nExpectedSize);

  std::vector<MapPoint*> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  std::vector<MapPoint*> vpMapPointEdgeBody;
  vpMapPointEdgeBody.reserve(nExpectedSize);

  std::vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);

  std::vector<KeyFrame*> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);

  std::vector<MapPoint*> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  // Set KeyFrame vertices

  for (KeyFrame* const kf : vpKFs) {
    if (kf->isBad()) {
      continue;
    }
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float>    Tcw  = kf->GetPose();
    vSE3->setEstimate(
      g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>())
    );
    vSE3->setId(kf->mnId);
    vSE3->setFixed(kf->mnId == pMap->GetInitKFid());
    optimizer.addVertex(vSE3);
    if (kf->mnId > maxKFid) {
      maxKFid = kf->mnId;
    }
  }

  const float thHuber2D = std::sqrt(5.99);
  const float thHuber3D = std::sqrt(7.815);

  // Set MapPoint vertices
  for (std::size_t i = 0; i < vpMP.size(); i++) {
    MapPoint* pMP = vpMP[i];
    if (pMP->isBad()) {
      continue;
    }
    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
    const int id = pMP->mnId + maxKFid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    auto observations = pMP->GetObservations();

    int nEdges = 0;
    // SET EDGES
    for (const auto& [pKF, indices] : observations) {
      if (pKF->isBad() || pKF->mnId > maxKFid) {
        continue;
      }
      if (optimizer.vertex(id) == NULL || optimizer.vertex(pKF->mnId) == NULL) {
        continue;
      }
      nEdges++;

      auto [left_id, right_id] = indices;

      if (left_id != -1 && pKF->mvuRight[left_id] < 0) {
        const cv::KeyPoint& kpUn = pKF->mvKeysUn[left_id];

        Eigen::Matrix<double, 2, 1> obs;
        obs << kpUn.pt.x, kpUn.pt.y;

        ORB_SLAM3::EdgeSE3ProjectXYZ* e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
        e->setMeasurement(obs);
        const float& invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

        if (bRobust) {
          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuber2D);
        }

        e->pCamera = pKF->mpCamera;

        optimizer.addEdge(e);

        vpEdgesMono.push_back(e);
        vpEdgeKFMono.push_back(pKF);
        vpMapPointEdgeMono.push_back(pMP);
      } else if (left_id != -1 && pKF->mvuRight[left_id] >= 0) { // Stereo observation
        const cv::KeyPoint& kpUn = pKF->mvKeysUn[left_id];

        Eigen::Matrix<double, 3, 1> obs;
        const float                 kp_ur = pKF->mvuRight[left_id];
        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

        g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
        e->setMeasurement(obs);
        const float&    invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
        Eigen::Matrix3d Info      = Eigen::Matrix3d::Identity() * invSigma2;
        e->setInformation(Info);

        if (bRobust) {
          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuber3D);
        }

        e->fx = pKF->fx;
        e->fy = pKF->fy;
        e->cx = pKF->cx;
        e->cy = pKF->cy;
        e->bf = pKF->mbf;

        optimizer.addEdge(e);

        vpEdgesStereo.push_back(e);
        vpEdgeKFStereo.push_back(pKF);
        vpMapPointEdgeStereo.push_back(pMP);
      }

      if (pKF->mpCamera2) {
        if (right_id != -1 && right_id < pKF->mvKeysRight.size()) {
          right_id -= pKF->NLeft;

          Eigen::Matrix<double, 2, 1> obs;
          cv::KeyPoint                kp = pKF->mvKeysRight[right_id];
          obs << kp.pt.x, kp.pt.y;

          ORB_SLAM3::EdgeSE3ProjectXYZToBody* e = new ORB_SLAM3::EdgeSE3ProjectXYZToBody();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
          e->setVertex(
            1,
            dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId))
          );
          e->setMeasurement(obs);
          const float& invSigma2 = pKF->mvInvLevelSigma2[kp.octave];
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuber2D);

          Sophus::SE3f Trl = pKF->GetRelativePoseTrl();
          e->mTrl
            = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(), Trl.translation().cast<double>());

          e->pCamera = pKF->mpCamera2;

          optimizer.addEdge(e);
          vpEdgesBody.push_back(e);
          vpEdgeKFBody.push_back(pKF);
          vpMapPointEdgeBody.push_back(pMP);
        }
      }
    }

    if (nEdges == 0) {
      optimizer.removeVertex(vPoint);
      vbNotIncludedMP[i] = true;
    } else {
      vbNotIncludedMP[i] = false;
    }
  }

  // Optimize!
  optimizer.setVerbose(false);
  optimizer.initializeOptimization();
  optimizer.optimize(nIterations);

  // Recover optimized data
  // Keyframes
  for (KeyFrame* const kf : vpKFs) {
    if (kf->isBad()) {
      continue;
    }
    g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(kf->mnId));

    g2o::SE3Quat SE3quat = vSE3->estimate();
    if (nLoopKF == pMap->GetOriginKF()->mnId) {
      kf->SetPose(
        Sophus::SE3f(SE3quat.rotation().cast<float>(), SE3quat.translation().cast<float>())
      );
    } else {
      kf->mTcwGBA         = Sophus::SE3d(SE3quat.rotation(), SE3quat.translation()).cast<float>();
      kf->mnBAGlobalForKF = nLoopKF;

      Sophus::SE3f    mTwc        = kf->GetPoseInverse();
      Sophus::SE3f    mTcGBA_c    = kf->mTcwGBA * mTwc;
      Eigen::Vector3f vector_dist = mTcGBA_c.translation();
      double          dist        = vector_dist.norm();
      if (dist > 1) {
        int                    numMonoBadPoints = 0, numMonoOptPoints = 0;
        int                    numStereoBadPoints = 0, numStereoOptPoints = 0;
        std::vector<MapPoint*> vpMonoMPsOpt, vpStereoMPsOpt;

        for (std::size_t i = 0; i < vpEdgesMono.size(); i++) {
          ORB_SLAM3::EdgeSE3ProjectXYZ* e       = vpEdgesMono[i];
          MapPoint*                     pMP     = vpMapPointEdgeMono[i];
          KeyFrame*                     pKFedge = vpEdgeKFMono[i];

          if (kf != pKFedge) {
            continue;
          }

          if (pMP->isBad()) {
            continue;
          }

          if (e->chi2() > 5.991 || !e->isDepthPositive()) {
            numMonoBadPoints++;

          } else {
            numMonoOptPoints++;
            vpMonoMPsOpt.push_back(pMP);
          }
        }

        for (std::size_t i = 0; i < vpEdgesStereo.size(); i++) {
          g2o::EdgeStereoSE3ProjectXYZ* e       = vpEdgesStereo[i];
          MapPoint*                     pMP     = vpMapPointEdgeStereo[i];
          KeyFrame*                     pKFedge = vpEdgeKFMono[i];

          if (kf != pKFedge) {
            continue;
          }

          if (pMP->isBad()) {
            continue;
          }

          if (e->chi2() > 7.815 || !e->isDepthPositive()) {
            numStereoBadPoints++;
          } else {
            numStereoOptPoints++;
            vpStereoMPsOpt.push_back(pMP);
          }
        }
      }
    }
  }

  // Points
  for (std::size_t i = 0; i < vpMP.size(); i++) {
    if (vbNotIncludedMP[i]) {
      continue;
    }

    MapPoint* pMP = vpMP[i];

    if (pMP->isBad()) {
      continue;
    }
    g2o::VertexSBAPointXYZ* vPoint
      = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId + maxKFid + 1));

    if (nLoopKF == pMap->GetOriginKF()->mnId) {
      pMP->SetWorldPos(vPoint->estimate().cast<float>());
      pMP->UpdateNormalAndDepth();
    } else {
      pMP->mPosGBA         = vPoint->estimate().cast<float>();
      pMP->mnBAGlobalForKF = nLoopKF;
    }
  }
}

void Optimizer::FullInertialBA(
  Map*                    pMap,
  int                     its,
  const bool              bFixLocal,
  const long unsigned int nLoopId,
  bool*                   pbStopFlag,
  bool                    bInit,
  float                   priorG,
  float                   priorA,
  Eigen::VectorXd*        vSingVal,
  bool*                   bHess
) {
  long unsigned int            maxKFid = pMap->GetMaxKFid();
  const std::vector<KeyFrame*> vpKFs   = pMap->GetAllKeyFrames();
  const std::vector<MapPoint*> vpMPs   = pMap->GetAllMapPoints();

  // Setup optimizer
  g2o::SparseOptimizer                 optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  solver->setUserLambdaInit(1e-5);
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  if (pbStopFlag) {
    optimizer.setForceStopFlag(pbStopFlag);
  }

  int nNonFixed = 0;

  // Set KeyFrame vertices
  KeyFrame* pIncKF;
  for (KeyFrame* const kf : vpKFs) {
    if (kf->mnId > maxKFid) {
      continue;
    }
    VertexPose* VP = new VertexPose(kf);
    VP->setId(kf->mnId);
    pIncKF      = kf;
    bool bFixed = false;
    if (bFixLocal) {
      bFixed = (kf->mnBALocalForKF >= (maxKFid - 1)) || (kf->mnBAFixedForKF >= (maxKFid - 1));
      if (!bFixed) {
        nNonFixed++;
      }
      VP->setFixed(bFixed);
    }
    optimizer.addVertex(VP);

    if (kf->bImu) {
      VertexVelocity* VV = new VertexVelocity(kf);
      VV->setId(maxKFid + 3 * (kf->mnId) + 1);
      VV->setFixed(bFixed);
      optimizer.addVertex(VV);
      if (!bInit) {
        VertexGyroBias* VG = new VertexGyroBias(kf);
        VG->setId(maxKFid + 3 * (kf->mnId) + 2);
        VG->setFixed(bFixed);
        optimizer.addVertex(VG);
        VertexAccBias* VA = new VertexAccBias(kf);
        VA->setId(maxKFid + 3 * (kf->mnId) + 3);
        VA->setFixed(bFixed);
        optimizer.addVertex(VA);
      }
    }
  }

  if (bInit) {
    VertexGyroBias* VG = new VertexGyroBias(pIncKF);
    VG->setId(4 * maxKFid + 2);
    VG->setFixed(false);
    optimizer.addVertex(VG);
    VertexAccBias* VA = new VertexAccBias(pIncKF);
    VA->setId(4 * maxKFid + 3);
    VA->setFixed(false);
    optimizer.addVertex(VA);
  }

  if (bFixLocal) {
    if (nNonFixed < 3) {
      return;
    }
  }

  // IMU links
  for (KeyFrame* const kf : vpKFs) {
    if (!kf->mPrevKF) {
      // No inertial link to previous frame.
      continue;
    }

    if (kf->mPrevKF && kf->mnId <= maxKFid) {
      if (kf->isBad() || kf->mPrevKF->mnId > maxKFid) {
        continue;
      }
      if (kf->bImu && kf->mPrevKF->bImu) {
        kf->mpImuPreintegrated->SetNewBias(kf->mPrevKF->GetImuBias());
        g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(kf->mPrevKF->mnId);
        g2o::HyperGraph::Vertex* VV1 = optimizer.vertex(maxKFid + 3 * (kf->mPrevKF->mnId) + 1);

        g2o::HyperGraph::Vertex* VG1;
        g2o::HyperGraph::Vertex* VA1;
        g2o::HyperGraph::Vertex* VG2;
        g2o::HyperGraph::Vertex* VA2;
        if (!bInit) {
          VG1 = optimizer.vertex(maxKFid + 3 * (kf->mPrevKF->mnId) + 2);
          VA1 = optimizer.vertex(maxKFid + 3 * (kf->mPrevKF->mnId) + 3);
          VG2 = optimizer.vertex(maxKFid + 3 * (kf->mnId) + 2);
          VA2 = optimizer.vertex(maxKFid + 3 * (kf->mnId) + 3);
        } else {
          VG1 = optimizer.vertex(4 * maxKFid + 2);
          VA1 = optimizer.vertex(4 * maxKFid + 3);
        }

        g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(kf->mnId);
        g2o::HyperGraph::Vertex* VV2 = optimizer.vertex(maxKFid + 3 * (kf->mnId) + 1);

        if (!bInit) {
          if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2) {
            continue;
          }
        } else {
          if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2) {
            continue;
          }
        }

        EdgeInertial* ei = new EdgeInertial(kf->mpImuPreintegrated);
        ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
        ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
        ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
        ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
        ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
        ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

        g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
        ei->setRobustKernel(rki);
        rki->setDelta(std::sqrt(16.92));

        optimizer.addEdge(ei);

        if (!bInit) {
          EdgeGyroRW* egr = new EdgeGyroRW();
          egr->setVertex(0, VG1);
          egr->setVertex(1, VG2);
          Eigen::Matrix3d InfoG
            = kf->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
          egr->setInformation(InfoG);
          egr->computeError();
          optimizer.addEdge(egr);

          EdgeAccRW* ear = new EdgeAccRW();
          ear->setVertex(0, VA1);
          ear->setVertex(1, VA2);
          Eigen::Matrix3d InfoA
            = kf->mpImuPreintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
          ear->setInformation(InfoA);
          ear->computeError();
          optimizer.addEdge(ear);
        }
      } else {
        // both key frames and previous one have no IMU data.
      }
    }
  }

  if (bInit) {
    g2o::HyperGraph::Vertex* VG = optimizer.vertex(4 * maxKFid + 2);
    g2o::HyperGraph::Vertex* VA = optimizer.vertex(4 * maxKFid + 3);

    // Add prior to comon biases
    Eigen::Vector3f bprior;
    bprior.setZero();

    EdgePriorAcc* epa = new EdgePriorAcc(bprior);
    epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
    double infoPriorA = priorA; //
    epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
    optimizer.addEdge(epa);

    EdgePriorGyro* epg = new EdgePriorGyro(bprior);
    epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
    double infoPriorG = priorG; //
    epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
    optimizer.addEdge(epg);
  }

  const float thHuberMono   = std::sqrt(5.991);
  const float thHuberStereo = std::sqrt(7.815);

  const unsigned long iniMPid = maxKFid * 5;

  std::vector<bool> vbNotIncludedMP(vpMPs.size(), false);

  for (std::size_t i = 0; i < vpMPs.size(); i++) {
    MapPoint*               pMP    = vpMPs[i];
    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
    unsigned long id = pMP->mnId + iniMPid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    bool bAllFixed = true;

    // Set edges
    for (const auto& [pKFi, indices] : pMP->GetObservations()) {
      if (pKFi->mnId > maxKFid) {
        continue;
      }

      if (!pKFi->isBad()) {
        auto [left_id, right_id] = indices;
        cv::KeyPoint kpUn;

        if (left_id != -1 && pKFi->mvuRight[left_id] < 0) { // Monocular observation
          kpUn = pKFi->mvKeysUn[left_id];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMono* e = new EdgeMono(0);

          g2o::OptimizableGraph::Vertex* VP
            = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));
          if (bAllFixed) {
            if (!VP->fixed()) {
              bAllFixed = false;
            }
          }

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
          e->setVertex(1, VP);
          e->setMeasurement(obs);
          const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);
        } else if (left_id != -1 && pKFi->mvuRight[left_id] >= 0) { // stereo observation
          kpUn                              = pKFi->mvKeysUn[left_id];
          const float                 kp_ur = pKFi->mvuRight[left_id];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          EdgeStereo* e = new EdgeStereo(0);

          g2o::OptimizableGraph::Vertex* VP
            = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));
          if (bAllFixed) {
            if (!VP->fixed()) {
              bAllFixed = false;
            }
          }

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
          e->setVertex(1, VP);
          e->setMeasurement(obs);
          const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);
        }

        if (pKFi->mpCamera2) { // Monocular right observation
          if (right_id != -1 && right_id < pKFi->mvKeysRight.size()) {
            right_id -= pKFi->NLeft;

            Eigen::Matrix<double, 2, 1> obs;
            kpUn = pKFi->mvKeysRight[right_id];
            obs << kpUn.pt.x, kpUn.pt.y;

            EdgeMono* e = new EdgeMono(1);

            g2o::OptimizableGraph::Vertex* VP
              = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));
            if (bAllFixed) {
              if (!VP->fixed()) {
                bAllFixed = false;
              }
            }

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setVertex(1, VP);
            e->setMeasurement(obs);
            const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberMono);

            optimizer.addEdge(e);
          }
        }
      }
    }

    if (bAllFixed) {
      optimizer.removeVertex(vPoint);
      vbNotIncludedMP[i] = true;
    }
  }

  if (pbStopFlag) {
    if (*pbStopFlag) {
      return;
    }
  }

  optimizer.initializeOptimization();
  optimizer.optimize(its);

  // Recover optimized data
  // Keyframes
  for (KeyFrame* const kf : vpKFs) {
    if (kf->mnId > maxKFid) {
      continue;
    }
    VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(kf->mnId));
    if (nLoopId == 0) {
      Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
      kf->SetPose(Tcw);
    } else {
      kf->mTcwGBA
        = Sophus::SE3f(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
      kf->mnBAGlobalForKF = nLoopId;
    }
    if (kf->bImu) {
      VertexVelocity* VV
        = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid + 3 * (kf->mnId) + 1));
      if (nLoopId == 0) {
        kf->SetVelocity(VV->estimate().cast<float>());
      } else {
        kf->mVwbGBA = VV->estimate().cast<float>();
      }

      VertexGyroBias* VG;
      VertexAccBias*  VA;
      if (!bInit) {
        VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid + 3 * (kf->mnId) + 2));
        VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid + 3 * (kf->mnId) + 3));
      } else {
        VG = static_cast<VertexGyroBias*>(optimizer.vertex(4 * maxKFid + 2));
        VA = static_cast<VertexAccBias*>(optimizer.vertex(4 * maxKFid + 3));
      }

      Vector6d vb;
      vb << VG->estimate(), VA->estimate();
      IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);
      if (nLoopId == 0) {
        kf->SetNewBias(b);
      } else {
        kf->mBiasGBA = b;
      }
    }
  }

  // Points
  for (std::size_t i = 0; i < vpMPs.size(); i++) {
    if (vbNotIncludedMP[i]) {
      continue;
    }

    MapPoint*               pMP = vpMPs[i];
    g2o::VertexSBAPointXYZ* vPoint
      = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId + iniMPid + 1));

    if (nLoopId == 0) {
      pMP->SetWorldPos(vPoint->estimate().cast<float>());
      pMP->UpdateNormalAndDepth();
    } else {
      pMP->mPosGBA         = vPoint->estimate().cast<float>();
      pMP->mnBAGlobalForKF = nLoopId;
    }
  }

  pMap->IncreaseChangeIndex();
}

int Optimizer::PoseOptimization(Frame* pFrame) {
  g2o::SparseOptimizer                    optimizer;
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver;

  linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  int nInitialCorrespondences = 0;

  // Set Frame vertex
  g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
  Sophus::SE3<float>    Tcw  = pFrame->GetPose();
  vSE3->setEstimate(
    g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>())
  );
  vSE3->setId(0);
  vSE3->setFixed(false);
  optimizer.addVertex(vSE3);

  // Set MapPoint vertices
  const int N = pFrame->N;

  std::vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose*>       vpEdgesMono;
  std::vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody*> vpEdgesMono_FHR;
  std::vector<std::size_t>                                 vnIndexEdgeMono, vnIndexEdgeRight;
  vpEdgesMono.reserve(N);
  vpEdgesMono_FHR.reserve(N);
  vnIndexEdgeMono.reserve(N);
  vnIndexEdgeRight.reserve(N);

  std::vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
  std::vector<std::size_t>                           vnIndexEdgeStereo;
  vpEdgesStereo.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  const float deltaMono   = std::sqrt(5.991);
  const float deltaStereo = std::sqrt(7.815);

  {
    std::unique_lock<std::mutex> lock(MapPoint::mGlobalMutex);

    for (int i = 0; i < N; i++) {
      MapPoint* pMP = pFrame->mvpMapPoints[i];
      if (pMP) {
        // Conventional SLAM
        if (!pFrame->mpCamera2) {
          if (pFrame->mvuRight[i] < 0) { // Monocular observation
            nInitialCorrespondences++;
            pFrame->mvbOutlier[i] = false;

            Eigen::Matrix<double, 2, 1> obs;
            const cv::KeyPoint&         kpUn = pFrame->mvKeysUn[i];
            obs << kpUn.pt.x, kpUn.pt.y;

            ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose* e = new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaMono);

            e->pCamera = pFrame->mpCamera;
            e->Xw      = pMP->GetWorldPos().cast<double>();

            optimizer.addEdge(e);

            vpEdgesMono.push_back(e);
            vnIndexEdgeMono.push_back(i);
          } else { // Stereo observation
            nInitialCorrespondences++;
            pFrame->mvbOutlier[i] = false;

            Eigen::Matrix<double, 3, 1> obs;
            const cv::KeyPoint&         kpUn  = pFrame->mvKeysUn[i];
            const float&                kp_ur = pFrame->mvuRight[i];
            obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float     invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            Eigen::Matrix3d Info      = Eigen::Matrix3d::Identity() * invSigma2;
            e->setInformation(Info);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaStereo);

            e->fx = pFrame->fx;
            e->fy = pFrame->fy;
            e->cx = pFrame->cx;
            e->cy = pFrame->cy;
            e->bf = pFrame->mbf;
            e->Xw = pMP->GetWorldPos().cast<double>();

            optimizer.addEdge(e);

            vpEdgesStereo.push_back(e);
            vnIndexEdgeStereo.push_back(i);
          }
        } else { // SLAM with respect a rigid body
          nInitialCorrespondences++;

          cv::KeyPoint kpUn;

          if (i < pFrame->Nleft) { // Left camera observation
            kpUn = pFrame->mvKeys[i];

            pFrame->mvbOutlier[i] = false;

            Eigen::Matrix<double, 2, 1> obs;
            obs << kpUn.pt.x, kpUn.pt.y;

            ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose* e = new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaMono);

            e->pCamera = pFrame->mpCamera;
            e->Xw      = pMP->GetWorldPos().cast<double>();

            optimizer.addEdge(e);

            vpEdgesMono.push_back(e);
            vnIndexEdgeMono.push_back(i);
          } else {
            kpUn = pFrame->mvKeysRight[i - pFrame->Nleft];

            Eigen::Matrix<double, 2, 1> obs;
            obs << kpUn.pt.x, kpUn.pt.y;

            pFrame->mvbOutlier[i] = false;

            ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody* e
              = new ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
            e->setMeasurement(obs);
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaMono);

            e->pCamera = pFrame->mpCamera2;
            e->Xw      = pMP->GetWorldPos().cast<double>();

            e->mTrl = g2o::SE3Quat(
              pFrame->GetRelativePoseTrl().unit_quaternion().cast<double>(),
              pFrame->GetRelativePoseTrl().translation().cast<double>()
            );

            optimizer.addEdge(e);

            vpEdgesMono_FHR.push_back(e);
            vnIndexEdgeRight.push_back(i);
          }
        }
      }
    }
  }

  if (nInitialCorrespondences < 3) {
    return 0;
  }

  // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
  // At the next optimization, outliers are not included, but at the end they can be classified as
  // inliers again.
  const float chi2Mono[4]   = {5.991, 5.991, 5.991, 5.991};
  const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
  const int   its[4]        = {10, 10, 10, 10};

  int nBad = 0;
  for (std::size_t it = 0; it < 4; it++) {
    Tcw = pFrame->GetPose();
    vSE3->setEstimate(
      g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>())
    );

    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);

    nBad = 0;
    for (std::size_t i = 0; i < vpEdgesMono.size(); i++) {
      ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

      const std::size_t idx = vnIndexEdgeMono[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if (chi2 > chi2Mono[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBad++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
      }

      if (it == 2) {
        e->setRobustKernel(0);
      }
    }

    for (std::size_t i = 0; i < vpEdgesMono_FHR.size(); i++) {
      ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody* e = vpEdgesMono_FHR[i];

      const std::size_t idx = vnIndexEdgeRight[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if (chi2 > chi2Mono[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBad++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
      }

      if (it == 2) {
        e->setRobustKernel(0);
      }
    }

    for (std::size_t i = 0; i < vpEdgesStereo.size(); i++) {
      g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

      const std::size_t idx = vnIndexEdgeStereo[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if (chi2 > chi2Stereo[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBad++;
      } else {
        e->setLevel(0);
        pFrame->mvbOutlier[idx] = false;
      }

      if (it == 2) {
        e->setRobustKernel(0);
      }
    }

    if (optimizer.edges().size() < 10) {
      break;
    }
  }

  // Recover optimized pose and return number of inliers
  g2o::VertexSE3Expmap* vSE3_recov    = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
  g2o::SE3Quat          SE3quat_recov = vSE3_recov->estimate();
  Sophus::SE3<float>    pose(
    SE3quat_recov.rotation().cast<float>(),
    SE3quat_recov.translation().cast<float>()
  );
  pFrame->SetPose(pose);

  return nInitialCorrespondences - nBad;
}

void Optimizer::LocalBundleAdjustment(
  KeyFrame* pKF,
  bool*     pbStopFlag,
  Map*      pMap,
  int&      num_fixedKF,
  int&      num_OptKF,
  int&      num_MPs,
  int&      num_edges
) {
  // Local KeyFrames: First Breath Search from Current Keyframe
  std::list<KeyFrame*> lLocalKeyFrames;

  lLocalKeyFrames.push_back(pKF);
  pKF->mnBALocalForKF = pKF->mnId;
  Map* pCurrentMap    = pKF->GetMap();

  for (KeyFrame* const kf : pKF->GetVectorCovisibleKeyFrames()) {
    kf->mnBALocalForKF = pKF->mnId;
    if (!kf->isBad() && kf->GetMap() == pCurrentMap) {
      lLocalKeyFrames.push_back(kf);
    }
  }

  // Local MapPoints seen in Local KeyFrames
  num_fixedKF = 0;
  std::list<MapPoint*> lLocalMapPoints;
  std::set<MapPoint*>  sNumObsMP;
  for (KeyFrame* const kf : lLocalKeyFrames) {
    if (kf->mnId == pMap->GetInitKFid()) {
      num_fixedKF = 1;
    }
    for (MapPoint* const mp : kf->GetMapPointMatches()) {
      if (mp) {
        if (!mp->isBad() && mp->GetMap() == pCurrentMap) {
          if (mp->mnBALocalForKF != pKF->mnId) {
            lLocalMapPoints.push_back(mp);
            mp->mnBALocalForKF = pKF->mnId;
          }
        }
      }
    }
  }

  // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
  std::list<KeyFrame*> lFixedCameras;
  for (MapPoint* const mp : lLocalMapPoints) {
    auto observations = mp->GetObservations();
    for (KeyFrame* const kf : observations | std::views::keys) {
      if (kf->mnBALocalForKF != pKF->mnId && kf->mnBAFixedForKF != pKF->mnId) {
        kf->mnBAFixedForKF = pKF->mnId;
        if (!kf->isBad() && kf->GetMap() == pCurrentMap) {
          lFixedCameras.push_back(kf);
        }
      }
    }
  }
  num_fixedKF = lFixedCameras.size() + num_fixedKF;

  if (num_fixedKF == 0) {
    // No fixed key frame in optimization, so abort local bundle adjustment.
    return;
  }

  // Setup optimizer
  g2o::SparseOptimizer                    optimizer;
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver;

  linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  if (pMap->IsInertial()) {
    solver->setUserLambdaInit(100.0);
  }

  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  if (pbStopFlag) {
    optimizer.setForceStopFlag(pbStopFlag);
  }

  unsigned long maxKFid = 0;

  // DEBUG LBA
  pCurrentMap->msOptKFs.clear();
  pCurrentMap->msFixedKFs.clear();

  // Set Local KeyFrame vertices
  for (KeyFrame* const kf : lLocalKeyFrames) {
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float>    Tcw  = kf->GetPose();
    vSE3->setEstimate(
      g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>())
    );
    vSE3->setId(kf->mnId);
    vSE3->setFixed(kf->mnId == pMap->GetInitKFid());
    optimizer.addVertex(vSE3);
    if (kf->mnId > maxKFid) {
      maxKFid = kf->mnId;
    }
    // DEBUG LBA
    pCurrentMap->msOptKFs.insert(kf->mnId);
  }
  num_OptKF = lLocalKeyFrames.size();

  // Set Fixed KeyFrame vertices
  for (KeyFrame* const kf : lFixedCameras) {
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float>    Tcw  = kf->GetPose();
    vSE3->setEstimate(
      g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>())
    );
    vSE3->setId(kf->mnId);
    vSE3->setFixed(true);
    optimizer.addVertex(vSE3);
    if (kf->mnId > maxKFid) {
      maxKFid = kf->mnId;
    }
    // DEBUG LBA
    pCurrentMap->msFixedKFs.insert(kf->mnId);
  }

  // Set MapPoint vertices
  const int nExpectedSize
    = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

  std::vector<ORB_SLAM3::EdgeSE3ProjectXYZ*> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);

  std::vector<ORB_SLAM3::EdgeSE3ProjectXYZToBody*> vpEdgesBody;
  vpEdgesBody.reserve(nExpectedSize);

  std::vector<KeyFrame*> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);

  std::vector<KeyFrame*> vpEdgeKFBody;
  vpEdgeKFBody.reserve(nExpectedSize);

  std::vector<MapPoint*> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  std::vector<MapPoint*> vpMapPointEdgeBody;
  vpMapPointEdgeBody.reserve(nExpectedSize);

  std::vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);

  std::vector<KeyFrame*> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);

  std::vector<MapPoint*> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  const float thHuberMono   = std::sqrt(5.991);
  const float thHuberStereo = std::sqrt(7.815);

  int nPoints = 0;

  int nEdges = 0;

  for (MapPoint* const mp : lLocalMapPoints) {
    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(mp->GetWorldPos().cast<double>());
    int id = mp->mnId + maxKFid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);
    nPoints++;

    auto observations = mp->GetObservations();

    // Set edges
    for (const auto& [kf, indices] : observations) {
      if (!kf->isBad() && kf->GetMap() == pCurrentMap) {
        auto [left_id, right_id] = indices;

        // Monocular observation
        if (left_id != -1 && kf->mvuRight[left_id] < 0) {
          const cv::KeyPoint&         kpUn = kf->mvKeysUn[left_id];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          ORB_SLAM3::EdgeSE3ProjectXYZ* e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(kf->mnId)));
          e->setMeasurement(obs);
          const float& invSigma2 = kf->mvInvLevelSigma2[kpUn.octave];
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          e->pCamera = kf->mpCamera;

          optimizer.addEdge(e);
          vpEdgesMono.push_back(e);
          vpEdgeKFMono.push_back(kf);
          vpMapPointEdgeMono.push_back(mp);

          nEdges++;
        } else if (left_id != -1 && kf->mvuRight[left_id] >= 0) { // Stereo observation
          const cv::KeyPoint&         kpUn = kf->mvKeysUn[left_id];
          Eigen::Matrix<double, 3, 1> obs;
          const float                 kp_ur = kf->mvuRight[left_id];
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(kf->mnId)));
          e->setMeasurement(obs);
          const float&    invSigma2 = kf->mvInvLevelSigma2[kpUn.octave];
          Eigen::Matrix3d Info      = Eigen::Matrix3d::Identity() * invSigma2;
          e->setInformation(Info);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          e->fx = kf->fx;
          e->fy = kf->fy;
          e->cx = kf->cx;
          e->cy = kf->cy;
          e->bf = kf->mbf;

          optimizer.addEdge(e);
          vpEdgesStereo.push_back(e);
          vpEdgeKFStereo.push_back(kf);
          vpMapPointEdgeStereo.push_back(mp);

          nEdges++;
        }

        if (kf->mpCamera2) {
          if (right_id != -1) {
            right_id -= kf->NLeft;

            Eigen::Matrix<double, 2, 1> obs;
            cv::KeyPoint                kp = kf->mvKeysRight[right_id];
            obs << kp.pt.x, kp.pt.y;

            ORB_SLAM3::EdgeSE3ProjectXYZToBody* e = new ORB_SLAM3::EdgeSE3ProjectXYZToBody();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setVertex(
              1,
              dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(kf->mnId))
            );
            e->setMeasurement(obs);
            const float& invSigma2 = kf->mvInvLevelSigma2[kp.octave];
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberMono);

            Sophus::SE3f Trl = kf->GetRelativePoseTrl();
            e->mTrl          = g2o::SE3Quat(
              Trl.unit_quaternion().cast<double>(),
              Trl.translation().cast<double>()
            );

            e->pCamera = kf->mpCamera2;

            optimizer.addEdge(e);
            vpEdgesBody.push_back(e);
            vpEdgeKFBody.push_back(kf);
            vpMapPointEdgeBody.push_back(mp);

            nEdges++;
          }
        }
      }
    }
  }
  num_edges = nEdges;

  if (pbStopFlag) {
    if (*pbStopFlag) {
      return;
    }
  }

  optimizer.initializeOptimization();
  optimizer.optimize(10);

  std::vector<std::pair<KeyFrame*, MapPoint*>> vToErase;
  vToErase.reserve(vpEdgesMono.size() + vpEdgesBody.size() + vpEdgesStereo.size());

  // Check inlier observations
  for (std::size_t i = 0; i < vpEdgesMono.size(); i++) {
    ORB_SLAM3::EdgeSE3ProjectXYZ* e   = vpEdgesMono[i];
    MapPoint*                     pMP = vpMapPointEdgeMono[i];

    if (pMP->isBad()) {
      continue;
    }

    if (e->chi2() > 5.991 || !e->isDepthPositive()) {
      KeyFrame* pKFi = vpEdgeKFMono[i];
      vToErase.push_back(std::make_pair(pKFi, pMP));
    }
  }

  for (std::size_t i = 0; i < vpEdgesBody.size(); i++) {
    ORB_SLAM3::EdgeSE3ProjectXYZToBody* e   = vpEdgesBody[i];
    MapPoint*                           pMP = vpMapPointEdgeBody[i];

    if (pMP->isBad()) {
      continue;
    }

    if (e->chi2() > 5.991 || !e->isDepthPositive()) {
      KeyFrame* pKFi = vpEdgeKFBody[i];
      vToErase.push_back(std::make_pair(pKFi, pMP));
    }
  }

  for (std::size_t i = 0; i < vpEdgesStereo.size(); i++) {
    g2o::EdgeStereoSE3ProjectXYZ* e   = vpEdgesStereo[i];
    MapPoint*                     pMP = vpMapPointEdgeStereo[i];

    if (pMP->isBad()) {
      continue;
    }

    if (e->chi2() > 7.815 || !e->isDepthPositive()) {
      KeyFrame* pKFi = vpEdgeKFStereo[i];
      vToErase.push_back(std::make_pair(pKFi, pMP));
    }
  }

  // Get Map Mutex
  std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);

  if (!vToErase.empty()) {
    for (const auto& [kf, mp] : vToErase) {
      kf->EraseMapPointMatch(mp);
      mp->EraseObservation(kf);
    }
  }

  // Recover optimized data
  // Keyframes
  for (KeyFrame* const kf : lLocalKeyFrames) {
    g2o::VertexSE3Expmap* vSE3    = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(kf->mnId));
    g2o::SE3Quat          SE3quat = vSE3->estimate();
    Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(), SE3quat.translation().cast<float>());
    kf->SetPose(Tiw);
  }

  // Points
  for (MapPoint* const mp : lLocalMapPoints) {
    g2o::VertexSBAPointXYZ* vPoint
      = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(mp->mnId + maxKFid + 1));
    mp->SetWorldPos(vPoint->estimate().cast<float>());
    mp->UpdateNormalAndDepth();
  }

  pMap->IncreaseChangeIndex();
}

void Optimizer::OptimizeEssentialGraph(
  Map*                                            pMap,
  KeyFrame*                                       pLoopKF,
  KeyFrame*                                       pCurKF,
  const LoopClosing::KeyFrameAndPose&             NonCorrectedSim3,
  const LoopClosing::KeyFrameAndPose&             CorrectedSim3,
  const std::map<KeyFrame*, std::set<KeyFrame*>>& LoopConnections,
  const bool&                                     bFixScale
) {
  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  g2o::BlockSolver_7_3::LinearSolverType* linearSolver
    = new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
  g2o::BlockSolver_7_3*                solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  solver->setUserLambdaInit(1e-16);
  optimizer.setAlgorithm(solver);

  const std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
  const std::vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

  const unsigned int nMaxKFid = pMap->GetMaxKFid();

  std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
  std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(nMaxKFid + 1);
  std::vector<g2o::VertexSim3Expmap*>                         vpVertices(nMaxKFid + 1);

  std::vector<Eigen::Vector3d> vZvectors(nMaxKFid + 1); // For debugging
  Eigen::Vector3d              z_vec;
  z_vec << 0.0, 0.0, 1.0;

  const int minFeat = 100;

  // Set KeyFrame vertices
  for (KeyFrame* const kf : vpKFs) {
    if (kf->isBad()) {
      continue;
    }
    g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

    const int nIDi = kf->mnId;

    auto it = CorrectedSim3.find(kf);

    if (it != CorrectedSim3.end()) {
      vScw[nIDi] = it->second;
      VSim3->setEstimate(it->second);
    } else {
      Sophus::SE3d Tcw = kf->GetPose().cast<double>();
      g2o::Sim3    Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);
      vScw[nIDi] = Siw;
      VSim3->setEstimate(Siw);
    }

    if (kf->mnId == pMap->GetInitKFid()) {
      VSim3->setFixed(true);
    }

    VSim3->setId(nIDi);
    VSim3->setMarginalized(false);
    VSim3->_fix_scale = bFixScale;

    optimizer.addVertex(VSim3);
    vZvectors[nIDi] = vScw[nIDi].rotation() * z_vec; // For debugging

    vpVertices[nIDi] = VSim3;
  }

  std::set<std::pair<long unsigned int, long unsigned int>> sInsertedEdges;

  const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();

  // Set Loop edges
  int count_loop = 0;
  for (const auto& [kf, connections] : LoopConnections) {
    const long unsigned int nIDi = kf->mnId;
    const g2o::Sim3         Siw  = vScw[nIDi];
    const g2o::Sim3         Swi  = Siw.inverse();

    for (KeyFrame* const connected_kf : connections) {
      const long unsigned int nIDj = connected_kf->mnId;
      if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) && kf->GetWeight(connected_kf) < minFeat) {
        continue;
      }

      const g2o::Sim3 Sjw = vScw[nIDj];
      const g2o::Sim3 Sji = Sjw * Swi;

      g2o::EdgeSim3* e = new g2o::EdgeSim3();
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
      e->setMeasurement(Sji);

      e->information() = matLambda;

      optimizer.addEdge(e);
      count_loop++;
      sInsertedEdges.insert(std::make_pair(std::min(nIDi, nIDj), std::max(nIDi, nIDj)));
    }
  }

  // Set normal edges
  for (KeyFrame* const kf : vpKFs) {
    const int nIDi = kf->mnId;

    g2o::Sim3 Swi;

    auto iti = NonCorrectedSim3.find(kf);

    if (iti != NonCorrectedSim3.end()) {
      Swi = (iti->second).inverse();
    } else {
      Swi = vScw[nIDi].inverse();
    }

    KeyFrame* pParentKF = kf->GetParent();

    // Spanning tree edge
    if (pParentKF) {
      int nIDj = pParentKF->mnId;

      g2o::Sim3 Sjw;

      auto itj = NonCorrectedSim3.find(pParentKF);

      if (itj != NonCorrectedSim3.end()) {
        Sjw = itj->second;
      } else {
        Sjw = vScw[nIDj];
      }

      g2o::Sim3 Sji = Sjw * Swi;

      g2o::EdgeSim3* e = new g2o::EdgeSim3();
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
      e->setMeasurement(Sji);
      e->information() = matLambda;
      optimizer.addEdge(e);
    }

    // Loop edges
    const std::set<KeyFrame*> sLoopEdges = kf->GetLoopEdges();
    for (KeyFrame* const loop_kf : sLoopEdges) {
      if (loop_kf->mnId < kf->mnId) {
        g2o::Sim3 Slw;

        auto itl = NonCorrectedSim3.find(loop_kf);

        if (itl != NonCorrectedSim3.end()) {
          Slw = itl->second;
        } else {
          Slw = vScw[loop_kf->mnId];
        }

        g2o::Sim3      Sli = Slw * Swi;
        g2o::EdgeSim3* el  = new g2o::EdgeSim3();
        el->setVertex(
          1,
          dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(loop_kf->mnId))
        );
        el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
        el->setMeasurement(Sli);
        el->information() = matLambda;
        optimizer.addEdge(el);
      }
    }

    // Covisibility graph edges
    for (KeyFrame* const covisible_kf : kf->GetCovisiblesByWeight(minFeat)) {
      if (covisible_kf && covisible_kf != pParentKF && !kf->hasChild(covisible_kf) /*&& !sLoopEdges.count(pKFn)*/) {
        if (!covisible_kf->isBad() && covisible_kf->mnId < kf->mnId) {
          if (sInsertedEdges.count(std::make_pair(
                std::min(kf->mnId, covisible_kf->mnId),
                std::max(kf->mnId, covisible_kf->mnId)
              ))) {
            continue;
          }

          g2o::Sim3 Snw;

          auto itn = NonCorrectedSim3.find(covisible_kf);

          if (itn != NonCorrectedSim3.end()) {
            Snw = itn->second;
          } else {
            Snw = vScw[covisible_kf->mnId];
          }

          g2o::Sim3 Sni = Snw * Swi;

          g2o::EdgeSim3* en = new g2o::EdgeSim3();
          en->setVertex(
            1,
            dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(covisible_kf->mnId))
          );
          en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
          en->setMeasurement(Sni);
          en->information() = matLambda;
          optimizer.addEdge(en);
        }
      }
    }

    // Inertial edges if inertial
    if (kf->bImu && kf->mPrevKF) {
      g2o::Sim3 Spw;
      auto      itp = NonCorrectedSim3.find(kf->mPrevKF);
      if (itp != NonCorrectedSim3.end()) {
        Spw = itp->second;
      } else {
        Spw = vScw[kf->mPrevKF->mnId];
      }

      g2o::Sim3      Spi = Spw * Swi;
      g2o::EdgeSim3* ep  = new g2o::EdgeSim3();
      ep->setVertex(
        1,
        dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(kf->mPrevKF->mnId))
      );
      ep->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
      ep->setMeasurement(Spi);
      ep->information() = matLambda;
      optimizer.addEdge(ep);
    }
  }

  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  optimizer.optimize(20);
  optimizer.computeActiveErrors();
  std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);

  // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
  for (KeyFrame* const kf : vpKFs) {
    const int nIDi = kf->mnId;

    g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
    g2o::Sim3              CorrectedSiw = VSim3->estimate();
    vCorrectedSwc[nIDi]                 = CorrectedSiw.inverse();
    double s                            = CorrectedSiw.scale();

    Sophus::SE3f Tiw(
      CorrectedSiw.rotation().cast<float>(),
      CorrectedSiw.translation().cast<float>() / s
    );
    kf->SetPose(Tiw);
  }

  // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with
  // optimized pose
  for (MapPoint* const mp : vpMPs) {
    if (mp->isBad()) {
      continue;
    }

    int nIDr;
    if (mp->mnCorrectedByKF == pCurKF->mnId) {
      nIDr = mp->mnCorrectedReference;
    } else {
      KeyFrame* pRefKF = mp->GetReferenceKeyFrame();
      nIDr             = pRefKF->mnId;
    }

    g2o::Sim3 Srw          = vScw[nIDr];
    g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

    Eigen::Matrix<double, 3, 1> eigP3Dw          = mp->GetWorldPos().cast<double>();
    Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));
    mp->SetWorldPos(eigCorrectedP3Dw.cast<float>());

    mp->UpdateNormalAndDepth();
  }

  // TODO Check this changeindex
  pMap->IncreaseChangeIndex();
}

void Optimizer::OptimizeEssentialGraph(
  KeyFrame*               pCurKF,
  std::vector<KeyFrame*>& vpFixedKFs,
  std::vector<KeyFrame*>& vpFixedCorrectedKFs,
  std::vector<KeyFrame*>& vpNonFixedKFs,
  std::vector<MapPoint*>& vpNonCorrectedMPs
) {
  // vpFixedKFs: fixed key frames in merged map.
  // vpFixedCorrectedKFs: fixed key frames in old map.
  // vpNonFixedKFs: non-fixed key frames in merged map.
  // vpNonCorrectedMPs: non-corrected map points in merged map.

  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  g2o::BlockSolver_7_3::LinearSolverType* linearSolver
    = new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
  g2o::BlockSolver_7_3*                solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  solver->setUserLambdaInit(1e-16);
  optimizer.setAlgorithm(solver);

  Map*               pMap     = pCurKF->GetMap();
  const unsigned int nMaxKFid = pMap->GetMaxKFid();

  std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
  std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(nMaxKFid + 1);
  std::vector<g2o::VertexSim3Expmap*>                         vpVertices(nMaxKFid + 1);

  std::vector<bool> vpGoodPose(nMaxKFid + 1);
  std::vector<bool> vpBadPose(nMaxKFid + 1);

  const int minFeat = 100;

  // Load vpFixedKFs.
  for (KeyFrame* const kf : vpFixedKFs) {
    if (kf->isBad()) {
      continue;
    }

    g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

    const int nIDi = kf->mnId;

    Sophus::SE3d Tcw = kf->GetPose().cast<double>();
    g2o::Sim3    Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

    vCorrectedSwc[nIDi] = Siw.inverse();
    VSim3->setEstimate(Siw);

    VSim3->setFixed(true);

    VSim3->setId(nIDi);
    VSim3->setMarginalized(false);
    VSim3->_fix_scale = true;

    optimizer.addVertex(VSim3);

    vpVertices[nIDi] = VSim3;

    vpGoodPose[nIDi] = true;
    vpBadPose[nIDi]  = false;
  }

  std::set<unsigned long> sIdKF;
  for (KeyFrame* const kf : vpFixedCorrectedKFs) {
    if (kf->isBad()) {
      continue;
    }

    g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

    const int nIDi = kf->mnId;

    Sophus::SE3d Tcw = kf->GetPose().cast<double>();
    g2o::Sim3    Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

    vCorrectedSwc[nIDi] = Siw.inverse();
    VSim3->setEstimate(Siw);

    Sophus::SE3d Tcw_bef = kf->mTcwBefMerge.cast<double>();
    vScw[nIDi]           = g2o::Sim3(Tcw_bef.unit_quaternion(), Tcw_bef.translation(), 1.0);

    VSim3->setFixed(true);

    VSim3->setId(nIDi);
    VSim3->setMarginalized(false);

    optimizer.addVertex(VSim3);

    vpVertices[nIDi] = VSim3;

    sIdKF.insert(nIDi);

    vpGoodPose[nIDi] = true;
    vpBadPose[nIDi]  = true;
  }

  for (KeyFrame* const kf : vpNonFixedKFs) {
    if (kf->isBad()) {
      continue;
    }

    const int nIDi = kf->mnId;

    if (sIdKF.count(nIDi)) { // It has already added in the corrected merge KFs
      continue;
    }

    g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

    Sophus::SE3d Tcw = kf->GetPose().cast<double>();
    g2o::Sim3    Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

    vScw[nIDi] = Siw;
    VSim3->setEstimate(Siw);

    VSim3->setFixed(false);

    VSim3->setId(nIDi);
    VSim3->setMarginalized(false);

    optimizer.addVertex(VSim3);

    vpVertices[nIDi] = VSim3;

    sIdKF.insert(nIDi);

    vpGoodPose[nIDi] = false;
    vpBadPose[nIDi]  = true;
  }

  std::vector<KeyFrame*> vpKFs;
  vpKFs.reserve(vpFixedKFs.size() + vpFixedCorrectedKFs.size() + vpNonFixedKFs.size());
  vpKFs.insert(vpKFs.end(), vpFixedKFs.begin(), vpFixedKFs.end());
  vpKFs.insert(vpKFs.end(), vpFixedCorrectedKFs.begin(), vpFixedCorrectedKFs.end());
  vpKFs.insert(vpKFs.end(), vpNonFixedKFs.begin(), vpNonFixedKFs.end());
  std::set<KeyFrame*> spKFs(vpKFs.begin(), vpKFs.end());

  const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();

  for (KeyFrame* const kf : vpKFs) {
    int       num_connections = 0;
    const int nIDi            = kf->mnId;

    g2o::Sim3 correctedSwi;
    g2o::Sim3 Swi;

    if (vpGoodPose[nIDi]) {
      correctedSwi = vCorrectedSwc[nIDi];
    }
    if (vpBadPose[nIDi]) {
      Swi = vScw[nIDi].inverse();
    }

    KeyFrame* pParentKFi = kf->GetParent();

    // Spanning tree edge
    if (pParentKFi && spKFs.find(pParentKFi) != spKFs.end()) {
      int nIDj = pParentKFi->mnId;

      g2o::Sim3 Sjw;
      bool      bHasRelation = false;

      if (vpGoodPose[nIDi] && vpGoodPose[nIDj]) {
        Sjw          = vCorrectedSwc[nIDj].inverse();
        bHasRelation = true;
      } else if (vpBadPose[nIDi] && vpBadPose[nIDj]) {
        Sjw          = vScw[nIDj];
        bHasRelation = true;
      }

      if (bHasRelation) {
        g2o::Sim3 Sji = Sjw * Swi;

        g2o::EdgeSim3* e = new g2o::EdgeSim3();
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
        e->setMeasurement(Sji);

        e->information() = matLambda;
        optimizer.addEdge(e);
        num_connections++;
      }
    }

    // Loop edges
    const std::set<KeyFrame*> sLoopEdges = kf->GetLoopEdges();
    for (KeyFrame* const loop_kf : sLoopEdges) {
      if (spKFs.find(loop_kf) != spKFs.end() && loop_kf->mnId < kf->mnId) {
        g2o::Sim3 Slw;
        bool      bHasRelation = false;

        if (vpGoodPose[nIDi] && vpGoodPose[loop_kf->mnId]) {
          Slw          = vCorrectedSwc[loop_kf->mnId].inverse();
          bHasRelation = true;
        } else if (vpBadPose[nIDi] && vpBadPose[loop_kf->mnId]) {
          Slw          = vScw[loop_kf->mnId];
          bHasRelation = true;
        }

        if (bHasRelation) {
          g2o::Sim3      Sli = Slw * Swi;
          g2o::EdgeSim3* el  = new g2o::EdgeSim3();
          el->setVertex(
            1,
            dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(loop_kf->mnId))
          );
          el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
          el->setMeasurement(Sli);
          el->information() = matLambda;
          optimizer.addEdge(el);
          num_connections++;
        }
      }
    }

    // Covisibility graph edges
    for (KeyFrame* const covisible_kf : kf->GetCovisiblesByWeight(minFeat)) {
      if (covisible_kf && covisible_kf != pParentKFi && !kf->hasChild(covisible_kf) && !sLoopEdges.count(covisible_kf) && spKFs.find(covisible_kf) != spKFs.end()) {
        if (!covisible_kf->isBad() && covisible_kf->mnId < kf->mnId) {
          g2o::Sim3 Snw          = vScw[covisible_kf->mnId];
          bool      bHasRelation = false;

          if (vpGoodPose[nIDi] && vpGoodPose[covisible_kf->mnId]) {
            Snw          = vCorrectedSwc[covisible_kf->mnId].inverse();
            bHasRelation = true;
          } else if (vpBadPose[nIDi] && vpBadPose[covisible_kf->mnId]) {
            Snw          = vScw[covisible_kf->mnId];
            bHasRelation = true;
          }

          if (bHasRelation) {
            g2o::Sim3 Sni = Snw * Swi;

            g2o::EdgeSim3* en = new g2o::EdgeSim3();
            en->setVertex(
              1,
              dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(covisible_kf->mnId))
            );
            en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            en->setMeasurement(Sni);
            en->information() = matLambda;
            optimizer.addEdge(en);
            num_connections++;
          }
        }
      }
    }

    if (num_connections == 0) {
      // Key frame has no connection.
    }
  }

  // Optimize!
  optimizer.initializeOptimization();
  optimizer.optimize(20);

  std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);

  // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
  for (KeyFrame* const kf : vpNonFixedKFs) {
    if (kf->isBad()) {
      continue;
    }

    const int nIDi = kf->mnId;

    g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(nIDi));
    g2o::Sim3              CorrectedSiw = VSim3->estimate();
    vCorrectedSwc[nIDi]                 = CorrectedSiw.inverse();
    double       s                      = CorrectedSiw.scale();
    Sophus::SE3d Tiw(CorrectedSiw.rotation(), CorrectedSiw.translation() / s);

    kf->mTcwBefMerge = kf->GetPose();
    kf->mTwcBefMerge = kf->GetPoseInverse();
    kf->SetPose(Tiw.cast<float>());
  }

  // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with
  // optimized pose
  for (MapPoint* const mp : vpNonCorrectedMPs) {
    if (mp->isBad()) {
      continue;
    }

    KeyFrame* pRefKF = mp->GetReferenceKeyFrame();
    while (pRefKF->isBad()) {
      if (!pRefKF) {
        // Map point has no valid reference key frame.
        break;
      }

      mp->EraseObservation(pRefKF);
      pRefKF = mp->GetReferenceKeyFrame();
    }

    if (vpBadPose[pRefKF->mnId]) {
      Sophus::SE3f TNonCorrectedwr = pRefKF->mTwcBefMerge;
      Sophus::SE3f Twr             = pRefKF->GetPoseInverse();

      Eigen::Vector3f eigCorrectedP3Dw = Twr * TNonCorrectedwr.inverse() * mp->GetWorldPos();
      mp->SetWorldPos(eigCorrectedP3Dw);

      mp->UpdateNormalAndDepth();
    } else {
      // (error) Map point has reference key frame from another map.
    }
  }
}

int Optimizer::OptimizeSim3(
  KeyFrame*                    pKF1,
  KeyFrame*                    pKF2,
  std::vector<MapPoint*>&      vpMatches1,
  g2o::Sim3&                   g2oS12,
  const float                  th2,
  const bool                   bFixScale,
  Eigen::Matrix<double, 7, 7>& mAcumHessian,
  const bool                   bAllPoints
) {
  g2o::SparseOptimizer                 optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  // Camera poses
  const Eigen::Matrix3f R1w = pKF1->GetRotation();
  const Eigen::Vector3f t1w = pKF1->GetTranslation();
  const Eigen::Matrix3f R2w = pKF2->GetRotation();
  const Eigen::Vector3f t2w = pKF2->GetTranslation();

  // Set Sim3 vertex
  ORB_SLAM3::VertexSim3Expmap* vSim3 = new ORB_SLAM3::VertexSim3Expmap();
  vSim3->_fix_scale                  = bFixScale;
  vSim3->setEstimate(g2oS12);
  vSim3->setId(0);
  vSim3->setFixed(false);
  vSim3->pCamera1 = pKF1->mpCamera;
  vSim3->pCamera2 = pKF2->mpCamera;
  optimizer.addVertex(vSim3);

  // Set MapPoint vertices
  const int                                          N            = vpMatches1.size();
  const std::vector<MapPoint*>                       vpMapPoints1 = pKF1->GetMapPointMatches();
  std::vector<ORB_SLAM3::EdgeSim3ProjectXYZ*>        vpEdges12;
  std::vector<ORB_SLAM3::EdgeInverseSim3ProjectXYZ*> vpEdges21;
  std::vector<std::size_t>                           vnIndexEdge;
  std::vector<bool>                                  vbIsInKF2;

  vnIndexEdge.reserve(2 * N);
  vpEdges12.reserve(2 * N);
  vpEdges21.reserve(2 * N);
  vbIsInKF2.reserve(2 * N);

  const float deltaHuber = std::sqrt(th2);

  int nCorrespondences = 0;
  int nBadMPs          = 0;
  int nInKF2           = 0;
  int nOutKF2          = 0;
  int nMatchWithoutMP  = 0;

  std::vector<int> vIdsOnlyInKF2;

  for (int i = 0; i < N; i++) {
    if (!vpMatches1[i]) {
      continue;
    }

    MapPoint* pMP1 = vpMapPoints1[i];
    MapPoint* pMP2 = vpMatches1[i];

    const int id1 = 2 * i + 1;
    const int id2 = 2 * (i + 1);

    const int i2 = std::get<0>(pMP2->GetIndexInKeyFrame(pKF2));

    Eigen::Vector3f P3D1c;
    Eigen::Vector3f P3D2c;

    if (pMP1 && pMP2) {
      if (!pMP1->isBad() && !pMP2->isBad()) {
        g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
        Eigen::Vector3f         P3D1w   = pMP1->GetWorldPos();
        P3D1c                           = R1w * P3D1w + t1w;
        vPoint1->setEstimate(P3D1c.cast<double>());
        vPoint1->setId(id1);
        vPoint1->setFixed(true);
        optimizer.addVertex(vPoint1);

        g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
        Eigen::Vector3f         P3D2w   = pMP2->GetWorldPos();
        P3D2c                           = R2w * P3D2w + t2w;
        vPoint2->setEstimate(P3D2c.cast<double>());
        vPoint2->setId(id2);
        vPoint2->setFixed(true);
        optimizer.addVertex(vPoint2);
      } else {
        nBadMPs++;
        continue;
      }
    } else {
      nMatchWithoutMP++;

      // TODO The 3D position in KF1 doesn't exist
      if (!pMP2->isBad()) {
        g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
        Eigen::Vector3f         P3D2w   = pMP2->GetWorldPos();
        P3D2c                           = R2w * P3D2w + t2w;
        vPoint2->setEstimate(P3D2c.cast<double>());
        vPoint2->setId(id2);
        vPoint2->setFixed(true);
        optimizer.addVertex(vPoint2);

        vIdsOnlyInKF2.push_back(id2);
      }
      continue;
    }

    if (i2 < 0 && !bAllPoints) {
      continue;
    }

    if (P3D2c(2) < 0) {
      // Skip due to negative depth.
      continue;
    }

    nCorrespondences++;

    // Set edge x1 = S12*X2
    Eigen::Matrix<double, 2, 1> obs1;
    const cv::KeyPoint&         kpUn1 = pKF1->mvKeysUn[i];
    obs1 << kpUn1.pt.x, kpUn1.pt.y;

    ORB_SLAM3::EdgeSim3ProjectXYZ* e12 = new ORB_SLAM3::EdgeSim3ProjectXYZ();

    e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2)));
    e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
    e12->setMeasurement(obs1);
    const float& invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
    e12->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare1);

    g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
    e12->setRobustKernel(rk1);
    rk1->setDelta(deltaHuber);
    optimizer.addEdge(e12);

    // Set edge x2 = S21*X1
    Eigen::Matrix<double, 2, 1> obs2;
    cv::KeyPoint                kpUn2;
    bool                        inKF2;
    if (i2 >= 0) {
      kpUn2 = pKF2->mvKeysUn[i2];
      obs2 << kpUn2.pt.x, kpUn2.pt.y;
      inKF2 = true;

      nInKF2++;
    } else {
      float invz = 1 / P3D2c(2);
      float x    = P3D2c(0) * invz;
      float y    = P3D2c(1) * invz;

      obs2 << x, y;
      kpUn2 = cv::KeyPoint(cv::Point2f(x, y), pMP2->mnTrackScaleLevel);

      inKF2 = false;
      nOutKF2++;
    }

    ORB_SLAM3::EdgeInverseSim3ProjectXYZ* e21 = new ORB_SLAM3::EdgeInverseSim3ProjectXYZ();

    e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));
    e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
    e21->setMeasurement(obs2);
    float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
    e21->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare2);

    g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
    e21->setRobustKernel(rk2);
    rk2->setDelta(deltaHuber);
    optimizer.addEdge(e21);

    vpEdges12.push_back(e12);
    vpEdges21.push_back(e21);
    vnIndexEdge.push_back(i);

    vbIsInKF2.push_back(inKF2);
  }

  // Optimize!
  optimizer.initializeOptimization();
  optimizer.optimize(5);

  // Check inliers
  int nBad       = 0;
  int nBadOutKF2 = 0;
  for (std::size_t i = 0; i < vpEdges12.size(); i++) {
    ORB_SLAM3::EdgeSim3ProjectXYZ*        e12 = vpEdges12[i];
    ORB_SLAM3::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
    if (!e12 || !e21) {
      continue;
    }

    if (e12->chi2() > th2 || e21->chi2() > th2) {
      std::size_t idx = vnIndexEdge[i];
      vpMatches1[idx] = static_cast<MapPoint*>(NULL);
      optimizer.removeEdge(e12);
      optimizer.removeEdge(e21);
      vpEdges12[i] = static_cast<ORB_SLAM3::EdgeSim3ProjectXYZ*>(NULL);
      vpEdges21[i] = static_cast<ORB_SLAM3::EdgeInverseSim3ProjectXYZ*>(NULL);
      nBad++;

      if (!vbIsInKF2[i]) {
        nBadOutKF2++;
      }
      continue;
    }

    // Check if remove the robust adjustment improve the result
    e12->setRobustKernel(0);
    e21->setRobustKernel(0);
  }

  int nMoreIterations;
  if (nBad > 0) {
    nMoreIterations = 10;
  } else {
    nMoreIterations = 5;
  }

  if (nCorrespondences - nBad < 10) {
    return 0;
  }

  // Optimize again only with inliers
  optimizer.initializeOptimization();
  optimizer.optimize(nMoreIterations);

  int nIn      = 0;
  mAcumHessian = Eigen::MatrixXd::Zero(7, 7);
  for (std::size_t i = 0; i < vpEdges12.size(); i++) {
    ORB_SLAM3::EdgeSim3ProjectXYZ*        e12 = vpEdges12[i];
    ORB_SLAM3::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
    if (!e12 || !e21) {
      continue;
    }

    e12->computeError();
    e21->computeError();

    if (e12->chi2() > th2 || e21->chi2() > th2) {
      std::size_t idx = vnIndexEdge[i];
      vpMatches1[idx] = static_cast<MapPoint*>(NULL);
    } else {
      nIn++;
    }
  }

  // Recover optimized Sim3
  g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
  g2oS12                             = vSim3_recov->estimate();

  return nIn;
}

void Optimizer::LocalInertialBA(
  KeyFrame* pKF,
  bool*     pbStopFlag,
  Map*      pMap,
  int&      num_fixedKF,
  int&      num_OptKF,
  int&      num_MPs,
  int&      num_edges,
  bool      bLarge,
  bool      bRecInit
) {
  Map* pCurrentMap = pKF->GetMap();

  int maxOpt = 10;
  int opt_it = 10;
  if (bLarge) {
    maxOpt = 25;
    opt_it = 4;
  }
  const int           Nd      = std::min((int)pCurrentMap->KeyFramesInMap() - 2, maxOpt);
  const unsigned long maxKFid = pKF->mnId;

  std::vector<KeyFrame*>       vpOptimizableKFs;
  const std::vector<KeyFrame*> vpNeighsKFs = pKF->GetVectorCovisibleKeyFrames();
  std::list<KeyFrame*>         lpOptVisKFs;

  vpOptimizableKFs.reserve(Nd);
  vpOptimizableKFs.push_back(pKF);
  pKF->mnBALocalForKF = pKF->mnId;
  for (int i = 1; i < Nd; i++) {
    if (vpOptimizableKFs.back()->mPrevKF) {
      vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
      vpOptimizableKFs.back()->mnBALocalForKF = pKF->mnId;
    } else {
      break;
    }
  }

  int N = vpOptimizableKFs.size();

  // Optimizable points seen by temporal optimizable keyframes
  std::list<MapPoint*> lLocalMapPoints;
  for (int i = 0; i < N; i++) {
    for (MapPoint* const mp : vpOptimizableKFs[i]->GetMapPointMatches()) {
      if (mp) {
        if (!mp->isBad()) {
          if (mp->mnBALocalForKF != pKF->mnId) {
            lLocalMapPoints.push_back(mp);
            mp->mnBALocalForKF = pKF->mnId;
          }
        }
      }
    }
  }

  // Fixed Keyframe: First frame previous KF to optimization window)
  std::list<KeyFrame*> lFixedKeyFrames;
  if (vpOptimizableKFs.back()->mPrevKF) {
    lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
    vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF = pKF->mnId;
  } else {
    vpOptimizableKFs.back()->mnBALocalForKF = 0;
    vpOptimizableKFs.back()->mnBAFixedForKF = pKF->mnId;
    lFixedKeyFrames.push_back(vpOptimizableKFs.back());
    vpOptimizableKFs.pop_back();
  }

  // Optimizable visual KFs
  const int maxCovKF = 0;
  for (KeyFrame* const kf : vpNeighsKFs) {
    if (lpOptVisKFs.size() >= maxCovKF) {
      break;
    }

    if (kf->mnBALocalForKF == pKF->mnId || kf->mnBAFixedForKF == pKF->mnId) {
      continue;
    }
    kf->mnBALocalForKF = pKF->mnId;
    if (!kf->isBad() && kf->GetMap() == pCurrentMap) {
      lpOptVisKFs.push_back(kf);

      for (MapPoint* const mp : kf->GetMapPointMatches()) {
        if (mp) {
          if (!mp->isBad()) {
            if (mp->mnBALocalForKF != pKF->mnId) {
              lLocalMapPoints.push_back(mp);
              mp->mnBALocalForKF = pKF->mnId;
            }
          }
        }
      }
    }
  }

  // Fixed KFs which are not covisible optimizable
  const int maxFixKF = 200;

  for (MapPoint* const mp : lLocalMapPoints) {
    auto observations = mp->GetObservations();
    for (KeyFrame* const kf : observations | std::views::keys) {
      if (kf->mnBALocalForKF != pKF->mnId && kf->mnBAFixedForKF != pKF->mnId) {
        kf->mnBAFixedForKF = pKF->mnId;
        if (!kf->isBad()) {
          lFixedKeyFrames.push_back(kf);
          break;
        }
      }
    }
    if (lFixedKeyFrames.size() >= maxFixKF) {
      break;
    }
  }

  bool bNonFixed = (lFixedKeyFrames.size() == 0);

  // Setup optimizer
  g2o::SparseOptimizer                 optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;
  linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  if (bLarge) {
    g2o::OptimizationAlgorithmLevenberg* solver
      = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    solver->setUserLambdaInit(1e-2); // to avoid iterating for finding optimal lambda
    optimizer.setAlgorithm(solver);
  } else {
    g2o::OptimizationAlgorithmLevenberg* solver
      = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    solver->setUserLambdaInit(1e0);
    optimizer.setAlgorithm(solver);
  }

  // Set Local temporal KeyFrame vertices
  N = vpOptimizableKFs.size();
  for (int i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];

    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(false);
    optimizer.addVertex(VP);

    if (pKFi->bImu) {
      VertexVelocity* VV = new VertexVelocity(pKFi);
      VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
      VV->setFixed(false);
      optimizer.addVertex(VV);
      VertexGyroBias* VG = new VertexGyroBias(pKFi);
      VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
      VG->setFixed(false);
      optimizer.addVertex(VG);
      VertexAccBias* VA = new VertexAccBias(pKFi);
      VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
      VA->setFixed(false);
      optimizer.addVertex(VA);
    }
  }

  // Set Local visual KeyFrame vertices
  for (KeyFrame* const kf : lpOptVisKFs) {
    VertexPose* VP = new VertexPose(kf);
    VP->setId(kf->mnId);
    VP->setFixed(false);
    optimizer.addVertex(VP);
  }

  // Set Fixed KeyFrame vertices
  for (KeyFrame* const kf : lFixedKeyFrames) {
    VertexPose* VP = new VertexPose(kf);
    VP->setId(kf->mnId);
    VP->setFixed(true);
    optimizer.addVertex(VP);

    if (kf->bImu) { // This should be done only for keyframe just before temporal window
      VertexVelocity* VV = new VertexVelocity(kf);
      VV->setId(maxKFid + 3 * (kf->mnId) + 1);
      VV->setFixed(true);
      optimizer.addVertex(VV);
      VertexGyroBias* VG = new VertexGyroBias(kf);
      VG->setId(maxKFid + 3 * (kf->mnId) + 2);
      VG->setFixed(true);
      optimizer.addVertex(VG);
      VertexAccBias* VA = new VertexAccBias(kf);
      VA->setId(maxKFid + 3 * (kf->mnId) + 3);
      VA->setFixed(true);
      optimizer.addVertex(VA);
    }
  }

  // Create intertial constraints
  std::vector<EdgeInertial*> vei(N, (EdgeInertial*)NULL);
  std::vector<EdgeGyroRW*>   vegr(N, (EdgeGyroRW*)NULL);
  std::vector<EdgeAccRW*>    vear(N, (EdgeAccRW*)NULL);

  for (int i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];

    if (!pKFi->mPrevKF) {
      // No inertial link to previous frame.
      continue;
    }
    if (pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated) {
      pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
      g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VV1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1);
      g2o::HyperGraph::Vertex* VG1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2);
      g2o::HyperGraph::Vertex* VA1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3);
      g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
      g2o::HyperGraph::Vertex* VV2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1);
      g2o::HyperGraph::Vertex* VG2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2);
      g2o::HyperGraph::Vertex* VA2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3);

      if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2) {
        continue;
      }

      vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);

      vei[i]->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
      vei[i]->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
      vei[i]->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
      vei[i]->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
      vei[i]->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
      vei[i]->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

      if (i == N - 1 || bRecInit) {
        // All inertial residuals are included without robust cost function, but not that one
        // linking the last optimizable keyframe inside of the local window and the first fixed
        // keyframe out. The information matrix for this measurement is also downweighted. This is
        // done to avoid accumulating error due to fixing variables.
        g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
        vei[i]->setRobustKernel(rki);
        if (i == N - 1) {
          vei[i]->setInformation(vei[i]->information() * 1e-2);
        }
        rki->setDelta(std::sqrt(16.92));
      }
      optimizer.addEdge(vei[i]);

      vegr[i] = new EdgeGyroRW();
      vegr[i]->setVertex(0, VG1);
      vegr[i]->setVertex(1, VG2);
      Eigen::Matrix3d InfoG
        = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
      vegr[i]->setInformation(InfoG);
      optimizer.addEdge(vegr[i]);

      vear[i] = new EdgeAccRW();
      vear[i]->setVertex(0, VA1);
      vear[i]->setVertex(1, VA2);
      Eigen::Matrix3d InfoA
        = pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
      vear[i]->setInformation(InfoA);

      optimizer.addEdge(vear[i]);
    } else {
      // (error) faile to build inertial edge.
    }
  }

  // Set MapPoint vertices
  const int nExpectedSize = (N + lFixedKeyFrames.size()) * lLocalMapPoints.size();

  // Mono
  std::vector<EdgeMono*> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);

  std::vector<KeyFrame*> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);

  std::vector<MapPoint*> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  // Stereo
  std::vector<EdgeStereo*> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);

  std::vector<KeyFrame*> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);

  std::vector<MapPoint*> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  const float thHuberMono   = std::sqrt(5.991);
  const float chi2Mono2     = 5.991;
  const float thHuberStereo = std::sqrt(7.815);
  const float chi2Stereo2   = 7.815;

  const unsigned long iniMPid = maxKFid * 5;

  std::map<int, int> mVisEdges;
  for (int i = 0; i < N; i++) {
    KeyFrame* pKFi        = vpOptimizableKFs[i];
    mVisEdges[pKFi->mnId] = 0;
  }
  for (KeyFrame* const kf : lFixedKeyFrames) {
    mVisEdges[kf->mnId] = 0;
  }

  for (MapPoint* const mp : lLocalMapPoints) {
    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(mp->GetWorldPos().cast<double>());

    unsigned long id = mp->mnId + iniMPid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);
    auto observations = mp->GetObservations();

    // Create visual constraints
    for (const auto& [kf, indices] : observations) {
      if (kf->mnBALocalForKF != pKF->mnId && kf->mnBAFixedForKF != pKF->mnId) {
        continue;
      }

      auto [left_id, right_id] = indices;

      if (!kf->isBad() && kf->GetMap() == pCurrentMap) {
        cv::KeyPoint kpUn;

        if (left_id != -1 && kf->mvuRight[left_id] < 0) { // Monocular left observation
          mVisEdges[kf->mnId]++;

          kpUn = kf->mvKeysUn[left_id];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMono* e = new EdgeMono(0);

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(kf->mnId)));
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = kf->mpCamera->uncertainty2(obs);

          const float& invSigma2 = kf->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);
          vpEdgesMono.push_back(e);
          vpEdgeKFMono.push_back(kf);
          vpMapPointEdgeMono.push_back(mp);
        } else if (left_id != -1) { // Stereo observation
          kpUn = kf->mvKeysUn[left_id];
          mVisEdges[kf->mnId]++;

          const float                 kp_ur = kf->mvuRight[left_id];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          EdgeStereo* e = new EdgeStereo(0);

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(kf->mnId)));
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = kf->mpCamera->uncertainty2(obs.head(2));

          const float& invSigma2 = kf->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);
          vpEdgesStereo.push_back(e);
          vpEdgeKFStereo.push_back(kf);
          vpMapPointEdgeStereo.push_back(mp);
        }

        if (kf->mpCamera2) { // Monocular right observation
          if (right_id != -1) {
            right_id -= kf->NLeft;
            mVisEdges[kf->mnId]++;

            Eigen::Matrix<double, 2, 1> obs;
            cv::KeyPoint                kp = kf->mvKeysRight[right_id];
            obs << kp.pt.x, kp.pt.y;

            EdgeMono* e = new EdgeMono(1);

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
            e->setVertex(
              1,
              dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(kf->mnId))
            );
            e->setMeasurement(obs);

            // Add here uncerteinty
            const float unc2 = kf->mpCamera->uncertainty2(obs);

            const float& invSigma2 = kf->mvInvLevelSigma2[kpUn.octave] / unc2;
            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuberMono);

            optimizer.addEdge(e);
            vpEdgesMono.push_back(e);
            vpEdgeKFMono.push_back(kf);
            vpMapPointEdgeMono.push_back(mp);
          }
        }
      }
    }
  }

  for (const int val : mVisEdges | std::views::values) {
    assert(val >= 3);
  }

  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  float err = optimizer.activeRobustChi2();
  optimizer.optimize(opt_it); // Originally to 2
  float err_end = optimizer.activeRobustChi2();
  if (pbStopFlag) {
    optimizer.setForceStopFlag(pbStopFlag);
  }

  std::vector<std::pair<KeyFrame*, MapPoint*>> vToErase;
  vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

  // Check inlier observations
  // Mono
  for (std::size_t i = 0; i < vpEdgesMono.size(); i++) {
    EdgeMono* e      = vpEdgesMono[i];
    MapPoint* pMP    = vpMapPointEdgeMono[i];
    bool      bClose = pMP->mTrackDepth < 10.f;

    if (pMP->isBad()) {
      continue;
    }

    if ((e->chi2() > chi2Mono2 && !bClose) || (e->chi2() > 1.5f * chi2Mono2 && bClose) || !e->isDepthPositive()) {
      KeyFrame* pKFi = vpEdgeKFMono[i];
      vToErase.push_back(std::make_pair(pKFi, pMP));
    }
  }

  // Stereo
  for (std::size_t i = 0; i < vpEdgesStereo.size(); i++) {
    EdgeStereo* e   = vpEdgesStereo[i];
    MapPoint*   pMP = vpMapPointEdgeStereo[i];

    if (pMP->isBad()) {
      continue;
    }

    if (e->chi2() > chi2Stereo2) {
      KeyFrame* pKFi = vpEdgeKFStereo[i];
      vToErase.push_back(std::make_pair(pKFi, pMP));
    }
  }

  // Get Map Mutex and erase outliers
  std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);

  // TODO: Some convergence problems have been detected here
  if ((2 * err < err_end || isnan(err) || isnan(err_end)) && !bLarge) { // bGN)
    // Local-inertial bundle adjustment failed.
    return;
  }

  if (!vToErase.empty()) {
    for (const auto& [kf, mp] : vToErase) {
      kf->EraseMapPointMatch(mp);
      mp->EraseObservation(kf);
    }
  }

  for (KeyFrame* const kf : lFixedKeyFrames) {
    kf->mnBAFixedForKF = 0;
  }

  // Recover optimized data
  // Local temporal Keyframes
  N = vpOptimizableKFs.size();
  for (int i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];

    VertexPose*  VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
    Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
    pKFi->SetPose(Tcw);
    pKFi->mnBALocalForKF = 0;

    if (pKFi->bImu) {
      VertexVelocity* VV
        = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
      pKFi->SetVelocity(VV->estimate().cast<float>());
      VertexGyroBias* VG
        = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
      VertexAccBias* VA
        = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
      Vector6d b;
      b << VG->estimate(), VA->estimate();
      pKFi->SetNewBias(IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
    }
  }

  // Local visual KeyFrame
  for (KeyFrame* const kf : lpOptVisKFs) {
    VertexPose*  VP = static_cast<VertexPose*>(optimizer.vertex(kf->mnId));
    Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
    kf->SetPose(Tcw);
    kf->mnBALocalForKF = 0;
  }

  // Points
  for (MapPoint* const mp : lLocalMapPoints) {
    g2o::VertexSBAPointXYZ* vPoint
      = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(mp->mnId + iniMPid + 1));
    mp->SetWorldPos(vPoint->estimate().cast<float>());
    mp->UpdateNormalAndDepth();
  }

  pMap->IncreaseChangeIndex();
}

Eigen::MatrixXd Optimizer::Marginalize(const Eigen::MatrixXd& H, const int& start, const int& end) {
  // Goal
  // a  | ab | ac       a*  | 0 | ac*
  // ba | b  | bc  -->  0   | 0 | 0
  // ca | cb | c        ca* | 0 | c*

  // Size of block before block to marginalize
  const int a = start;
  // Size of block to marginalize
  const int b = end - start + 1;
  // Size of block after block to marginalize
  const int c = H.cols() - (end + 1);

  // Reorder as follows:
  // a  | ab | ac       a  | ac | ab
  // ba | b  | bc  -->  ca | c  | cb
  // ca | cb | c        ba | bc | b

  Eigen::MatrixXd Hn = Eigen::MatrixXd::Zero(H.rows(), H.cols());
  if (a > 0) {
    Hn.block(0, 0, a, a)     = H.block(0, 0, a, a);
    Hn.block(0, a + c, a, b) = H.block(0, a, a, b);
    Hn.block(a + c, 0, b, a) = H.block(a, 0, b, a);
  }
  if (a > 0 && c > 0) {
    Hn.block(0, a, a, c) = H.block(0, a + b, a, c);
    Hn.block(a, 0, c, a) = H.block(a + b, 0, c, a);
  }
  if (c > 0) {
    Hn.block(a, a, c, c)     = H.block(a + b, a + b, c, c);
    Hn.block(a, a + c, c, b) = H.block(a + b, a, c, b);
    Hn.block(a + c, a, b, c) = H.block(a, a + b, b, c);
  }
  Hn.block(a + c, a + c, b, b) = H.block(a, a, b, b);

  // Perform marginalization (Schur complement)
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
    Hn.block(a + c, a + c, b, b),
    Eigen::ComputeThinU | Eigen::ComputeThinV
  );
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singularValues_inv = svd.singularValues();
  for (int i = 0; i < b; ++i) {
    if (singularValues_inv(i) > 1e-6) {
      singularValues_inv(i) = 1.0 / singularValues_inv(i);
    } else {
      singularValues_inv(i) = 0;
    }
  }
  Eigen::MatrixXd invHb
    = svd.matrixV() * singularValues_inv.asDiagonal() * svd.matrixU().transpose();
  Hn.block(0, 0, a + c, a + c)
    = Hn.block(0, 0, a + c, a + c)
    - Hn.block(0, a + c, a + c, b) * invHb * Hn.block(a + c, 0, b, a + c);
  Hn.block(a + c, a + c, b, b) = Eigen::MatrixXd::Zero(b, b);
  Hn.block(0, a + c, a + c, b) = Eigen::MatrixXd::Zero(a + c, b);
  Hn.block(a + c, 0, b, a + c) = Eigen::MatrixXd::Zero(b, a + c);

  // Inverse reorder
  // a*  | ac* | 0       a*  | 0 | ac*
  // ca* | c*  | 0  -->  0   | 0 | 0
  // 0   | 0   | 0       ca* | 0 | c*
  Eigen::MatrixXd res = Eigen::MatrixXd::Zero(H.rows(), H.cols());
  if (a > 0) {
    res.block(0, 0, a, a) = Hn.block(0, 0, a, a);
    res.block(0, a, a, b) = Hn.block(0, a + c, a, b);
    res.block(a, 0, b, a) = Hn.block(a + c, 0, b, a);
  }
  if (a > 0 && c > 0) {
    res.block(0, a + b, a, c) = Hn.block(0, a, a, c);
    res.block(a + b, 0, c, a) = Hn.block(a, 0, c, a);
  }
  if (c > 0) {
    res.block(a + b, a + b, c, c) = Hn.block(a, a, c, c);
    res.block(a + b, a, c, b)     = Hn.block(a, a + c, c, b);
    res.block(a, a + b, b, c)     = Hn.block(a + c, a, b, c);
  }

  res.block(a, a, b, b) = Hn.block(a + c, a + c, b, b);

  return res;
}

void Optimizer::InertialOptimization(
  Map*             pMap,
  Eigen::Matrix3d& Rwg,
  double&          scale,
  Eigen::Vector3d& bg,
  Eigen::Vector3d& ba,
  bool             bMono,
  Eigen::MatrixXd& covInertial,
  bool             bFixedVel,
  bool             bGauss,
  float            priorG,
  float            priorA
) {
  int                          its     = 200;
  long unsigned int            maxKFid = pMap->GetMaxKFid();
  const std::vector<KeyFrame*> vpKFs   = pMap->GetAllKeyFrames();

  // Setup optimizer
  g2o::SparseOptimizer                 optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  if (priorG != 0.f) {
    solver->setUserLambdaInit(1e3);
  }

  optimizer.setAlgorithm(solver);

  // Set KeyFrame vertices (fixed poses and optimizable velocities)
  for (KeyFrame* const kf : vpKFs) {
    if (kf->mnId > maxKFid) {
      continue;
    }
    VertexPose* VP = new VertexPose(kf);
    VP->setId(kf->mnId);
    VP->setFixed(true);
    optimizer.addVertex(VP);

    VertexVelocity* VV = new VertexVelocity(kf);
    VV->setId(maxKFid + (kf->mnId) + 1);
    if (bFixedVel) {
      VV->setFixed(true);
    } else {
      VV->setFixed(false);
    }

    optimizer.addVertex(VV);
  }

  // Biases
  VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
  VG->setId(maxKFid * 2 + 2);
  if (bFixedVel) {
    VG->setFixed(true);
  } else {
    VG->setFixed(false);
  }
  optimizer.addVertex(VG);
  VertexAccBias* VA = new VertexAccBias(vpKFs.front());
  VA->setId(maxKFid * 2 + 3);
  if (bFixedVel) {
    VA->setFixed(true);
  } else {
    VA->setFixed(false);
  }

  optimizer.addVertex(VA);
  // prior acc bias
  Eigen::Vector3f bprior;
  bprior.setZero();

  EdgePriorAcc* epa = new EdgePriorAcc(bprior);
  epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
  double infoPriorA = priorA;
  epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
  optimizer.addEdge(epa);
  EdgePriorGyro* epg = new EdgePriorGyro(bprior);
  epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
  double infoPriorG = priorG;
  epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
  optimizer.addEdge(epg);

  // Gravity and scale
  VertexGDir* VGDir = new VertexGDir(Rwg);
  VGDir->setId(maxKFid * 2 + 4);
  VGDir->setFixed(false);
  optimizer.addVertex(VGDir);
  VertexScale* VS = new VertexScale(scale);
  VS->setId(maxKFid * 2 + 5);
  VS->setFixed(!bMono); // Fixed for stereo case
  optimizer.addVertex(VS);

  // Graph edges
  // IMU links with gravity and scale
  std::vector<EdgeInertialGS*> vpei;
  vpei.reserve(vpKFs.size());
  std::vector<std::pair<KeyFrame*, KeyFrame*>> vppUsedKF;
  vppUsedKF.reserve(vpKFs.size());

  for (KeyFrame* const kf : vpKFs) {
    if (kf->mPrevKF && kf->mnId <= maxKFid) {
      if (kf->isBad() || kf->mPrevKF->mnId > maxKFid) {
        continue;
      }
      if (!kf->mpImuPreintegrated) {
        // No preintegrated measurement.
      }

      kf->mpImuPreintegrated->SetNewBias(kf->mPrevKF->GetImuBias());
      g2o::HyperGraph::Vertex* VP1   = optimizer.vertex(kf->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VV1   = optimizer.vertex(maxKFid + (kf->mPrevKF->mnId) + 1);
      g2o::HyperGraph::Vertex* VP2   = optimizer.vertex(kf->mnId);
      g2o::HyperGraph::Vertex* VV2   = optimizer.vertex(maxKFid + (kf->mnId) + 1);
      g2o::HyperGraph::Vertex* VG    = optimizer.vertex(maxKFid * 2 + 2);
      g2o::HyperGraph::Vertex* VA    = optimizer.vertex(maxKFid * 2 + 3);
      g2o::HyperGraph::Vertex* VGDir = optimizer.vertex(maxKFid * 2 + 4);
      g2o::HyperGraph::Vertex* VS    = optimizer.vertex(maxKFid * 2 + 5);
      if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS) {
        continue;
      }
      EdgeInertialGS* ei = new EdgeInertialGS(kf->mpImuPreintegrated);
      ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
      ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
      ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
      ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
      ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
      ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));
      ei->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VGDir));
      ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VS));

      vpei.push_back(ei);

      vppUsedKF.push_back(std::make_pair(kf->mPrevKF, kf));
      optimizer.addEdge(ei);
    }
  }

  // Compute error for different scales
  std::set<g2o::HyperGraph::Edge*> setEdges = optimizer.edges();

  optimizer.setVerbose(false);
  optimizer.initializeOptimization();
  optimizer.optimize(its);

  scale = VS->estimate();

  // Recover optimized data
  // Biases
  VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid * 2 + 2));
  VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid * 2 + 3));
  Vector6d vb;
  vb << VG->estimate(), VA->estimate();
  bg << VG->estimate();
  ba << VA->estimate();
  scale = VS->estimate();

  IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);
  Rwg = VGDir->estimate().Rwg;

  // Keyframes velocities and biases
  for (KeyFrame* const kf : vpKFs) {
    if (kf->mnId > maxKFid) {
      continue;
    }

    VertexVelocity* VV = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid + (kf->mnId) + 1));
    Eigen::Vector3d Vw = VV->estimate(); // Velocity is scaled after
    kf->SetVelocity(Vw.cast<float>());

    if ((kf->GetGyroBias() - bg.cast<float>()).norm() > 0.01) {
      kf->SetNewBias(b);
      if (kf->mpImuPreintegrated) {
        kf->mpImuPreintegrated->Reintegrate();
      }
    } else {
      kf->SetNewBias(b);
    }
  }
}

void Optimizer::InertialOptimization(
  Map* pMap, Eigen::Vector3d& bg, Eigen::Vector3d& ba, float priorG, float priorA
) {
  int                          its     = 200; // Check number of iterations
  long unsigned int            maxKFid = pMap->GetMaxKFid();
  const std::vector<KeyFrame*> vpKFs   = pMap->GetAllKeyFrames();

  // Setup optimizer
  g2o::SparseOptimizer                 optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  solver->setUserLambdaInit(1e3);

  optimizer.setAlgorithm(solver);

  // Set KeyFrame vertices (fixed poses and optimizable velocities)
  for (KeyFrame* const kf : vpKFs) {
    if (kf->mnId > maxKFid) {
      continue;
    }
    VertexPose* VP = new VertexPose(kf);
    VP->setId(kf->mnId);
    VP->setFixed(true);
    optimizer.addVertex(VP);

    VertexVelocity* VV = new VertexVelocity(kf);
    VV->setId(maxKFid + kf->mnId + 1);
    VV->setFixed(false);
    optimizer.addVertex(VV);
  }

  // Biases
  VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
  VG->setId(maxKFid * 2 + 2);
  VG->setFixed(false);
  optimizer.addVertex(VG);

  VertexAccBias* VA = new VertexAccBias(vpKFs.front());
  VA->setId(maxKFid * 2 + 3);
  VA->setFixed(false);

  optimizer.addVertex(VA);
  // prior acc bias
  Eigen::Vector3f bprior;
  bprior.setZero();

  EdgePriorAcc* epa = new EdgePriorAcc(bprior);
  epa->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
  double infoPriorA = priorA;
  epa->setInformation(infoPriorA * Eigen::Matrix3d::Identity());
  optimizer.addEdge(epa);
  EdgePriorGyro* epg = new EdgePriorGyro(bprior);
  epg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
  double infoPriorG = priorG;
  epg->setInformation(infoPriorG * Eigen::Matrix3d::Identity());
  optimizer.addEdge(epg);

  // Gravity and scale
  VertexGDir* VGDir = new VertexGDir(Eigen::Matrix3d::Identity());
  VGDir->setId(maxKFid * 2 + 4);
  VGDir->setFixed(true);
  optimizer.addVertex(VGDir);
  VertexScale* VS = new VertexScale(1.0);
  VS->setId(maxKFid * 2 + 5);
  VS->setFixed(true); // Fixed since scale is obtained from already well initialized map
  optimizer.addVertex(VS);

  // Graph edges
  // IMU links with gravity and scale
  std::vector<EdgeInertialGS*> vpei;
  vpei.reserve(vpKFs.size());
  std::vector<std::pair<KeyFrame*, KeyFrame*>> vppUsedKF;
  vppUsedKF.reserve(vpKFs.size());

  for (KeyFrame* const kf : vpKFs) {
    if (kf->mPrevKF && kf->mnId <= maxKFid) {
      if (kf->isBad() || kf->mPrevKF->mnId > maxKFid) {
        continue;
      }

      kf->mpImuPreintegrated->SetNewBias(kf->mPrevKF->GetImuBias());
      g2o::HyperGraph::Vertex* VP1   = optimizer.vertex(kf->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VV1   = optimizer.vertex(maxKFid + (kf->mPrevKF->mnId) + 1);
      g2o::HyperGraph::Vertex* VP2   = optimizer.vertex(kf->mnId);
      g2o::HyperGraph::Vertex* VV2   = optimizer.vertex(maxKFid + (kf->mnId) + 1);
      g2o::HyperGraph::Vertex* VG    = optimizer.vertex(maxKFid * 2 + 2);
      g2o::HyperGraph::Vertex* VA    = optimizer.vertex(maxKFid * 2 + 3);
      g2o::HyperGraph::Vertex* VGDir = optimizer.vertex(maxKFid * 2 + 4);
      g2o::HyperGraph::Vertex* VS    = optimizer.vertex(maxKFid * 2 + 5);
      if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS) {
        continue;
      }
      EdgeInertialGS* ei = new EdgeInertialGS(kf->mpImuPreintegrated);
      ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
      ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
      ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
      ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
      ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
      ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));
      ei->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VGDir));
      ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VS));

      vpei.push_back(ei);

      vppUsedKF.push_back(std::make_pair(kf->mPrevKF, kf));
      optimizer.addEdge(ei);
    }
  }

  // Compute error for different scales
  optimizer.setVerbose(false);
  optimizer.initializeOptimization();
  optimizer.optimize(its);

  // Recover optimized data
  // Biases
  VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid * 2 + 2));
  VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid * 2 + 3));
  Vector6d vb;
  vb << VG->estimate(), VA->estimate();
  bg << VG->estimate();
  ba << VA->estimate();

  IMU::Bias b(vb[3], vb[4], vb[5], vb[0], vb[1], vb[2]);

  // Keyframes velocities and biases
  for (KeyFrame* const kf : vpKFs) {
    if (kf->mnId > maxKFid) {
      continue;
    }

    VertexVelocity* VV = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid + (kf->mnId) + 1));
    Eigen::Vector3d Vw = VV->estimate();
    kf->SetVelocity(Vw.cast<float>());

    if ((kf->GetGyroBias() - bg.cast<float>()).norm() > 0.01) {
      kf->SetNewBias(b);
      if (kf->mpImuPreintegrated) {
        kf->mpImuPreintegrated->Reintegrate();
      }
    } else {
      kf->SetNewBias(b);
    }
  }
}

void Optimizer::InertialOptimization(Map* pMap, Eigen::Matrix3d& Rwg, double& scale) {
  int                          its     = 10;
  long unsigned int            maxKFid = pMap->GetMaxKFid();
  const std::vector<KeyFrame*> vpKFs   = pMap->GetAllKeyFrames();

  // Setup optimizer
  g2o::SparseOptimizer                 optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmGaussNewton* solver
    = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
  optimizer.setAlgorithm(solver);

  // Set KeyFrame vertices (all variables are fixed)
  for (KeyFrame* const kf : vpKFs) {
    if (kf->mnId > maxKFid) {
      continue;
    }
    VertexPose* VP = new VertexPose(kf);
    VP->setId(kf->mnId);
    VP->setFixed(true);
    optimizer.addVertex(VP);

    VertexVelocity* VV = new VertexVelocity(kf);
    VV->setId(maxKFid + 1 + (kf->mnId));
    VV->setFixed(true);
    optimizer.addVertex(VV);

    // Vertex of fixed biases
    VertexGyroBias* VG = new VertexGyroBias(vpKFs.front());
    VG->setId(2 * (maxKFid + 1) + (kf->mnId));
    VG->setFixed(true);
    optimizer.addVertex(VG);
    VertexAccBias* VA = new VertexAccBias(vpKFs.front());
    VA->setId(3 * (maxKFid + 1) + (kf->mnId));
    VA->setFixed(true);
    optimizer.addVertex(VA);
  }

  // Gravity and scale
  VertexGDir* VGDir = new VertexGDir(Rwg);
  VGDir->setId(4 * (maxKFid + 1));
  VGDir->setFixed(false);
  optimizer.addVertex(VGDir);
  VertexScale* VS = new VertexScale(scale);
  VS->setId(4 * (maxKFid + 1) + 1);
  VS->setFixed(false);
  optimizer.addVertex(VS);

  // Graph edges
  int count_edges = 0;
  for (KeyFrame* const kf : vpKFs) {
    if (kf->mPrevKF && kf->mnId <= maxKFid) {
      if (kf->isBad() || kf->mPrevKF->mnId > maxKFid) {
        continue;
      }

      g2o::HyperGraph::Vertex* VP1   = optimizer.vertex(kf->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VV1   = optimizer.vertex((maxKFid + 1) + kf->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VP2   = optimizer.vertex(kf->mnId);
      g2o::HyperGraph::Vertex* VV2   = optimizer.vertex((maxKFid + 1) + kf->mnId);
      g2o::HyperGraph::Vertex* VG    = optimizer.vertex(2 * (maxKFid + 1) + kf->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VA    = optimizer.vertex(3 * (maxKFid + 1) + kf->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VGDir = optimizer.vertex(4 * (maxKFid + 1));
      g2o::HyperGraph::Vertex* VS    = optimizer.vertex(4 * (maxKFid + 1) + 1);
      if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS) {
        continue;
      }
      count_edges++;
      EdgeInertialGS* ei = new EdgeInertialGS(kf->mpImuPreintegrated);
      ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
      ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
      ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG));
      ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA));
      ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
      ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));
      ei->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VGDir));
      ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VS));
      g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
      ei->setRobustKernel(rk);
      rk->setDelta(1.f);
      optimizer.addEdge(ei);
    }
  }

  // Compute error for different scales
  optimizer.setVerbose(false);
  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  float err = optimizer.activeRobustChi2();
  optimizer.optimize(its);
  optimizer.computeActiveErrors();
  float err_end = optimizer.activeRobustChi2();
  // Recover optimized data
  scale = VS->estimate();
  Rwg   = VGDir->estimate().Rwg;
}

void Optimizer::LocalBundleAdjustment(
  KeyFrame*              pMainKF,
  std::vector<KeyFrame*> vpAdjustKF,
  std::vector<KeyFrame*> vpFixedKF,
  bool*                  pbStopFlag
) {
  bool bShowImages = false;

  std::vector<MapPoint*> vpMPs;

  g2o::SparseOptimizer                    optimizer;
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver;

  linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  optimizer.setVerbose(false);

  if (pbStopFlag) {
    optimizer.setForceStopFlag(pbStopFlag);
  }

  long unsigned int   maxKFid = 0;
  std::set<KeyFrame*> spKeyFrameBA;

  Map* pCurrentMap = pMainKF->GetMap();

  // Set fixed KeyFrame vertices
  int numInsertedPoints = 0;
  for (KeyFrame* const kf : vpFixedKF) {
    if (kf->isBad() || kf->GetMap() != pCurrentMap) {
      // Skip due to bad key frame or non-existing key frame in current map.
      continue;
    }

    kf->mnBALocalForMerge = pMainKF->mnId;

    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float>    Tcw  = kf->GetPose();
    vSE3->setEstimate(
      g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>())
    );
    vSE3->setId(kf->mnId);
    vSE3->setFixed(true);
    optimizer.addVertex(vSE3);
    if (kf->mnId > maxKFid) {
      maxKFid = kf->mnId;
    }

    std::set<MapPoint*> spViewMPs = kf->GetMapPoints();
    for (MapPoint* const mp : spViewMPs) {
      if (mp) {
        if (!mp->isBad() && mp->GetMap() == pCurrentMap) {
          if (mp->mnBALocalForMerge != pMainKF->mnId) {
            vpMPs.push_back(mp);
            mp->mnBALocalForMerge = pMainKF->mnId;
            numInsertedPoints++;
          }
        }
      }
    }

    spKeyFrameBA.insert(kf);
  }

  // Set non fixed Keyframe vertices
  std::set<KeyFrame*> spAdjustKF(vpAdjustKF.begin(), vpAdjustKF.end());
  numInsertedPoints = 0;
  for (KeyFrame* const kf : vpAdjustKF) {
    if (kf->isBad() || kf->GetMap() != pCurrentMap) {
      continue;
    }

    kf->mnBALocalForMerge = pMainKF->mnId;

    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float>    Tcw  = kf->GetPose();
    vSE3->setEstimate(
      g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>())
    );
    vSE3->setId(kf->mnId);
    optimizer.addVertex(vSE3);
    if (kf->mnId > maxKFid) {
      maxKFid = kf->mnId;
    }

    std::set<MapPoint*> spViewMPs = kf->GetMapPoints();
    for (MapPoint* const mp : spViewMPs) {
      if (mp) {
        if (!mp->isBad() && mp->GetMap() == pCurrentMap) {
          if (mp->mnBALocalForMerge != pMainKF->mnId) {
            vpMPs.push_back(mp);
            mp->mnBALocalForMerge = pMainKF->mnId;
            numInsertedPoints++;
          }
        }
      }
    }

    spKeyFrameBA.insert(kf);
  }

  const int nExpectedSize = (vpAdjustKF.size() + vpFixedKF.size()) * vpMPs.size();

  std::vector<ORB_SLAM3::EdgeSE3ProjectXYZ*> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);

  std::vector<KeyFrame*> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);

  std::vector<MapPoint*> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  std::vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);

  std::vector<KeyFrame*> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);

  std::vector<MapPoint*> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  const float thHuber2D = std::sqrt(5.99);
  const float thHuber3D = std::sqrt(7.815);

  // Set MapPoint vertices
  std::map<KeyFrame*, int> mpObsKFs;
  std::map<KeyFrame*, int> mpObsFinalKFs;
  std::map<MapPoint*, int> mpObsMPs;
  for (MapPoint* const mp : vpMPs) {
    if (mp->isBad()) {
      continue;
    }

    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(mp->GetWorldPos().cast<double>());
    const int id = mp->mnId + maxKFid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    auto observations = mp->GetObservations();
    int  nEdges       = 0;
    // SET EDGES
    for (const auto& [kf, indices] : observations) {
      auto [left_id, _] = indices;

      if (kf->isBad() || kf->mnId > maxKFid || kf->mnBALocalForMerge != pMainKF->mnId || !kf->GetMapPoint(left_id)) {
        continue;
      }

      nEdges++;

      const cv::KeyPoint& kpUn = kf->mvKeysUn[left_id];

      if (kf->mvuRight[left_id] < 0) { // Monocular
        mpObsMPs[mp]++;
        Eigen::Matrix<double, 2, 1> obs;
        obs << kpUn.pt.x, kpUn.pt.y;

        ORB_SLAM3::EdgeSE3ProjectXYZ* e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(kf->mnId)));
        e->setMeasurement(obs);
        const float& invSigma2 = kf->mvInvLevelSigma2[kpUn.octave];
        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(thHuber2D);

        e->pCamera = kf->mpCamera;

        optimizer.addEdge(e);

        vpEdgesMono.push_back(e);
        vpEdgeKFMono.push_back(kf);
        vpMapPointEdgeMono.push_back(mp);

        mpObsKFs[kf]++;
      } else { // RGBD or Stereo
        mpObsMPs[mp] += 2;
        Eigen::Matrix<double, 3, 1> obs;
        const float                 kp_ur = kf->mvuRight[left_id];
        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

        g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(kf->mnId)));
        e->setMeasurement(obs);
        const float&    invSigma2 = kf->mvInvLevelSigma2[kpUn.octave];
        Eigen::Matrix3d Info      = Eigen::Matrix3d::Identity() * invSigma2;
        e->setInformation(Info);

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(thHuber3D);

        e->fx = kf->fx;
        e->fy = kf->fy;
        e->cx = kf->cx;
        e->cy = kf->cy;
        e->bf = kf->mbf;

        optimizer.addEdge(e);

        vpEdgesStereo.push_back(e);
        vpEdgeKFStereo.push_back(kf);
        vpMapPointEdgeStereo.push_back(mp);

        mpObsKFs[kf]++;
      }
    }
  }

  if (pbStopFlag) {
    if (*pbStopFlag) {
      return;
    }
  }

  optimizer.initializeOptimization();
  optimizer.optimize(5);

  bool bDoMore = true;

  if (pbStopFlag) {
    if (*pbStopFlag) {
      bDoMore = false;
    }
  }

  std::map<unsigned long int, int> mWrongObsKF;
  if (bDoMore) {
    // Check inlier observations
    int badMonoMP = 0, badStereoMP = 0;
    for (std::size_t i = 0; i < vpEdgesMono.size(); i++) {
      ORB_SLAM3::EdgeSE3ProjectXYZ* e   = vpEdgesMono[i];
      MapPoint*                     pMP = vpMapPointEdgeMono[i];

      if (pMP->isBad()) {
        continue;
      }

      if (e->chi2() > 5.991 || !e->isDepthPositive()) {
        e->setLevel(1);
        badMonoMP++;
      }
      e->setRobustKernel(0);
    }

    for (std::size_t i = 0; i < vpEdgesStereo.size(); i++) {
      g2o::EdgeStereoSE3ProjectXYZ* e   = vpEdgesStereo[i];
      MapPoint*                     pMP = vpMapPointEdgeStereo[i];

      if (pMP->isBad()) {
        continue;
      }

      if (e->chi2() > 7.815 || !e->isDepthPositive()) {
        e->setLevel(1);
        badStereoMP++;
      }

      e->setRobustKernel(0);
    }

    optimizer.initializeOptimization(0);
    optimizer.optimize(10);
  }

  std::vector<std::pair<KeyFrame*, MapPoint*>> vToErase;
  vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());
  std::set<MapPoint*> spErasedMPs;
  std::set<KeyFrame*> spErasedKFs;

  // Check inlier observations
  int badMonoMP = 0, badStereoMP = 0;
  for (std::size_t i = 0; i < vpEdgesMono.size(); i++) {
    ORB_SLAM3::EdgeSE3ProjectXYZ* e   = vpEdgesMono[i];
    MapPoint*                     pMP = vpMapPointEdgeMono[i];

    if (pMP->isBad()) {
      continue;
    }

    if (e->chi2() > 5.991 || !e->isDepthPositive()) {
      KeyFrame* pKFi = vpEdgeKFMono[i];
      vToErase.push_back(std::make_pair(pKFi, pMP));
      mWrongObsKF[pKFi->mnId]++;
      badMonoMP++;

      spErasedMPs.insert(pMP);
      spErasedKFs.insert(pKFi);
    }
  }

  for (std::size_t i = 0; i < vpEdgesStereo.size(); i++) {
    g2o::EdgeStereoSE3ProjectXYZ* e   = vpEdgesStereo[i];
    MapPoint*                     pMP = vpMapPointEdgeStereo[i];

    if (pMP->isBad()) {
      continue;
    }

    if (e->chi2() > 7.815 || !e->isDepthPositive()) {
      KeyFrame* pKFi = vpEdgeKFStereo[i];
      vToErase.push_back(std::make_pair(pKFi, pMP));
      mWrongObsKF[pKFi->mnId]++;
      badStereoMP++;

      spErasedMPs.insert(pMP);
      spErasedKFs.insert(pKFi);
    }
  }

  // Get Map Mutex
  std::unique_lock<std::mutex> lock(pMainKF->GetMap()->mMutexMapUpdate);

  if (!vToErase.empty()) {
    for (const auto& [kf, mp] : vToErase) {
      kf->EraseMapPointMatch(mp);
      mp->EraseObservation(kf);
    }
  }
  for (MapPoint* const mp : vpMPs) {
    if (mp->isBad()) {
      continue;
    }

    auto observations = mp->GetObservations();
    for (const auto& [pKF, indices] : observations) {
      auto [left_id, _] = indices;
      if (pKF->isBad() || pKF->mnId > maxKFid || pKF->mnBALocalForKF != pMainKF->mnId || !pKF->GetMapPoint(left_id)) {
        continue;
      }

      if (pKF->mvuRight[left_id] < 0) { // Monocular
        mpObsFinalKFs[pKF]++;
      } else { // RGBD or Stereo
        mpObsFinalKFs[pKF]++;
      }
    }
  }

  // Recover optimized data
  // Keyframes
  for (KeyFrame* const kf : vpAdjustKF) {
    if (kf->isBad()) {
      continue;
    }

    g2o::VertexSE3Expmap* vSE3    = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(kf->mnId));
    g2o::SE3Quat          SE3quat = vSE3->estimate();
    Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(), SE3quat.translation().cast<float>());

    int                    numMonoBadPoints = 0, numMonoOptPoints = 0;
    int                    numStereoBadPoints = 0, numStereoOptPoints = 0;
    std::vector<MapPoint*> vpMonoMPsOpt, vpStereoMPsOpt;
    std::vector<MapPoint*> vpMonoMPsBad, vpStereoMPsBad;

    for (std::size_t i = 0; i < vpEdgesMono.size(); i++) {
      ORB_SLAM3::EdgeSE3ProjectXYZ* e       = vpEdgesMono[i];
      MapPoint*                     pMP     = vpMapPointEdgeMono[i];
      KeyFrame*                     pKFedge = vpEdgeKFMono[i];

      if (kf != pKFedge) {
        continue;
      }

      if (pMP->isBad()) {
        continue;
      }

      if (e->chi2() > 5.991 || !e->isDepthPositive()) {
        numMonoBadPoints++;
        vpMonoMPsBad.push_back(pMP);

      } else {
        numMonoOptPoints++;
        vpMonoMPsOpt.push_back(pMP);
      }
    }

    for (std::size_t i = 0; i < vpEdgesStereo.size(); i++) {
      g2o::EdgeStereoSE3ProjectXYZ* e       = vpEdgesStereo[i];
      MapPoint*                     pMP     = vpMapPointEdgeStereo[i];
      KeyFrame*                     pKFedge = vpEdgeKFMono[i];

      if (kf != pKFedge) {
        continue;
      }

      if (pMP->isBad()) {
        continue;
      }

      if (e->chi2() > 7.815 || !e->isDepthPositive()) {
        numStereoBadPoints++;
        vpStereoMPsBad.push_back(pMP);
      } else {
        numStereoOptPoints++;
        vpStereoMPsOpt.push_back(pMP);
      }
    }

    kf->SetPose(Tiw);
  }

  // Points
  for (MapPoint* const mp : vpMPs) {
    if (mp->isBad()) {
      continue;
    }

    g2o::VertexSBAPointXYZ* vPoint
      = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(mp->mnId + maxKFid + 1));
    mp->SetWorldPos(vPoint->estimate().cast<float>());
    mp->UpdateNormalAndDepth();
  }
}

void Optimizer::MergeInertialBA(
  KeyFrame*                     pCurrKF,
  KeyFrame*                     pMergeKF,
  bool*                         pbStopFlag,
  Map*                          pMap,
  LoopClosing::KeyFrameAndPose& corrPoses
) {
  const int           Nd      = 6;
  const unsigned long maxKFid = pCurrKF->mnId;

  std::vector<KeyFrame*> vpOptimizableKFs;
  vpOptimizableKFs.reserve(2 * Nd);

  // For cov KFS, inertial parameters are not optimized
  const int              maxCovKF = 30;
  std::vector<KeyFrame*> vpOptimizableCovKFs;
  vpOptimizableCovKFs.reserve(maxCovKF);

  // Add sliding window for current KF
  vpOptimizableKFs.push_back(pCurrKF);
  pCurrKF->mnBALocalForKF = pCurrKF->mnId;
  for (int i = 1; i < Nd; i++) {
    if (vpOptimizableKFs.back()->mPrevKF) {
      vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
      vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
    } else {
      break;
    }
  }

  std::list<KeyFrame*> lFixedKeyFrames;
  if (vpOptimizableKFs.back()->mPrevKF) {
    vpOptimizableCovKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
    vpOptimizableKFs.back()->mPrevKF->mnBALocalForKF = pCurrKF->mnId;
  } else {
    vpOptimizableCovKFs.push_back(vpOptimizableKFs.back());
    vpOptimizableKFs.pop_back();
  }

  // Add temporal neighbours to merge KF (previous and next KFs)
  vpOptimizableKFs.push_back(pMergeKF);
  pMergeKF->mnBALocalForKF = pCurrKF->mnId;

  // Previous KFs
  for (int i = 1; i < (Nd / 2); i++) {
    if (vpOptimizableKFs.back()->mPrevKF) {
      vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
      vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
    } else {
      break;
    }
  }

  // We fix just once the old map
  if (vpOptimizableKFs.back()->mPrevKF) {
    lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
    vpOptimizableKFs.back()->mPrevKF->mnBAFixedForKF = pCurrKF->mnId;
  } else {
    vpOptimizableKFs.back()->mnBALocalForKF = 0;
    vpOptimizableKFs.back()->mnBAFixedForKF = pCurrKF->mnId;
    lFixedKeyFrames.push_back(vpOptimizableKFs.back());
    vpOptimizableKFs.pop_back();
  }

  // Next KFs
  if (pMergeKF->mNextKF) {
    vpOptimizableKFs.push_back(pMergeKF->mNextKF);
    vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
  }

  while (vpOptimizableKFs.size() < (2 * Nd)) {
    if (vpOptimizableKFs.back()->mNextKF) {
      vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mNextKF);
      vpOptimizableKFs.back()->mnBALocalForKF = pCurrKF->mnId;
    } else {
      break;
    }
  }

  int N = vpOptimizableKFs.size();

  // Optimizable points seen by optimizable keyframes
  std::list<MapPoint*>     lLocalMapPoints;
  std::map<MapPoint*, int> mLocalObs;
  for (int i = 0; i < N; i++) {
    std::vector<MapPoint*> vpMPs = vpOptimizableKFs[i]->GetMapPointMatches();
    for (MapPoint* const mp : vpMPs) {
      // Using mnBALocalForKF we avoid redundance here, one MP can not be added several times to
      // lLocalMapPoints
      if (mp) {
        if (!mp->isBad()) {
          if (mp->mnBALocalForKF != pCurrKF->mnId) {
            mLocalObs[mp] = 1;
            lLocalMapPoints.push_back(mp);
            mp->mnBALocalForKF = pCurrKF->mnId;
          } else {
            mLocalObs[mp]++;
          }
        }
      }
    }
  }

  std::vector<std::pair<MapPoint*, int>> pairs;
  pairs.reserve(mLocalObs.size());
  for (const auto& obs : mLocalObs) {
    pairs.push_back(obs);
  }
  std::sort(pairs.begin(), pairs.end(), sortByVal);

  // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
  int i = 0;
  for (auto lit = pairs.begin(); lit != pairs.end(); lit++, i++) {
    auto observations = lit->first->GetObservations();
    if (i >= maxCovKF) {
      break;
    }
    for (KeyFrame* const kf : observations | std::views::keys) {
      // If optimizable or already included...
      if (kf->mnBALocalForKF != pCurrKF->mnId && kf->mnBAFixedForKF != pCurrKF->mnId) {
        kf->mnBALocalForKF = pCurrKF->mnId;
        if (!kf->isBad()) {
          vpOptimizableCovKFs.push_back(kf);
          break;
        }
      }
    }
  }

  g2o::SparseOptimizer                 optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;
  linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  solver->setUserLambdaInit(1e3);

  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  // Set Local KeyFrame vertices
  N = vpOptimizableKFs.size();
  for (int i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];

    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(false);
    optimizer.addVertex(VP);

    if (pKFi->bImu) {
      VertexVelocity* VV = new VertexVelocity(pKFi);
      VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
      VV->setFixed(false);
      optimizer.addVertex(VV);
      VertexGyroBias* VG = new VertexGyroBias(pKFi);
      VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
      VG->setFixed(false);
      optimizer.addVertex(VG);
      VertexAccBias* VA = new VertexAccBias(pKFi);
      VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
      VA->setFixed(false);
      optimizer.addVertex(VA);
    }
  }

  // Set Local cov keyframes vertices
  int Ncov = vpOptimizableCovKFs.size();
  for (int i = 0; i < Ncov; i++) {
    KeyFrame* pKFi = vpOptimizableCovKFs[i];

    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(false);
    optimizer.addVertex(VP);

    if (pKFi->bImu) {
      VertexVelocity* VV = new VertexVelocity(pKFi);
      VV->setId(maxKFid + 3 * (pKFi->mnId) + 1);
      VV->setFixed(false);
      optimizer.addVertex(VV);
      VertexGyroBias* VG = new VertexGyroBias(pKFi);
      VG->setId(maxKFid + 3 * (pKFi->mnId) + 2);
      VG->setFixed(false);
      optimizer.addVertex(VG);
      VertexAccBias* VA = new VertexAccBias(pKFi);
      VA->setId(maxKFid + 3 * (pKFi->mnId) + 3);
      VA->setFixed(false);
      optimizer.addVertex(VA);
    }
  }

  // Set Fixed KeyFrame vertices
  for (KeyFrame* const kf : lFixedKeyFrames) {
    VertexPose* VP = new VertexPose(kf);
    VP->setId(kf->mnId);
    VP->setFixed(true);
    optimizer.addVertex(VP);

    if (kf->bImu) {
      VertexVelocity* VV = new VertexVelocity(kf);
      VV->setId(maxKFid + 3 * (kf->mnId) + 1);
      VV->setFixed(true);
      optimizer.addVertex(VV);
      VertexGyroBias* VG = new VertexGyroBias(kf);
      VG->setId(maxKFid + 3 * (kf->mnId) + 2);
      VG->setFixed(true);
      optimizer.addVertex(VG);
      VertexAccBias* VA = new VertexAccBias(kf);
      VA->setId(maxKFid + 3 * (kf->mnId) + 3);
      VA->setFixed(true);
      optimizer.addVertex(VA);
    }
  }

  // Create intertial constraints
  std::vector<EdgeInertial*> vei(N, (EdgeInertial*)NULL);
  std::vector<EdgeGyroRW*>   vegr(N, (EdgeGyroRW*)NULL);
  std::vector<EdgeAccRW*>    vear(N, (EdgeAccRW*)NULL);
  for (int i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];

    if (!pKFi->mPrevKF) {
      // No inertial link to previous frame.
      continue;
    }
    if (pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated) {
      pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
      g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
      g2o::HyperGraph::Vertex* VV1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 1);
      g2o::HyperGraph::Vertex* VG1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 2);
      g2o::HyperGraph::Vertex* VA1 = optimizer.vertex(maxKFid + 3 * (pKFi->mPrevKF->mnId) + 3);
      g2o::HyperGraph::Vertex* VP2 = optimizer.vertex(pKFi->mnId);
      g2o::HyperGraph::Vertex* VV2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1);
      g2o::HyperGraph::Vertex* VG2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2);
      g2o::HyperGraph::Vertex* VA2 = optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3);

      if (!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2) {
        continue;
      }

      vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);

      vei[i]->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
      vei[i]->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
      vei[i]->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
      vei[i]->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
      vei[i]->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
      vei[i]->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

      // TODO Uncomment
      g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
      vei[i]->setRobustKernel(rki);
      rki->setDelta(std::sqrt(16.92));
      optimizer.addEdge(vei[i]);

      vegr[i] = new EdgeGyroRW();
      vegr[i]->setVertex(0, VG1);
      vegr[i]->setVertex(1, VG2);
      Eigen::Matrix3d InfoG
        = pKFi->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
      vegr[i]->setInformation(InfoG);
      optimizer.addEdge(vegr[i]);

      vear[i] = new EdgeAccRW();
      vear[i]->setVertex(0, VA1);
      vear[i]->setVertex(1, VA2);
      Eigen::Matrix3d InfoA
        = pKFi->mpImuPreintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
      vear[i]->setInformation(InfoA);
      optimizer.addEdge(vear[i]);
    } else {
      // (error) Failed to build inertial edge.
    }
  }

  // Set MapPoint vertices
  const int nExpectedSize = (N + Ncov + lFixedKeyFrames.size()) * lLocalMapPoints.size();

  // Mono
  std::vector<EdgeMono*> vpEdgesMono;
  vpEdgesMono.reserve(nExpectedSize);

  std::vector<KeyFrame*> vpEdgeKFMono;
  vpEdgeKFMono.reserve(nExpectedSize);

  std::vector<MapPoint*> vpMapPointEdgeMono;
  vpMapPointEdgeMono.reserve(nExpectedSize);

  // Stereo
  std::vector<EdgeStereo*> vpEdgesStereo;
  vpEdgesStereo.reserve(nExpectedSize);

  std::vector<KeyFrame*> vpEdgeKFStereo;
  vpEdgeKFStereo.reserve(nExpectedSize);

  std::vector<MapPoint*> vpMapPointEdgeStereo;
  vpMapPointEdgeStereo.reserve(nExpectedSize);

  const float thHuberMono   = std::sqrt(5.991);
  const float chi2Mono2     = 5.991;
  const float thHuberStereo = std::sqrt(7.815);
  const float chi2Stereo2   = 7.815;

  const unsigned long iniMPid = maxKFid * 5;

  for (MapPoint* const mp : lLocalMapPoints) {
    if (!mp) {
      continue;
    }

    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
    vPoint->setEstimate(mp->GetWorldPos().cast<double>());

    unsigned long id = mp->mnId + iniMPid + 1;
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);

    auto observations = mp->GetObservations();

    // Create visual constraints
    for (const auto& [kf, indices] : observations) {
      auto [left_id, _] = indices;

      if (!kf) {
        continue;
      }

      if ((kf->mnBALocalForKF != pCurrKF->mnId) && (kf->mnBAFixedForKF != pCurrKF->mnId)) {
        continue;
      }

      if (kf->mnId > maxKFid) {
        continue;
      }

      if (optimizer.vertex(id) == NULL || optimizer.vertex(kf->mnId) == NULL) {
        continue;
      }

      if (!kf->isBad()) {
        const cv::KeyPoint& kpUn = kf->mvKeysUn[left_id];

        if (kf->mvuRight[left_id] < 0) { // Monocular observation
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMono* e = new EdgeMono();
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(kf->mnId)));
          e->setMeasurement(obs);
          const float& invSigma2 = kf->mvInvLevelSigma2[kpUn.octave];
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);
          optimizer.addEdge(e);
          vpEdgesMono.push_back(e);
          vpEdgeKFMono.push_back(kf);
          vpMapPointEdgeMono.push_back(mp);
        } else { // stereo observation
          const float                 kp_ur = kf->mvuRight[left_id];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          EdgeStereo* e = new EdgeStereo();

          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(kf->mnId)));
          e->setMeasurement(obs);
          const float& invSigma2 = kf->mvInvLevelSigma2[kpUn.octave];
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);
          vpEdgesStereo.push_back(e);
          vpEdgeKFStereo.push_back(kf);
          vpMapPointEdgeStereo.push_back(mp);
        }
      }
    }
  }

  if (pbStopFlag) {
    optimizer.setForceStopFlag(pbStopFlag);
  }

  if (pbStopFlag) {
    if (*pbStopFlag) {
      return;
    }
  }

  optimizer.initializeOptimization();
  optimizer.optimize(8);

  std::vector<std::pair<KeyFrame*, MapPoint*>> vToErase;
  vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

  // Check inlier observations
  // Mono
  for (std::size_t i = 0; i < vpEdgesMono.size(); i++) {
    EdgeMono* e   = vpEdgesMono[i];
    MapPoint* pMP = vpMapPointEdgeMono[i];

    if (pMP->isBad()) {
      continue;
    }

    if (e->chi2() > chi2Mono2) {
      KeyFrame* pKFi = vpEdgeKFMono[i];
      vToErase.push_back(std::make_pair(pKFi, pMP));
    }
  }

  // Stereo
  for (std::size_t i = 0; i < vpEdgesStereo.size(); i++) {
    EdgeStereo* e   = vpEdgesStereo[i];
    MapPoint*   pMP = vpMapPointEdgeStereo[i];

    if (pMP->isBad()) {
      continue;
    }

    if (e->chi2() > chi2Stereo2) {
      KeyFrame* pKFi = vpEdgeKFStereo[i];
      vToErase.push_back(std::make_pair(pKFi, pMP));
    }
  }

  // Get Map Mutex and erase outliers
  std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);
  if (!vToErase.empty()) {
    for (const auto& [pKFi, pMPi] : vToErase) {
      pKFi->EraseMapPointMatch(pMPi);
      pMPi->EraseObservation(pKFi);
    }
  }

  // Recover optimized data
  // Keyframes
  for (int i = 0; i < N; i++) {
    KeyFrame* pKFi = vpOptimizableKFs[i];

    VertexPose*  VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
    Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
    pKFi->SetPose(Tcw);

    Sophus::SE3d Tiw = pKFi->GetPose().cast<double>();
    g2o::Sim3    g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
    corrPoses[pKFi] = g2oSiw;

    if (pKFi->bImu) {
      VertexVelocity* VV
        = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
      pKFi->SetVelocity(VV->estimate().cast<float>());
      VertexGyroBias* VG
        = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
      VertexAccBias* VA
        = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
      Vector6d b;
      b << VG->estimate(), VA->estimate();
      pKFi->SetNewBias(IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
    }
  }

  for (int i = 0; i < Ncov; i++) {
    KeyFrame* pKFi = vpOptimizableCovKFs[i];

    VertexPose*  VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
    Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
    pKFi->SetPose(Tcw);

    Sophus::SE3d Tiw = pKFi->GetPose().cast<double>();
    g2o::Sim3    g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
    corrPoses[pKFi] = g2oSiw;

    if (pKFi->bImu) {
      VertexVelocity* VV
        = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 1));
      pKFi->SetVelocity(VV->estimate().cast<float>());
      VertexGyroBias* VG
        = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 2));
      VertexAccBias* VA
        = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid + 3 * (pKFi->mnId) + 3));
      Vector6d b;
      b << VG->estimate(), VA->estimate();
      pKFi->SetNewBias(IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]));
    }
  }

  // Points
  for (MapPoint* const mp : lLocalMapPoints) {
    g2o::VertexSBAPointXYZ* vPoint
      = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(mp->mnId + iniMPid + 1));
    mp->SetWorldPos(vPoint->estimate().cast<float>());
    mp->UpdateNormalAndDepth();
  }

  pMap->IncreaseChangeIndex();
}

int Optimizer::PoseInertialOptimizationLastKeyFrame(Frame* pFrame, bool bRecInit) {
  g2o::SparseOptimizer                 optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmGaussNewton* solver
    = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
  optimizer.setVerbose(false);
  optimizer.setAlgorithm(solver);

  int nInitialMonoCorrespondences   = 0;
  int nInitialStereoCorrespondences = 0;
  int nInitialCorrespondences       = 0;

  // Set Frame vertex
  VertexPose* VP = new VertexPose(pFrame);
  VP->setId(0);
  VP->setFixed(false);
  optimizer.addVertex(VP);
  VertexVelocity* VV = new VertexVelocity(pFrame);
  VV->setId(1);
  VV->setFixed(false);
  optimizer.addVertex(VV);
  VertexGyroBias* VG = new VertexGyroBias(pFrame);
  VG->setId(2);
  VG->setFixed(false);
  optimizer.addVertex(VG);
  VertexAccBias* VA = new VertexAccBias(pFrame);
  VA->setId(3);
  VA->setFixed(false);
  optimizer.addVertex(VA);

  // Set MapPoint vertices
  const int  N      = pFrame->N;
  const int  Nleft  = pFrame->Nleft;
  const bool bRight = (Nleft != -1);

  std::vector<EdgeMonoOnlyPose*>   vpEdgesMono;
  std::vector<EdgeStereoOnlyPose*> vpEdgesStereo;
  std::vector<std::size_t>         vnIndexEdgeMono;
  std::vector<std::size_t>         vnIndexEdgeStereo;
  vpEdgesMono.reserve(N);
  vpEdgesStereo.reserve(N);
  vnIndexEdgeMono.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  const float thHuberMono   = std::sqrt(5.991);
  const float thHuberStereo = std::sqrt(7.815);

  {
    std::unique_lock<std::mutex> lock(MapPoint::mGlobalMutex);

    for (int i = 0; i < N; i++) {
      MapPoint* pMP = pFrame->mvpMapPoints[i];
      if (pMP) {
        cv::KeyPoint kpUn;

        if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft) { // Left monocular observation
          if (i < Nleft) {                                       // pair left-right
            kpUn = pFrame->mvKeys[i];
          } else {
            kpUn = pFrame->mvKeysUn[i];
          }

          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 0);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        } else if (!bRight) { // Stereo observation
          nInitialStereoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn                              = pFrame->mvKeysUn[i];
          const float                 kp_ur = pFrame->mvuRight[i];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          EdgeStereoOnlyPose* e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

          const float& invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);

          vpEdgesStereo.push_back(e);
          vnIndexEdgeStereo.push_back(i);
        }

        if (bRight && i >= Nleft) { // Right monocular observation
          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn = pFrame->mvKeysRight[i - Nleft];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 1);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
      }
    }
  }
  nInitialCorrespondences = nInitialMonoCorrespondences + nInitialStereoCorrespondences;

  KeyFrame*   pKF = pFrame->mpLastKeyFrame;
  VertexPose* VPk = new VertexPose(pKF);
  VPk->setId(4);
  VPk->setFixed(true);
  optimizer.addVertex(VPk);
  VertexVelocity* VVk = new VertexVelocity(pKF);
  VVk->setId(5);
  VVk->setFixed(true);
  optimizer.addVertex(VVk);
  VertexGyroBias* VGk = new VertexGyroBias(pKF);
  VGk->setId(6);
  VGk->setFixed(true);
  optimizer.addVertex(VGk);
  VertexAccBias* VAk = new VertexAccBias(pKF);
  VAk->setId(7);
  VAk->setFixed(true);
  optimizer.addVertex(VAk);

  EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegrated);

  ei->setVertex(0, VPk);
  ei->setVertex(1, VVk);
  ei->setVertex(2, VGk);
  ei->setVertex(3, VAk);
  ei->setVertex(4, VP);
  ei->setVertex(5, VV);
  optimizer.addEdge(ei);

  EdgeGyroRW* egr = new EdgeGyroRW();
  egr->setVertex(0, VGk);
  egr->setVertex(1, VG);
  Eigen::Matrix3d InfoG = pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
  egr->setInformation(InfoG);
  optimizer.addEdge(egr);

  EdgeAccRW* ear = new EdgeAccRW();
  ear->setVertex(0, VAk);
  ear->setVertex(1, VA);
  Eigen::Matrix3d InfoA
    = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
  ear->setInformation(InfoA);
  optimizer.addEdge(ear);

  // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
  // At the next optimization, outliers are not included, but at the end they can be classified as
  // inliers again.
  float chi2Mono[4]   = {12, 7.5, 5.991, 5.991};
  float chi2Stereo[4] = {15.6, 9.8, 7.815, 7.815};

  int its[4] = {10, 10, 10, 10};

  int nBad           = 0;
  int nBadMono       = 0;
  int nBadStereo     = 0;
  int nInliersMono   = 0;
  int nInliersStereo = 0;
  int nInliers       = 0;
  for (std::size_t it = 0; it < 4; it++) {
    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);

    nBad            = 0;
    nBadMono        = 0;
    nBadStereo      = 0;
    nInliers        = 0;
    nInliersMono    = 0;
    nInliersStereo  = 0;
    float chi2close = 1.5 * chi2Mono[it];

    // For monocular observations
    for (std::size_t i = 0; i < vpEdgesMono.size(); i++) {
      EdgeMonoOnlyPose* e = vpEdgesMono[i];

      const std::size_t idx = vnIndexEdgeMono[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2   = e->chi2();
      bool        bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

      if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) || !e->isDepthPositive()) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBadMono++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersMono++;
      }

      if (it == 2) {
        e->setRobustKernel(0);
      }
    }

    // For stereo observations
    for (std::size_t i = 0; i < vpEdgesStereo.size(); i++) {
      EdgeStereoOnlyPose* e = vpEdgesStereo[i];

      const std::size_t idx = vnIndexEdgeStereo[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if (chi2 > chi2Stereo[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1); // not included in next optimization
        nBadStereo++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersStereo++;
      }

      if (it == 2) {
        e->setRobustKernel(0);
      }
    }

    nInliers = nInliersMono + nInliersStereo;
    nBad     = nBadMono + nBadStereo;

    if (optimizer.edges().size() < 10) {
      break;
    }
  }

  // If not too much tracks, recover not too bad points
  if ((nInliers < 30) && !bRecInit) {
    nBad                              = 0;
    const float         chi2MonoOut   = 18.f;
    const float         chi2StereoOut = 24.f;
    EdgeMonoOnlyPose*   e1;
    EdgeStereoOnlyPose* e2;
    for (std::size_t i = 0; i < vnIndexEdgeMono.size(); i++) {
      const std::size_t idx = vnIndexEdgeMono[i];
      e1                    = vpEdgesMono[i];
      e1->computeError();
      if (e1->chi2() < chi2MonoOut) {
        pFrame->mvbOutlier[idx] = false;
      } else {
        nBad++;
      }
    }
    for (std::size_t i = 0; i < vnIndexEdgeStereo.size(); i++) {
      const std::size_t idx = vnIndexEdgeStereo[i];
      e2                    = vpEdgesStereo[i];
      e2->computeError();
      if (e2->chi2() < chi2StereoOut) {
        pFrame->mvbOutlier[idx] = false;
      } else {
        nBad++;
      }
    }
  }

  // Recover optimized pose, velocity and biases
  pFrame->SetImuPoseVelocity(
    VP->estimate().Rwb.cast<float>(),
    VP->estimate().twb.cast<float>(),
    VV->estimate().cast<float>()
  );
  Vector6d b;
  b << VG->estimate(), VA->estimate();
  pFrame->mImuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);

  // Recover Hessian, marginalize keyFframe states and generate new prior for frame
  Eigen::Matrix<double, 15, 15> H;
  H.setZero();

  H.block<9, 9>(0, 0)   += ei->GetHessian2();
  H.block<3, 3>(9, 9)   += egr->GetHessian2();
  H.block<3, 3>(12, 12) += ear->GetHessian2();

  int tot_in = 0, tot_out = 0;
  for (std::size_t i = 0; i < vpEdgesMono.size(); i++) {
    EdgeMonoOnlyPose* e = vpEdgesMono[i];

    const std::size_t idx = vnIndexEdgeMono[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(0, 0) += e->GetHessian();
      tot_in++;
    } else {
      tot_out++;
    }
  }

  for (std::size_t i = 0; i < vpEdgesStereo.size(); i++) {
    EdgeStereoOnlyPose* e = vpEdgesStereo[i];

    const std::size_t idx = vnIndexEdgeStereo[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(0, 0) += e->GetHessian();
      tot_in++;
    } else {
      tot_out++;
    }
  }

  pFrame->mpcpi = new ConstraintPoseImu(
    VP->estimate().Rwb,
    VP->estimate().twb,
    VV->estimate(),
    VG->estimate(),
    VA->estimate(),
    H
  );

  return nInitialCorrespondences - nBad;
}

int Optimizer::PoseInertialOptimizationLastFrame(Frame* pFrame, bool bRecInit) {
  g2o::SparseOptimizer                 optimizer;
  g2o::BlockSolverX::LinearSolverType* linearSolver;

  linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmGaussNewton* solver
    = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  int nInitialMonoCorrespondences   = 0;
  int nInitialStereoCorrespondences = 0;
  int nInitialCorrespondences       = 0;

  // Set Current Frame vertex
  VertexPose* VP = new VertexPose(pFrame);
  VP->setId(0);
  VP->setFixed(false);
  optimizer.addVertex(VP);
  VertexVelocity* VV = new VertexVelocity(pFrame);
  VV->setId(1);
  VV->setFixed(false);
  optimizer.addVertex(VV);
  VertexGyroBias* VG = new VertexGyroBias(pFrame);
  VG->setId(2);
  VG->setFixed(false);
  optimizer.addVertex(VG);
  VertexAccBias* VA = new VertexAccBias(pFrame);
  VA->setId(3);
  VA->setFixed(false);
  optimizer.addVertex(VA);

  // Set MapPoint vertices
  const int  N      = pFrame->N;
  const int  Nleft  = pFrame->Nleft;
  const bool bRight = (Nleft != -1);

  std::vector<EdgeMonoOnlyPose*>   vpEdgesMono;
  std::vector<EdgeStereoOnlyPose*> vpEdgesStereo;
  std::vector<std::size_t>         vnIndexEdgeMono;
  std::vector<std::size_t>         vnIndexEdgeStereo;
  vpEdgesMono.reserve(N);
  vpEdgesStereo.reserve(N);
  vnIndexEdgeMono.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  const float thHuberMono   = std::sqrt(5.991);
  const float thHuberStereo = std::sqrt(7.815);

  {
    std::unique_lock<std::mutex> lock(MapPoint::mGlobalMutex);

    for (int i = 0; i < N; i++) {
      MapPoint* pMP = pFrame->mvpMapPoints[i];
      if (pMP) {
        cv::KeyPoint kpUn;
        // Left monocular observation
        if ((!bRight && pFrame->mvuRight[i] < 0) || i < Nleft) {
          if (i < Nleft) { // pair left-right
            kpUn = pFrame->mvKeys[i];
          } else {
            kpUn = pFrame->mvKeysUn[i];
          }

          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 0);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
        // Stereo observation
        else if (!bRight) {
          nInitialStereoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn                              = pFrame->mvKeysUn[i];
          const float                 kp_ur = pFrame->mvuRight[i];
          Eigen::Matrix<double, 3, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

          EdgeStereoOnlyPose* e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

          const float& invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix3d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberStereo);

          optimizer.addEdge(e);

          vpEdgesStereo.push_back(e);
          vnIndexEdgeStereo.push_back(i);
        }

        // Right monocular observation
        if (bRight && i >= Nleft) {
          nInitialMonoCorrespondences++;
          pFrame->mvbOutlier[i] = false;

          kpUn = pFrame->mvKeysRight[i - Nleft];
          Eigen::Matrix<double, 2, 1> obs;
          obs << kpUn.pt.x, kpUn.pt.y;

          EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(), 1);

          e->setVertex(0, VP);
          e->setMeasurement(obs);

          // Add here uncerteinty
          const float unc2 = pFrame->mpCamera->uncertainty2(obs);

          const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave] / unc2;
          e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

          g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
          e->setRobustKernel(rk);
          rk->setDelta(thHuberMono);

          optimizer.addEdge(e);

          vpEdgesMono.push_back(e);
          vnIndexEdgeMono.push_back(i);
        }
      }
    }
  }

  nInitialCorrespondences = nInitialMonoCorrespondences + nInitialStereoCorrespondences;

  // Set Previous Frame Vertex
  Frame* pFp = pFrame->mpPrevFrame;

  VertexPose* VPk = new VertexPose(pFp);
  VPk->setId(4);
  VPk->setFixed(false);
  optimizer.addVertex(VPk);
  VertexVelocity* VVk = new VertexVelocity(pFp);
  VVk->setId(5);
  VVk->setFixed(false);
  optimizer.addVertex(VVk);
  VertexGyroBias* VGk = new VertexGyroBias(pFp);
  VGk->setId(6);
  VGk->setFixed(false);
  optimizer.addVertex(VGk);
  VertexAccBias* VAk = new VertexAccBias(pFp);
  VAk->setId(7);
  VAk->setFixed(false);
  optimizer.addVertex(VAk);

  EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegratedFrame);

  ei->setVertex(0, VPk);
  ei->setVertex(1, VVk);
  ei->setVertex(2, VGk);
  ei->setVertex(3, VAk);
  ei->setVertex(4, VP);
  ei->setVertex(5, VV);
  optimizer.addEdge(ei);

  EdgeGyroRW* egr = new EdgeGyroRW();
  egr->setVertex(0, VGk);
  egr->setVertex(1, VG);
  Eigen::Matrix3d InfoG = pFrame->mpImuPreintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
  egr->setInformation(InfoG);
  optimizer.addEdge(egr);

  EdgeAccRW* ear = new EdgeAccRW();
  ear->setVertex(0, VAk);
  ear->setVertex(1, VA);
  Eigen::Matrix3d InfoA
    = pFrame->mpImuPreintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
  ear->setInformation(InfoA);
  optimizer.addEdge(ear);

  if (!pFp->mpcpi) {
    // Constraint to previous frame non existing.
  }

  EdgePriorPoseImu* ep = new EdgePriorPoseImu(pFp->mpcpi);

  ep->setVertex(0, VPk);
  ep->setVertex(1, VVk);
  ep->setVertex(2, VGk);
  ep->setVertex(3, VAk);
  g2o::RobustKernelHuber* rkp = new g2o::RobustKernelHuber;
  ep->setRobustKernel(rkp);
  rkp->setDelta(5);
  optimizer.addEdge(ep);

  // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
  // At the next optimization, outliers are not included, but at the end they can be classified as
  // inliers again.
  const float chi2Mono[4]   = {5.991, 5.991, 5.991, 5.991};
  const float chi2Stereo[4] = {15.6f, 9.8f, 7.815f, 7.815f};
  const int   its[4]        = {10, 10, 10, 10};

  int nBad           = 0;
  int nBadMono       = 0;
  int nBadStereo     = 0;
  int nInliersMono   = 0;
  int nInliersStereo = 0;
  int nInliers       = 0;
  for (std::size_t it = 0; it < 4; it++) {
    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);

    nBad            = 0;
    nBadMono        = 0;
    nBadStereo      = 0;
    nInliers        = 0;
    nInliersMono    = 0;
    nInliersStereo  = 0;
    float chi2close = 1.5 * chi2Mono[it];

    for (std::size_t i = 0; i < vpEdgesMono.size(); i++) {
      EdgeMonoOnlyPose* e = vpEdgesMono[i];

      const std::size_t idx    = vnIndexEdgeMono[i];
      bool              bClose = pFrame->mvpMapPoints[idx]->mTrackDepth < 10.f;

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if ((chi2 > chi2Mono[it] && !bClose) || (bClose && chi2 > chi2close) || !e->isDepthPositive()) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBadMono++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersMono++;
      }

      if (it == 2) {
        e->setRobustKernel(0);
      }
    }

    for (std::size_t i = 0; i < vpEdgesStereo.size(); i++) {
      EdgeStereoOnlyPose* e = vpEdgesStereo[i];

      const std::size_t idx = vnIndexEdgeStereo[i];

      if (pFrame->mvbOutlier[idx]) {
        e->computeError();
      }

      const float chi2 = e->chi2();

      if (chi2 > chi2Stereo[it]) {
        pFrame->mvbOutlier[idx] = true;
        e->setLevel(1);
        nBadStereo++;
      } else {
        pFrame->mvbOutlier[idx] = false;
        e->setLevel(0);
        nInliersStereo++;
      }

      if (it == 2) {
        e->setRobustKernel(0);
      }
    }

    nInliers = nInliersMono + nInliersStereo;
    nBad     = nBadMono + nBadStereo;

    if (optimizer.edges().size() < 10) {
      break;
    }
  }

  if ((nInliers < 30) && !bRecInit) {
    nBad                              = 0;
    const float         chi2MonoOut   = 18.f;
    const float         chi2StereoOut = 24.f;
    EdgeMonoOnlyPose*   e1;
    EdgeStereoOnlyPose* e2;
    for (std::size_t i = 0; i < vnIndexEdgeMono.size(); i++) {
      const std::size_t idx = vnIndexEdgeMono[i];
      e1                    = vpEdgesMono[i];
      e1->computeError();
      if (e1->chi2() < chi2MonoOut) {
        pFrame->mvbOutlier[idx] = false;
      } else {
        nBad++;
      }
    }
    for (std::size_t i = 0; i < vnIndexEdgeStereo.size(); i++) {
      const std::size_t idx = vnIndexEdgeStereo[i];
      e2                    = vpEdgesStereo[i];
      e2->computeError();
      if (e2->chi2() < chi2StereoOut) {
        pFrame->mvbOutlier[idx] = false;
      } else {
        nBad++;
      }
    }
  }

  nInliers = nInliersMono + nInliersStereo;

  // Recover optimized pose, velocity and biases
  pFrame->SetImuPoseVelocity(
    VP->estimate().Rwb.cast<float>(),
    VP->estimate().twb.cast<float>(),
    VV->estimate().cast<float>()
  );
  Vector6d b;
  b << VG->estimate(), VA->estimate();
  pFrame->mImuBias = IMU::Bias(b[3], b[4], b[5], b[0], b[1], b[2]);

  // Recover Hessian, marginalize previous frame states and generate new prior for frame
  Eigen::Matrix<double, 30, 30> H;
  H.setZero();

  H.block<24, 24>(0, 0) += ei->GetHessian();

  Eigen::Matrix<double, 6, 6> Hgr  = egr->GetHessian();
  H.block<3, 3>(9, 9)             += Hgr.block<3, 3>(0, 0);
  H.block<3, 3>(9, 24)            += Hgr.block<3, 3>(0, 3);
  H.block<3, 3>(24, 9)            += Hgr.block<3, 3>(3, 0);
  H.block<3, 3>(24, 24)           += Hgr.block<3, 3>(3, 3);

  Eigen::Matrix<double, 6, 6> Har  = ear->GetHessian();
  H.block<3, 3>(12, 12)           += Har.block<3, 3>(0, 0);
  H.block<3, 3>(12, 27)           += Har.block<3, 3>(0, 3);
  H.block<3, 3>(27, 12)           += Har.block<3, 3>(3, 0);
  H.block<3, 3>(27, 27)           += Har.block<3, 3>(3, 3);

  H.block<15, 15>(0, 0) += ep->GetHessian();

  int tot_in = 0, tot_out = 0;
  for (std::size_t i = 0; i < vpEdgesMono.size(); i++) {
    EdgeMonoOnlyPose* e = vpEdgesMono[i];

    const std::size_t idx = vnIndexEdgeMono[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(15, 15) += e->GetHessian();
      tot_in++;
    } else {
      tot_out++;
    }
  }

  for (std::size_t i = 0; i < vpEdgesStereo.size(); i++) {
    EdgeStereoOnlyPose* e = vpEdgesStereo[i];

    const std::size_t idx = vnIndexEdgeStereo[i];

    if (!pFrame->mvbOutlier[idx]) {
      H.block<6, 6>(15, 15) += e->GetHessian();
      tot_in++;
    } else {
      tot_out++;
    }
  }

  H = Marginalize(H, 0, 14);

  pFrame->mpcpi = new ConstraintPoseImu(
    VP->estimate().Rwb,
    VP->estimate().twb,
    VV->estimate(),
    VG->estimate(),
    VA->estimate(),
    H.block<15, 15>(15, 15)
  );
  delete pFp->mpcpi;
  pFp->mpcpi = NULL;

  return nInitialCorrespondences - nBad;
}

void Optimizer::OptimizeEssentialGraph4DoF(
  Map*                                            pMap,
  KeyFrame*                                       pLoopKF,
  KeyFrame*                                       pCurKF,
  const LoopClosing::KeyFrameAndPose&             NonCorrectedSim3,
  const LoopClosing::KeyFrameAndPose&             CorrectedSim3,
  const std::map<KeyFrame*, std::set<KeyFrame*>>& LoopConnections
) {
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<4, 4>> BlockSolver_4_4;

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  g2o::BlockSolverX::LinearSolverType* linearSolver
    = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
  g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  optimizer.setAlgorithm(solver);

  const std::vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
  const std::vector<MapPoint*> vpMPs = pMap->GetAllMapPoints();

  const unsigned int nMaxKFid = pMap->GetMaxKFid();

  std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
  std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(nMaxKFid + 1);

  std::vector<VertexPose4DoF*> vpVertices(nMaxKFid + 1);

  const int minFeat = 100;
  // Set KeyFrame vertices
  for (KeyFrame* const kf : vpKFs) {
    if (kf->isBad()) {
      continue;
    }

    VertexPose4DoF* V4DoF;

    const int nIDi = kf->mnId;

    auto it = CorrectedSim3.find(kf);

    if (it != CorrectedSim3.end()) {
      vScw[nIDi]          = it->second;
      const g2o::Sim3 Swc = it->second.inverse();
      Eigen::Matrix3d Rwc = Swc.rotation().toRotationMatrix();
      Eigen::Vector3d twc = Swc.translation();
      V4DoF               = new VertexPose4DoF(Rwc, twc, kf);
    } else {
      Sophus::SE3d Tcw = kf->GetPose().cast<double>();
      g2o::Sim3    Siw(Tcw.unit_quaternion(), Tcw.translation(), 1.0);

      vScw[nIDi] = Siw;
      V4DoF      = new VertexPose4DoF(kf);
    }

    if (kf == pLoopKF) {
      V4DoF->setFixed(true);
    }

    V4DoF->setId(nIDi);
    V4DoF->setMarginalized(false);

    optimizer.addVertex(V4DoF);
    vpVertices[nIDi] = V4DoF;
  }
  std::set<std::pair<long unsigned int, long unsigned int>> sInsertedEdges;

  // Edge used in posegraph has still 6Dof, even if updates of camera poses are just in 4DoF
  Eigen::Matrix<double, 6, 6> matLambda = Eigen::Matrix<double, 6, 6>::Identity();
  matLambda(0, 0)                       = 1e3;
  matLambda(1, 1)                       = 1e3;
  matLambda(0, 0)                       = 1e3;

  // Set Loop edges
  Edge4DoF* e_loop;
  for (const auto& [kf, connections] : LoopConnections) {
    const long unsigned int nIDi = kf->mnId;
    const g2o::Sim3         Siw  = vScw[nIDi];

    for (KeyFrame* const connected_kf : connections) {
      const long unsigned int nIDj = connected_kf->mnId;
      if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) && kf->GetWeight(connected_kf) < minFeat) {
        continue;
      }

      const g2o::Sim3 Sjw = vScw[nIDj];
      const g2o::Sim3 Sij = Siw * Sjw.inverse();
      Eigen::Matrix4d Tij;
      Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
      Tij.block<3, 1>(0, 3) = Sij.translation();
      Tij(3, 3)             = 1.;

      Edge4DoF* e = new Edge4DoF(Tij);
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));

      e->information() = matLambda;
      e_loop           = e;
      optimizer.addEdge(e);

      sInsertedEdges.insert(std::make_pair(std::min(nIDi, nIDj), std::max(nIDi, nIDj)));
    }
  }

  // 1. Set normal edges
  for (KeyFrame* const kf : vpKFs) {
    const int nIDi = kf->mnId;

    g2o::Sim3 Siw;

    // Use noncorrected poses for posegraph edges
    auto iti = NonCorrectedSim3.find(kf);

    if (iti != NonCorrectedSim3.end()) {
      Siw = iti->second;
    } else {
      Siw = vScw[nIDi];
    }

    // 1.1.0 Spanning tree edge
    KeyFrame* pParentKF = static_cast<KeyFrame*>(NULL);
    if (pParentKF) {
      int nIDj = pParentKF->mnId;

      g2o::Sim3 Swj;

      auto itj = NonCorrectedSim3.find(pParentKF);

      if (itj != NonCorrectedSim3.end()) {
        Swj = (itj->second).inverse();
      } else {
        Swj = vScw[nIDj].inverse();
      }

      g2o::Sim3       Sij = Siw * Swj;
      Eigen::Matrix4d Tij;
      Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
      Tij.block<3, 1>(0, 3) = Sij.translation();
      Tij(3, 3)             = 1.;

      Edge4DoF* e = new Edge4DoF(Tij);
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
      e->information() = matLambda;
      optimizer.addEdge(e);
    }

    // 1.1.1 Inertial edges
    KeyFrame* prevKF = kf->mPrevKF;
    if (prevKF) {
      int nIDj = prevKF->mnId;

      g2o::Sim3 Swj;

      auto itj = NonCorrectedSim3.find(prevKF);

      if (itj != NonCorrectedSim3.end()) {
        Swj = (itj->second).inverse();
      } else {
        Swj = vScw[nIDj].inverse();
      }

      g2o::Sim3       Sij = Siw * Swj;
      Eigen::Matrix4d Tij;
      Tij.block<3, 3>(0, 0) = Sij.rotation().toRotationMatrix();
      Tij.block<3, 1>(0, 3) = Sij.translation();
      Tij(3, 3)             = 1.;

      Edge4DoF* e = new Edge4DoF(Tij);
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
      e->information() = matLambda;
      optimizer.addEdge(e);
    }

    // 1.2 Loop edges
    const std::set<KeyFrame*> sLoopEdges = kf->GetLoopEdges();
    for (KeyFrame* const loop_kf : sLoopEdges) {
      if (loop_kf->mnId < kf->mnId) {
        g2o::Sim3 Swl;

        auto itl = NonCorrectedSim3.find(loop_kf);

        if (itl != NonCorrectedSim3.end()) {
          Swl = itl->second.inverse();
        } else {
          Swl = vScw[loop_kf->mnId].inverse();
        }

        g2o::Sim3       Sil = Siw * Swl;
        Eigen::Matrix4d Til;
        Til.block<3, 3>(0, 0) = Sil.rotation().toRotationMatrix();
        Til.block<3, 1>(0, 3) = Sil.translation();
        Til(3, 3)             = 1.;

        Edge4DoF* e = new Edge4DoF(Til);
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
        e->setVertex(
          1,
          dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(loop_kf->mnId))
        );
        e->information() = matLambda;
        optimizer.addEdge(e);
      }
    }

    // 1.3 Covisibility graph edges
    for (KeyFrame* const covisible_kf : kf->GetCovisiblesByWeight(minFeat)) {
      if (covisible_kf && covisible_kf!=pParentKF && covisible_kf!=prevKF && covisible_kf!=kf->mNextKF && !kf->hasChild(covisible_kf) && !sLoopEdges.count(covisible_kf)) {
        if (!covisible_kf->isBad() && covisible_kf->mnId < kf->mnId) {
          if (sInsertedEdges.count(std::make_pair(
                std::min(kf->mnId, covisible_kf->mnId),
                std::max(kf->mnId, covisible_kf->mnId)
              ))) {
            continue;
          }

          g2o::Sim3 Swn;

          auto itn = NonCorrectedSim3.find(covisible_kf);

          if (itn != NonCorrectedSim3.end()) {
            Swn = itn->second.inverse();
          } else {
            Swn = vScw[covisible_kf->mnId].inverse();
          }

          g2o::Sim3       Sin = Siw * Swn;
          Eigen::Matrix4d Tin;
          Tin.block<3, 3>(0, 0) = Sin.rotation().toRotationMatrix();
          Tin.block<3, 1>(0, 3) = Sin.translation();
          Tin(3, 3)             = 1.;
          Edge4DoF* e           = new Edge4DoF(Tin);
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
          e->setVertex(
            1,
            dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(covisible_kf->mnId))
          );
          e->information() = matLambda;
          optimizer.addEdge(e);
        }
      }
    }
  }

  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  optimizer.optimize(20);

  std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);

  // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
  for (KeyFrame* const kf : vpKFs) {
    const int nIDi = kf->mnId;

    VertexPose4DoF* Vi = static_cast<VertexPose4DoF*>(optimizer.vertex(nIDi));
    Eigen::Matrix3d Ri = Vi->estimate().Rcw[0];
    Eigen::Vector3d ti = Vi->estimate().tcw[0];

    g2o::Sim3 CorrectedSiw = g2o::Sim3(Ri, ti, 1.);
    vCorrectedSwc[nIDi]    = CorrectedSiw.inverse();

    Sophus::SE3d Tiw(CorrectedSiw.rotation(), CorrectedSiw.translation());
    kf->SetPose(Tiw.cast<float>());
  }

  // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with
  // optimized pose
  for (MapPoint* const mp : vpMPs) {
    if (mp->isBad()) {
      continue;
    }

    int nIDr;

    KeyFrame* pRefKF = mp->GetReferenceKeyFrame();
    nIDr             = pRefKF->mnId;

    g2o::Sim3 Srw          = vScw[nIDr];
    g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

    Eigen::Matrix<double, 3, 1> eigP3Dw          = mp->GetWorldPos().cast<double>();
    Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));
    mp->SetWorldPos(eigCorrectedP3Dw.cast<float>());

    mp->UpdateNormalAndDepth();
  }
  pMap->IncreaseChangeIndex();
}

} // namespace ORB_SLAM3

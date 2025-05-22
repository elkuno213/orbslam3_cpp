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

#include "KeyFrameDatabase.h"
#include <algorithm>
#include <ranges>
#include <Thirdparty/DBoW2/DBoW2/BowVector.h>
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"

namespace ORB_SLAM3 {

KeyFrameDatabase::KeyFrameDatabase() {
}

KeyFrameDatabase::KeyFrameDatabase(const ORBVocabulary& voc) : mpVoc(&voc) {
  mvInvertedFile.resize(voc.size());
}

void KeyFrameDatabase::add(KeyFrame* pKF) {
  std::unique_lock<std::mutex> lock(mMutex);

  std::ranges::for_each(pKF->mBowVec | std::views::keys, [pKF, this](const unsigned id) {
    mvInvertedFile[id].push_back(pKF);
  });
}

void KeyFrameDatabase::erase(KeyFrame* pKF) {
  std::unique_lock<std::mutex> lock(mMutex);

  // TODO(VuHoi): consider using std::ranges::remove_if to remove all keyframes which is pKF to
  // avoid duplicate bug?

  // Erase elements in the Inverse File for the entry
  for (const unsigned id : pKF->mBowVec | std::views::keys) {
    // List of keyframes that share the word
    std::list<KeyFrame*>& lKFs = mvInvertedFile[id];
    // Remove the keyframe from the list.
    // TODO(VuHoi): how to manage allocated memory of deleted keyframe?
    auto it = std::ranges::find(lKFs, pKF);
    if (it != lKFs.end()) {
      lKFs.erase(it);
    }
  }
}

void KeyFrameDatabase::clear() {
  mvInvertedFile.clear();
  mvInvertedFile.resize(mpVoc->size());
}

void KeyFrameDatabase::clearMap(Map* pMap) {
  std::unique_lock<std::mutex> lock(mMutex);

  // Erase elements in the Inverse File for the entry
  // keyframes = List of keyframes that share the word
  for (auto& keyframes : mvInvertedFile) {
    // Don't delete the keyframe because the class Map clean all the KF when it is destroyed
    std::erase_if(keyframes, [pMap](KeyFrame* const kf) {
      return pMap == kf->GetMap();
    });
  }
}

vector<KeyFrame*> KeyFrameDatabase::DetectLoopCandidates(KeyFrame* pKF, float minScore) {
  std::set<KeyFrame*>  spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
  std::list<KeyFrame*> lKFsSharingWords;

  // Search all keyframes that share a word with current keyframes
  // Discard keyframes connected to the query keyframe
  {
    std::unique_lock<std::mutex> lock(mMutex);

    for (const unsigned word_id : pKF->mBowVec | std::views::keys) {
      for (KeyFrame* const candidate : mvInvertedFile[word_id]) {
        // For consider a loop candidate it a candidate it must be in the same map
        if (candidate->GetMap() == pKF->GetMap()) {
          if (candidate->mnLoopQuery != pKF->mnId) {
            candidate->mnLoopWords = 0;
            if (!spConnectedKeyFrames.count(candidate)) {
              candidate->mnLoopQuery = pKF->mnId;
              lKFsSharingWords.push_back(candidate);
            }
          }
          candidate->mnLoopWords++;
        }
      }
    }
  }

  if (lKFsSharingWords.empty()) {
    return std::vector<KeyFrame*>();
  }

  std::list<std::pair<float, KeyFrame*>> lScoreAndMatch;

  // Only compare against those keyframes that share enough words
  int maxCommonWords = 0;
  for (KeyFrame* const candidate : lKFsSharingWords) {
    if (candidate->mnLoopWords > maxCommonWords) {
      maxCommonWords = candidate->mnLoopWords;
    }
  }

  int minCommonWords = maxCommonWords * 0.8f;

  int nscores = 0;

  // Compute similarity score. Retain the matches whose score is higher than minScore
  for (KeyFrame* const candidate : lKFsSharingWords) {
    if (candidate->mnLoopWords > minCommonWords) {
      nscores++;

      float si = mpVoc->score(pKF->mBowVec, candidate->mBowVec);

      candidate->mLoopScore = si;
      if (si >= minScore) {
        lScoreAndMatch.push_back(std::make_pair(si, candidate));
      }
    }
  }

  if (lScoreAndMatch.empty()) {
    return std::vector<KeyFrame*>();
  }

  std::list<std::pair<float, KeyFrame*>> lAccScoreAndMatch;
  float                                  bestAccScore = minScore;

  // Lets now accumulate score by covisibility
  for (const auto& [score, candidate] : lScoreAndMatch) {
    float     bestScore = score;
    float     accScore  = score;
    KeyFrame* pBestKF   = candidate;
    for (KeyFrame* const neighbor : candidate->GetBestCovisibilityKeyFrames(10)) {
      if (neighbor->mnLoopQuery == pKF->mnId && neighbor->mnLoopWords > minCommonWords) {
        accScore += neighbor->mLoopScore;
        if (neighbor->mLoopScore > bestScore) {
          pBestKF   = neighbor;
          bestScore = neighbor->mLoopScore;
        }
      }
    }

    lAccScoreAndMatch.push_back(std::make_pair(accScore, pBestKF));
    if (accScore > bestAccScore) {
      bestAccScore = accScore;
    }
  }

  // Return all those keyframes with a score higher than 0.75*bestScore
  float minScoreToRetain = 0.75f * bestAccScore;

  std::set<KeyFrame*>    spAlreadyAddedKF;
  std::vector<KeyFrame*> vpLoopCandidates;
  vpLoopCandidates.reserve(lAccScoreAndMatch.size());

  for (const auto& [score, candidate] : lAccScoreAndMatch) {
    if (score > minScoreToRetain) {
      if (!spAlreadyAddedKF.count(candidate)) {
        vpLoopCandidates.push_back(candidate);
        spAlreadyAddedKF.insert(candidate);
      }
    }
  }

  return vpLoopCandidates;
}

void KeyFrameDatabase::DetectCandidates(
  KeyFrame*               pKF,
  float                   minScore,
  std::vector<KeyFrame*>& vpLoopCand,
  std::vector<KeyFrame*>& vpMergeCand
) {
  std::set<KeyFrame*>  spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
  std::list<KeyFrame*> lKFsSharingWordsLoop, lKFsSharingWordsMerge;

  // Search all keyframes that share a word with current keyframes
  // Discard keyframes connected to the query keyframe
  {
    std::unique_lock<std::mutex> lock(mMutex);

    for (const unsigned word_id : pKF->mBowVec | std::views::keys) {
      for (KeyFrame* const candidate : mvInvertedFile[word_id]) {
        // For consider a loop candidate it a candidate it must be in the same map
        if (candidate->GetMap() == pKF->GetMap()) {
          if (candidate->mnLoopQuery != pKF->mnId) {
            candidate->mnLoopWords = 0;
            if (!spConnectedKeyFrames.count(candidate)) {
              candidate->mnLoopQuery = pKF->mnId;
              lKFsSharingWordsLoop.push_back(candidate);
            }
          }
          candidate->mnLoopWords++;
        } else if (!candidate->GetMap()->IsBad()) {
          if (candidate->mnMergeQuery != pKF->mnId) {
            candidate->mnMergeWords = 0;
            if (!spConnectedKeyFrames.count(candidate)) {
              candidate->mnMergeQuery = pKF->mnId;
              lKFsSharingWordsMerge.push_back(candidate);
            }
          }
          candidate->mnMergeWords++;
        }
      }
    }
  }

  if (lKFsSharingWordsLoop.empty() && lKFsSharingWordsMerge.empty()) {
    return;
  }

  if (!lKFsSharingWordsLoop.empty()) {
    std::list<std::pair<float, KeyFrame*>> lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    int maxCommonWords = 0;
    for (KeyFrame* const candidate : lKFsSharingWordsLoop) {
      if (candidate->mnLoopWords > maxCommonWords) {
        maxCommonWords = candidate->mnLoopWords;
      }
    }

    int minCommonWords = maxCommonWords * 0.8f;

    int nscores = 0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    for (KeyFrame* const candidate : lKFsSharingWordsLoop) {
      if (candidate->mnLoopWords > minCommonWords) {
        nscores++;

        float si = mpVoc->score(pKF->mBowVec, candidate->mBowVec);

        candidate->mLoopScore = si;
        if (si >= minScore) {
          lScoreAndMatch.push_back(std::make_pair(si, candidate));
        }
      }
    }

    if (!lScoreAndMatch.empty()) {
      std::list<std::pair<float, KeyFrame*>> lAccScoreAndMatch;
      float                                  bestAccScore = minScore;

      // Lets now accumulate score by covisibility
      for (const auto& [score, candidate] : lScoreAndMatch) {
        float     bestScore = score;
        float     accScore  = score;
        KeyFrame* pBestKF   = candidate;
        for (KeyFrame* const neighbor : candidate->GetBestCovisibilityKeyFrames(10)) {
          if (neighbor->mnLoopQuery == pKF->mnId && neighbor->mnLoopWords > minCommonWords) {
            accScore += neighbor->mLoopScore;
            if (neighbor->mLoopScore > bestScore) {
              pBestKF   = neighbor;
              bestScore = neighbor->mLoopScore;
            }
          }
        }

        lAccScoreAndMatch.push_back(std::make_pair(accScore, pBestKF));
        if (accScore > bestAccScore) {
          bestAccScore = accScore;
        }
      }

      // Return all those keyframes with a score higher than 0.75*bestScore
      float minScoreToRetain = 0.75f * bestAccScore;

      std::set<KeyFrame*> spAlreadyAddedKF;
      vpLoopCand.reserve(lAccScoreAndMatch.size());

      for (const auto& [score, candidate] : lAccScoreAndMatch) {
        if (score > minScoreToRetain) {
          if (!spAlreadyAddedKF.count(candidate)) {
            vpLoopCand.push_back(candidate);
            spAlreadyAddedKF.insert(candidate);
          }
        }
      }
    }
  }

  if (!lKFsSharingWordsMerge.empty()) {
    std::list<std::pair<float, KeyFrame*>> lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    int maxCommonWords = 0;
    for (KeyFrame* const candidate : lKFsSharingWordsMerge) {
      if (candidate->mnMergeWords > maxCommonWords) {
        maxCommonWords = candidate->mnMergeWords;
      }
    }

    int minCommonWords = maxCommonWords * 0.8f;

    int nscores = 0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    for (KeyFrame* const candidate : lKFsSharingWordsMerge) {
      if (candidate->mnMergeWords > minCommonWords) {
        nscores++;

        float si = mpVoc->score(pKF->mBowVec, candidate->mBowVec);

        candidate->mMergeScore = si;
        if (si >= minScore) {
          lScoreAndMatch.push_back(std::make_pair(si, candidate));
        }
      }
    }

    if (!lScoreAndMatch.empty()) {
      std::list<std::pair<float, KeyFrame*>> lAccScoreAndMatch;
      float                                  bestAccScore = minScore;

      // Lets now accumulate score by covisibility
      for (const auto& [score, candidate] : lScoreAndMatch) {
        float     bestScore = score;
        float     accScore  = score;
        KeyFrame* pBestKF   = candidate;
        for (KeyFrame* const neighbor : candidate->GetBestCovisibilityKeyFrames(10)) {
          if (neighbor->mnMergeQuery == pKF->mnId && neighbor->mnMergeWords > minCommonWords) {
            accScore += neighbor->mMergeScore;
            if (neighbor->mMergeScore > bestScore) {
              pBestKF   = neighbor;
              bestScore = neighbor->mMergeScore;
            }
          }
        }

        lAccScoreAndMatch.push_back(std::make_pair(accScore, pBestKF));
        if (accScore > bestAccScore) {
          bestAccScore = accScore;
        }
      }

      // Return all those keyframes with a score higher than 0.75*bestScore
      float minScoreToRetain = 0.75f * bestAccScore;

      std::set<KeyFrame*> spAlreadyAddedKF;
      vpMergeCand.reserve(lAccScoreAndMatch.size());

      for (const auto& [score, candidate] : lAccScoreAndMatch) {
        if (score > minScoreToRetain) {
          if (!spAlreadyAddedKF.count(candidate)) {
            vpMergeCand.push_back(candidate);
            spAlreadyAddedKF.insert(candidate);
          }
        }
      }
    }
  }

  for (const unsigned word_id : pKF->mBowVec | std::views::keys) {
    for (KeyFrame* const candidate : mvInvertedFile[word_id]) {
      candidate->mnLoopQuery  = -1;
      candidate->mnMergeQuery = -1;
    }
  }
}

void KeyFrameDatabase::DetectBestCandidates(
  KeyFrame*               pKF,
  std::vector<KeyFrame*>& vpLoopCand,
  std::vector<KeyFrame*>& vpMergeCand,
  int                     nMinWords
) {
  std::list<KeyFrame*> lKFsSharingWords;
  std::set<KeyFrame*>  spConnectedKF;

  // Search all keyframes that share a word with current frame
  {
    std::unique_lock<std::mutex> lock(mMutex);

    spConnectedKF = pKF->GetConnectedKeyFrames();

    for (const unsigned word_id : pKF->mBowVec | std::views::keys) {
      for (KeyFrame* const candidate : mvInvertedFile[word_id]) {
        if (spConnectedKF.find(candidate) != spConnectedKF.end()) {
          continue;
        }
        if (candidate->mnPlaceRecognitionQuery != pKF->mnId) {
          candidate->mnPlaceRecognitionWords = 0;
          candidate->mnPlaceRecognitionQuery = pKF->mnId;
          lKFsSharingWords.push_back(candidate);
        }
        candidate->mnPlaceRecognitionWords++;
      }
    }
  }
  if (lKFsSharingWords.empty()) {
    return;
  }

  // Only compare against those keyframes that share enough words
  int maxCommonWords = 0;
  for (KeyFrame* const candidate : lKFsSharingWords) {
    if (candidate->mnPlaceRecognitionWords > maxCommonWords) {
      maxCommonWords = candidate->mnPlaceRecognitionWords;
    }
  }

  int minCommonWords = maxCommonWords * 0.8f;

  if (minCommonWords < nMinWords) {
    minCommonWords = nMinWords;
  }

  std::list<std::pair<float, KeyFrame*>> lScoreAndMatch;

  int nscores = 0;

  // Compute similarity score.
  for (KeyFrame* const candidate : lKFsSharingWords) {
    if (candidate->mnPlaceRecognitionWords > minCommonWords) {
      nscores++;
      float si                          = mpVoc->score(pKF->mBowVec, candidate->mBowVec);
      candidate->mPlaceRecognitionScore = si;
      lScoreAndMatch.push_back(std::make_pair(si, candidate));
    }
  }

  if (lScoreAndMatch.empty()) {
    return;
  }

  std::list<std::pair<float, KeyFrame*>> lAccScoreAndMatch;
  float                                  bestAccScore = 0;

  // Lets now accumulate score by covisibility
  for (const auto& [score, pKFi] : lScoreAndMatch) {
    float     bestScore = score;
    float     accScore  = score;
    KeyFrame* pBestKF   = pKFi;
    for (KeyFrame* const neighbor : pKFi->GetBestCovisibilityKeyFrames(10)) {
      if (neighbor->mnPlaceRecognitionQuery != pKF->mnId) {
        continue;
      }

      accScore += neighbor->mPlaceRecognitionScore;
      if (neighbor->mPlaceRecognitionScore > bestScore) {
        pBestKF   = neighbor;
        bestScore = neighbor->mPlaceRecognitionScore;
      }
    }
    lAccScoreAndMatch.push_back(std::make_pair(accScore, pBestKF));
    if (accScore > bestAccScore) {
      bestAccScore = accScore;
    }
  }

  // Return all those keyframes with a score higher than 0.75*bestScore
  float               minScoreToRetain = 0.75f * bestAccScore;
  std::set<KeyFrame*> spAlreadyAddedKF;
  vpLoopCand.reserve(lAccScoreAndMatch.size());
  vpMergeCand.reserve(lAccScoreAndMatch.size());
  for (const auto& [score, candidate] : lAccScoreAndMatch) {
    if (score > minScoreToRetain) {
      if (!spAlreadyAddedKF.count(candidate)) {
        if (pKF->GetMap() == candidate->GetMap()) {
          vpLoopCand.push_back(candidate);
        } else {
          vpMergeCand.push_back(candidate);
        }
        spAlreadyAddedKF.insert(candidate);
      }
    }
  }
}

bool compFirst(const std::pair<float, KeyFrame*>& a, const std::pair<float, KeyFrame*>& b) {
  return a.first > b.first;
}

void KeyFrameDatabase::DetectNBestCandidates(
  KeyFrame*               pKF,
  std::vector<KeyFrame*>& vpLoopCand,
  std::vector<KeyFrame*>& vpMergeCand,
  int                     nNumCandidates
) {
  std::list<KeyFrame*> lKFsSharingWords;
  std::set<KeyFrame*>  spConnectedKF;

  // Search all keyframes that share a word with current frame
  {
    std::unique_lock<std::mutex> lock(mMutex);

    spConnectedKF = pKF->GetConnectedKeyFrames();

    for (const unsigned word_id : pKF->mBowVec | std::views::keys) {
      for (KeyFrame* const candidate : mvInvertedFile[word_id]) {
        if (candidate->mnPlaceRecognitionQuery != pKF->mnId) {
          candidate->mnPlaceRecognitionWords = 0;
          if (!spConnectedKF.count(candidate)) {
            candidate->mnPlaceRecognitionQuery = pKF->mnId;
            lKFsSharingWords.push_back(candidate);
          }
        }
        candidate->mnPlaceRecognitionWords++;
      }
    }
  }
  if (lKFsSharingWords.empty()) {
    return;
  }

  // Only compare against those keyframes that share enough words
  int maxCommonWords = 0;
  for (KeyFrame* const candidate : lKFsSharingWords) {
    if (candidate->mnPlaceRecognitionWords > maxCommonWords) {
      maxCommonWords = candidate->mnPlaceRecognitionWords;
    }
  }

  int minCommonWords = maxCommonWords * 0.8f;

  std::list<std::pair<float, KeyFrame*>> lScoreAndMatch;

  int nscores = 0;

  // Compute similarity score.
  for (KeyFrame* const candidate : lKFsSharingWords) {
    if (candidate->mnPlaceRecognitionWords > minCommonWords) {
      nscores++;
      float si                          = mpVoc->score(pKF->mBowVec, candidate->mBowVec);
      candidate->mPlaceRecognitionScore = si;
      lScoreAndMatch.push_back(std::make_pair(si, candidate));
    }
  }

  if (lScoreAndMatch.empty()) {
    return;
  }

  std::list<std::pair<float, KeyFrame*>> lAccScoreAndMatch;
  float                                  bestAccScore = 0;

  // Lets now accumulate score by covisibility
  for (const auto& [score, candidate] : lScoreAndMatch) {
    float     bestScore = score;
    float     accScore  = score;
    KeyFrame* pBestKF   = candidate;
    for (KeyFrame* const neighbor : candidate->GetBestCovisibilityKeyFrames(10)) {
      if (neighbor->mnPlaceRecognitionQuery != pKF->mnId) {
        continue;
      }

      accScore += neighbor->mPlaceRecognitionScore;
      if (neighbor->mPlaceRecognitionScore > bestScore) {
        pBestKF   = neighbor;
        bestScore = neighbor->mPlaceRecognitionScore;
      }
    }
    lAccScoreAndMatch.push_back(std::make_pair(accScore, pBestKF));
    if (accScore > bestAccScore) {
      bestAccScore = accScore;
    }
  }

  lAccScoreAndMatch.sort(compFirst);

  vpLoopCand.reserve(nNumCandidates);
  vpMergeCand.reserve(nNumCandidates);
  std::set<KeyFrame*> spAlreadyAddedKF;
  int                 i  = 0;
  auto                it = lAccScoreAndMatch.begin();
  while (i < lAccScoreAndMatch.size()
         && (vpLoopCand.size() < nNumCandidates || vpMergeCand.size() < nNumCandidates)) {
    KeyFrame* pKFi = it->second;
    if (pKFi->isBad()) {
      continue;
    }

    if (!spAlreadyAddedKF.count(pKFi)) {
      if (pKF->GetMap() == pKFi->GetMap() && vpLoopCand.size() < nNumCandidates) {
        vpLoopCand.push_back(pKFi);
      } else if(pKF->GetMap() != pKFi->GetMap() && vpMergeCand.size() < nNumCandidates && !pKFi->GetMap()->IsBad()) {
        vpMergeCand.push_back(pKFi);
      }
      spAlreadyAddedKF.insert(pKFi);
    }
    i++;
    it++;
  }
}

std::vector<KeyFrame*> KeyFrameDatabase::DetectRelocalizationCandidates(Frame* F, Map* pMap) {
  std::list<KeyFrame*> lKFsSharingWords;

  // Search all keyframes that share a word with current frame
  {
    std::unique_lock<std::mutex> lock(mMutex);

    for (const unsigned word_id : F->mBowVec | std::views::keys) {
      for (KeyFrame* const candidate : mvInvertedFile[word_id]) {
        if (candidate->mnRelocQuery != F->mnId) {
          candidate->mnRelocWords = 0;
          candidate->mnRelocQuery = F->mnId;
          lKFsSharingWords.push_back(candidate);
        }
        candidate->mnRelocWords++;
      }
    }
  }
  if (lKFsSharingWords.empty()) {
    return std::vector<KeyFrame*>();
  }

  // Only compare against those keyframes that share enough words
  int maxCommonWords = 0;
  for (KeyFrame* const candidate : lKFsSharingWords) {
    if (candidate->mnRelocWords > maxCommonWords) {
      maxCommonWords = candidate->mnRelocWords;
    }
  }

  int minCommonWords = maxCommonWords * 0.8f;

  std::list<std::pair<float, KeyFrame*>> lScoreAndMatch;

  int nscores = 0;

  // Compute similarity score.
  for (KeyFrame* const candidate : lKFsSharingWords) {
    if (candidate->mnRelocWords > minCommonWords) {
      nscores++;
      float si               = mpVoc->score(F->mBowVec, candidate->mBowVec);
      candidate->mRelocScore = si;
      lScoreAndMatch.push_back(std::make_pair(si, candidate));
    }
  }

  if (lScoreAndMatch.empty()) {
    return std::vector<KeyFrame*>();
  }

  std::list<std::pair<float, KeyFrame*>> lAccScoreAndMatch;
  float                                  bestAccScore = 0;

  // Lets now accumulate score by covisibility
  for (const auto& [score, candidate] : lScoreAndMatch) {
    float     bestScore = score;
    float     accScore  = score;
    KeyFrame* pBestKF   = candidate;
    for (KeyFrame* const neighbor : candidate->GetBestCovisibilityKeyFrames(10)) {
      if (neighbor->mnRelocQuery != F->mnId) {
        continue;
      }

      accScore += neighbor->mRelocScore;
      if (neighbor->mRelocScore > bestScore) {
        pBestKF   = neighbor;
        bestScore = neighbor->mRelocScore;
      }
    }
    lAccScoreAndMatch.push_back(std::make_pair(accScore, pBestKF));
    if (accScore > bestAccScore) {
      bestAccScore = accScore;
    }
  }

  // Return all those keyframes with a score higher than 0.75*bestScore
  float                  minScoreToRetain = 0.75f * bestAccScore;
  std::set<KeyFrame*>    spAlreadyAddedKF;
  std::vector<KeyFrame*> vpRelocCandidates;
  vpRelocCandidates.reserve(lAccScoreAndMatch.size());
  for (const auto& [score, candidate] : lAccScoreAndMatch) {
    if (score > minScoreToRetain) {
      if (candidate->GetMap() != pMap) {
        continue;
      }
      if (!spAlreadyAddedKF.count(candidate)) {
        vpRelocCandidates.push_back(candidate);
        spAlreadyAddedKF.insert(candidate);
      }
    }
  }

  return vpRelocCandidates;
}

void KeyFrameDatabase::SetORBVocabulary(ORBVocabulary* pORBVoc) {
  ORBVocabulary** ptr;
  ptr  = (ORBVocabulary**)(&mpVoc);
  *ptr = pORBVoc;

  mvInvertedFile.clear();
  mvInvertedFile.resize(mpVoc->size());
}

} // namespace ORB_SLAM3

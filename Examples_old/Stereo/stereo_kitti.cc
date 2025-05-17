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

#include <iomanip>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "System.h"

void LoadImages(
  const std::string&        strPathToSequence,
  std::vector<std::string>& vstrImageLeft,
  std::vector<std::string>& vstrImageRight,
  std::vector<double>&      vTimestamps
);

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << std::endl
              << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence"
              << std::endl;
    return 1;
  }

  // Retrieve paths to images
  std::vector<std::string> vstrImageLeft;
  std::vector<std::string> vstrImageRight;
  std::vector<double>      vTimestamps;
  LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);

  const int nImages = vstrImageLeft.size();

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::Sensor::Stereo, true);
  float             imageScale = SLAM.GetImageScale();

  // Vector for tracking time statistics
  std::vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  std::cout << std::endl << "-------" << std::endl;
  std::cout << "Start processing sequence ..." << std::endl;
  std::cout << "Images in the sequence: " << nImages << std::endl << std::endl;

  double t_track  = 0.f;
  double t_resize = 0.f;

  // Main loop
  cv::Mat imLeft, imRight;
  for (int ni = 0; ni < nImages; ni++) {
    // Read left and right images from file
    imLeft        = cv::imread(vstrImageLeft[ni], cv::IMREAD_UNCHANGED);  //,cv::IMREAD_UNCHANGED);
    imRight       = cv::imread(vstrImageRight[ni], cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
    double tframe = vTimestamps[ni];

    if (imLeft.empty()) {
      std::cerr << std::endl
                << "Failed to load image at: " << std::string(vstrImageLeft[ni]) << std::endl;
      return 1;
    }

    if (imageScale != 1.f) {
#ifdef REGISTER_TIMES
      std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
#endif
      int width  = imLeft.cols * imageScale;
      int height = imLeft.rows * imageScale;
      cv::resize(imLeft, imLeft, cv::Size(width, height));
      cv::resize(imRight, imRight, cv::Size(width, height));
#ifdef REGISTER_TIMES
      std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
      t_resize = std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(
                   t_End_Resize - t_Start_Resize
      )
                   .count();
      SLAM.InsertResizeTime(t_resize);
#endif
    }

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // Pass the images to the SLAM system
    SLAM.TrackStereo(imLeft, imRight, tframe);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

#ifdef REGISTER_TIMES
    t_track
      = t_resize
      + std::chrono::duration_cast<std::chrono::duration<double, std::milli> >(t2 - t1).count();
    SLAM.InsertTrackTime(t_track);
#endif

    double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

    vTimesTrack[ni] = ttrack;

    // Wait to load the next frame
    double T = 0;
    if (ni < nImages - 1) {
      T = vTimestamps[ni + 1] - tframe;
    } else if (ni > 0) {
      T = tframe - vTimestamps[ni - 1];
    }

    if (ttrack < T) {
      usleep((T - ttrack) * 1e6);
    }
  }

  // Stop all threads
  SLAM.Shutdown();

  // Tracking time statistics
  std::sort(vTimesTrack.begin(), vTimesTrack.end());
  float totaltime = 0;
  for (int ni = 0; ni < nImages; ni++) {
    totaltime += vTimesTrack[ni];
  }
  std::cout << "-------" << std::endl << std::endl;
  std::cout << "median tracking time: " << vTimesTrack[nImages / 2] << std::endl;
  std::cout << "mean tracking time: " << totaltime / nImages << std::endl;

  // Save camera trajectory
  SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

  return 0;
}

void LoadImages(
  const std::string&        strPathToSequence,
  std::vector<std::string>& vstrImageLeft,
  std::vector<std::string>& vstrImageRight,
  std::vector<double>&      vTimestamps
) {
  std::ifstream fTimes;
  std::string   strPathTimeFile = strPathToSequence + "/times.txt";
  fTimes.open(strPathTimeFile.c_str());
  while (!fTimes.eof()) {
    std::string s;
    std::getline(fTimes, s);
    if (!s.empty()) {
      std::stringstream ss;
      ss << s;
      double t;
      ss >> t;
      vTimestamps.push_back(t);
    }
  }

  std::string strPrefixLeft  = strPathToSequence + "/image_0/";
  std::string strPrefixRight = strPathToSequence + "/image_1/";

  const int nTimes = vTimestamps.size();
  vstrImageLeft.resize(nTimes);
  vstrImageRight.resize(nTimes);

  for (int i = 0; i < nTimes; i++) {
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(6) << i;
    vstrImageLeft[i]  = strPrefixLeft + ss.str() + ".png";
    vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
  }
}

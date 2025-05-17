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

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "System.h"

void LoadImages(
  const std::string&        strFile,
  std::vector<std::string>& vstrImageFilenames,
  std::vector<double>&      vTimestamps
);

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << std::endl
              << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence"
              << std::endl;
    return 1;
  }

  // Retrieve paths to images
  std::vector<std::string> vstrImageFilenames;
  std::vector<double>      vTimestamps;
  std::string              strFile = std::string(argv[3]) + "/rgb.txt";
  LoadImages(strFile, vstrImageFilenames, vTimestamps);

  int nImages = vstrImageFilenames.size();

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::Sensor::Monocular, true);
  float             imageScale = SLAM.GetImageScale();

  // Vector for tracking time statistics
  std::vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  std::cout << std::endl << "-------" << std::endl;
  std::cout << "Start processing sequence ..." << std::endl;
  std::cout << "Images in the sequence: " << nImages << std::endl << std::endl;

  double t_resize = 0.f;
  double t_track  = 0.f;

  // Main loop
  cv::Mat im;
  for (int ni = 0; ni < nImages; ni++) {
    // Read image from file
    im = cv::imread(
      std::string(argv[3]) + "/" + vstrImageFilenames[ni],
      cv::IMREAD_UNCHANGED
    ); //,cv::IMREAD_UNCHANGED);
    double tframe = vTimestamps[ni];

    if (im.empty()) {
      std::cerr << std::endl
                << "Failed to load image at: " << std::string(argv[3]) << "/"
                << vstrImageFilenames[ni] << std::endl;
      return 1;
    }

    if (imageScale != 1.f) {
#ifdef REGISTER_TIMES
      std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
#endif
      int width  = im.cols * imageScale;
      int height = im.rows * imageScale;
      cv::resize(im, im, cv::Size(width, height));
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

    // Pass the image to the SLAM system
    SLAM.TrackMonocular(im, tframe);

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
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  return 0;
}

void LoadImages(
  const std::string&        strFile,
  std::vector<std::string>& vstrImageFilenames,
  std::vector<double>&      vTimestamps
) {
  std::ifstream f;
  f.open(strFile.c_str());

  // skip first three lines
  std::string s0;
  std::getline(f, s0);
  std::getline(f, s0);
  std::getline(f, s0);

  while (!f.eof()) {
    std::string s;
    std::getline(f, s);
    if (!s.empty()) {
      std::stringstream ss;
      ss << s;
      double      t;
      std::string sRGB;
      ss >> t;
      vTimestamps.push_back(t);
      ss >> sRGB;
      vstrImageFilenames.push_back(sRGB);
    }
  }
}

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

#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/cfg/argv.h>
#include <spdlog/cfg/env.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>
#include "Common/TUMVI.h"
#include "Converter.h"
#include "LoggingUtils.h"
#include "System.h"

namespace fs = std::filesystem;

double ttrack_tot = 0;

int main(int argc, char** argv) {
  // Load env vars and args.
  spdlog::cfg::load_env_levels();
  spdlog::cfg::load_argv_levels(argc, argv);
  // Initialize application logger.
  ORB_SLAM3::logging::InitializeAppLogger("ORB-SLAM3", false);
  // Add file sink to the application logger.
  const std::string basename  = fs::path(argv[0]).stem().string();
  const std::string logfile   = fmt::format("/tmp/{}.log", basename);
  auto              file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(logfile);
  spdlog::default_logger()->sinks().push_back(file_sink);

  // Parse arguments.
  std::string              vocabulary_file, settings_file, output_dir;
  std::vector<std::string> sequences;
  try {
    const bool parsed = ORB_SLAM3::TUMVI::ParseArguments(
      argc,
      argv,
      vocabulary_file,
      settings_file,
      sequences,
      output_dir,
      ORB_SLAM3::Sensor::InertialMonocular
    );
    if (!parsed) {
      return 0;
    }
  } catch (const std::exception& e) {
    spdlog::error("Error when parsing arguments: {}", e.what());
    return 1;
  }

  // Run.
  try {
    const int num_seq = sequences.size() / 2;

    // Load all sequences:
    int                              seq;
    std::vector<vector<std::string>> vstrImageFilenames;
    std::vector<vector<double>>      vTimestampsCam;
    std::vector<int>                 nImages;

    vstrImageFilenames.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    nImages.resize(num_seq);

    int tot_images = 0;
    for (seq = 0; seq < num_seq; seq++) {
      std::string pathSeq        = sequences[2 * seq];
      std::string pathTimeStamps = sequences[2 * seq + 1];

      spdlog::info("Loading images for sequence {}...", seq);
      ORB_SLAM3::TUMVI::LoadMonocularImages(
        pathSeq,
        pathTimeStamps,
        vstrImageFilenames[seq],
        vTimestampsCam[seq]
      );
      spdlog::info("Images loaded!");

      nImages[seq] = vstrImageFilenames[seq].size();
      tot_images   += nImages[seq];

      if ((nImages[seq] <= 0)) {
        spdlog::error("Failed to load images for sequence {}", seq);
        return 1;
      }
    }
    // Vector for tracking time statistics
    std::vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System
          SLAM(vocabulary_file, settings_file, ORB_SLAM3::Sensor::Monocular, false, 0, output_dir);
    float imageScale = SLAM.GetImageScale();

    double t_resize = 0.f;
    double t_track  = 0.f;

    int proccIm = 0;
    for (seq = 0; seq < num_seq; seq++) {
      // Main loop
      cv::Mat im;
      proccIm                  = 0;
      cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
      for (int ni = 0; ni < nImages[seq]; ni++, proccIm++) {
        // Read image from file
        im
          = cv::imread(vstrImageFilenames[seq][ni], cv::IMREAD_GRAYSCALE); //,cv::IMREAD_GRAYSCALE);

        if (imageScale != 1.f) {
#ifdef REGISTER_TIMES
          std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
#endif
          int width  = im.cols * imageScale;
          int height = im.rows * imageScale;
          cv::resize(im, im, cv::Size(width, height));
#ifdef REGISTER_TIMES
          std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
          t_resize = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                       t_End_Resize - t_Start_Resize
          )
                       .count();
          SLAM.InsertResizeTime(t_resize);
#endif
        }

        // clahe
        clahe->apply(im, im);

        double tframe = vTimestampsCam[seq][ni];

        if (im.empty()) {
          spdlog::error("Failed to load image at: {}", vstrImageFilenames[seq][ni]);
          return 1;
        }
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im, tframe); // TODO change to monocular_inertial

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

#ifdef REGISTER_TIMES
        t_track
          = t_resize
          + std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t2 - t1).count();
        SLAM.InsertTrackTime(t_track);
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
        ttrack_tot    += ttrack;

        vTimesTrack[ni] = ttrack;

        // Wait to load the next frame
        double T = 0;
        if (ni < nImages[seq] - 1) {
          T = vTimestampsCam[seq][ni + 1] - tframe;
        } else if (ni > 0) {
          T = tframe - vTimestampsCam[seq][ni - 1];
        }

        if (ttrack < T) {
          usleep((T - ttrack) * 1e6); // 1e6
        }
      }
      if (seq < num_seq - 1) {
        spdlog::info("Changing the dataset...");
        SLAM.ChangeDataset();
      }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics

    // Save camera trajectory
    fs::path output_file_path;
    output_file_path = fs::path(output_dir) / "CameraTrajectory.txt";
    SLAM.SaveTrajectoryEuRoC(output_file_path.string());
    output_file_path = fs::path(output_dir) / "KeyFrameTrajectory.txt";
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

    std::sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages[0]; ni++) {
      totaltime += vTimesTrack[ni];
    }
    spdlog::info("median tracking time: {}", vTimesTrack[nImages[0] / 2]);
    spdlog::info("mean tracking time: {}", totaltime / proccIm);
  } catch (const std::exception& e) {
    spdlog::error("Error when running ORB-SLAM3: {}", e.what());
  } catch (...) {
    spdlog::error("Unknown error when running ORB-SLAM3");
  }

  return 0;
}

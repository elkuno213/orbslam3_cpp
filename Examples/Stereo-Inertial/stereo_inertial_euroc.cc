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
#include <spdlog/cfg/argv.h>
#include <spdlog/cfg/env.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>
#include "Common/EuRoC.h"
#include "ImuTypes.h"
#include "LoggingUtils.h"
#include "Optimizer.h"
#include "System.h"

namespace fs = std::filesystem;

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
    const bool parsed = ORB_SLAM3::EuRoC::ParseArguments(
      argc,
      argv,
      vocabulary_file,
      settings_file,
      sequences,
      output_dir
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
    std::vector<vector<std::string>> vstrImageLeft;
    std::vector<vector<std::string>> vstrImageRight;
    std::vector<vector<double>>      vTimestampsCam;
    std::vector<vector<cv::Point3f>> vAcc, vGyro;
    std::vector<vector<double>>      vTimestampsImu;
    std::vector<int>                 nImages;
    std::vector<int>                 nImu;
    std::vector<int>                 first_imu(num_seq, 0);

    vstrImageLeft.resize(num_seq);
    vstrImageRight.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    vAcc.resize(num_seq);
    vGyro.resize(num_seq);
    vTimestampsImu.resize(num_seq);
    nImages.resize(num_seq);
    nImu.resize(num_seq);

    int tot_images = 0;
    for (seq = 0; seq < num_seq; seq++) {
      std::string pathSeq        = sequences[2 * seq];
      std::string pathTimeStamps = sequences[2 * seq + 1];

      std::string pathCam0 = pathSeq + "/mav0/cam0/data";
      std::string pathCam1 = pathSeq + "/mav0/cam1/data";
      std::string pathImu  = pathSeq + "/mav0/imu0/data.csv";

      spdlog::info("Loading images for sequence {}...", seq);
      ORB_SLAM3::EuRoC::LoadStereoImages(
        pathCam0,
        pathCam1,
        pathTimeStamps,
        vstrImageLeft[seq],
        vstrImageRight[seq],
        vTimestampsCam[seq]
      );
      spdlog::info("Images loaded!");

      spdlog::info("Loading IMU for sequence {}...", seq);
      ORB_SLAM3::EuRoC::LoadIMU(pathImu, vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
      spdlog::info("IMU data loaded!");

      nImages[seq] = vstrImageLeft[seq].size();
      tot_images   += nImages[seq];
      nImu[seq]    = vTimestampsImu[seq].size();

      if ((nImages[seq] <= 0) || (nImu[seq] <= 0)) {
        spdlog::error("Failed to load images or IMU for sequence {}", seq);
        return 1;
      }

      // Find first imu to be considered, supposing imu measurements start first

      while (vTimestampsImu[seq][first_imu[seq]] <= vTimestampsCam[seq][0]) {
        first_imu[seq]++;
      }
      first_imu[seq]--; // first imu measurement to be considered
    }

    // Read rectification parameters
    cv::FileStorage fsSettings(settings_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
      spdlog::error("Wrong path to settings file: {}", settings_file);
      return -1;
    }

    // Vector for tracking time statistics
    std::vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System
      SLAM(vocabulary_file, settings_file, ORB_SLAM3::Sensor::InertialStereo, false);

    cv::Mat imLeft, imRight;
    for (seq = 0; seq < num_seq; seq++) {
      // Seq loop
      std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
      double                             t_rect   = 0.f;
      double                             t_resize = 0.f;
      double                             t_track  = 0.f;
      int                                num_rect = 0;
      int                                proccIm  = 0;
      for (int ni = 0; ni < nImages[seq]; ni++, proccIm++) {
        // Read left and right images from file
        imLeft  = cv::imread(vstrImageLeft[seq][ni], cv::IMREAD_UNCHANGED);
        imRight = cv::imread(vstrImageRight[seq][ni], cv::IMREAD_UNCHANGED);

        if (imLeft.empty()) {
          spdlog::error("Failed to load image at: {}", vstrImageLeft[seq][ni]);
          return 1;
        }

        if (imRight.empty()) {
          spdlog::error("Failed to load image at: {}", vstrImageRight[seq][ni]);
          return 1;
        }

        double tframe = vTimestampsCam[seq][ni];

        // Load imu measurements from previous frame
        vImuMeas.clear();

        if (ni > 0) {
          while (vTimestampsImu[seq][first_imu[seq]] <= vTimestampsCam[seq][ni]
          ) // while(vTimestampsImu[first_imu]<=vTimestampsCam[ni])
          {
            vImuMeas.push_back(ORB_SLAM3::IMU::Point(
              vAcc[seq][first_imu[seq]].x,
              vAcc[seq][first_imu[seq]].y,
              vAcc[seq][first_imu[seq]].z,
              vGyro[seq][first_imu[seq]].x,
              vGyro[seq][first_imu[seq]].y,
              vGyro[seq][first_imu[seq]].z,
              vTimestampsImu[seq][first_imu[seq]]
            ));
            first_imu[seq]++;
          }
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft, imRight, tframe, vImuMeas);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

#ifdef REGISTER_TIMES
        t_track
          = t_rect + t_resize
          + std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t2 - t1).count();
        SLAM.InsertTrackTime(t_track);
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

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

    // Save camera trajectory
    fs::path output_file_path;
    output_file_path = fs::path(output_dir) / "CameraTrajectory.txt";
    SLAM.SaveTrajectoryEuRoC(output_file_path.string());
    output_file_path = fs::path(output_dir) / "KeyFrameTrajectory.txt";
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
  } catch (const std::exception& e) {
    spdlog::error("Error when running ORB-SLAM3: {}", e.what());
  } catch (...) {
    spdlog::error("Unknown error when running ORB-SLAM3");
  }

  return 0;
}

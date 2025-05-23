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

#include <condition_variable>
#include <csignal>
#include <filesystem>
#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/cfg/argv.h>
#include <spdlog/cfg/env.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>
#include "Common/RealSense.h"
#include "ImuTypes.h"
#include "LoggingUtils.h"
#include "System.h"

namespace fs = std::filesystem;

bool b_continue_session;

void exit_loop_handler(int s) {
  spdlog::info("Finishing session");
  b_continue_session = false;
}

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
  std::string vocabulary_file, settings_file, output_dir;
  try {
    const bool parsed = ORB_SLAM3::RealSense::ParseArguments(
      argc,
      argv,
      vocabulary_file,
      settings_file,
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
    ORB_SLAM3::System
      SLAM(vocabulary_file, settings_file, ORB_SLAM3::System::IMU_MONOCULAR, true, 0, output_dir);
    float imageScale = SLAM.GetImageScale();

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    double offset = 0; // ms

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // Enable both image strams (for some reason realsense does not allow to enable just one)
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8, 30);

    // Add streams of gyro and accelerometer to configuration
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    std::mutex              imu_mutex;
    std::condition_variable cond_image_rec;

    std::vector<double>     v_accel_timestamp;
    std::vector<rs2_vector> v_accel_data;
    std::vector<double>     v_gyro_timestamp;
    std::vector<rs2_vector> v_gyro_data;

    double                  prev_accel_timestamp = 0;
    rs2_vector              prev_accel_data;
    double                  current_accel_timestamp = 0;
    rs2_vector              current_accel_data;
    std::vector<double>     v_accel_timestamp_sync;
    std::vector<rs2_vector> v_accel_data_sync;

    cv::Mat imCV, imCV_right;
    int     width_img, height_img; // width_img = 848, height_img = 800;
    double  timestamp_image = -1.0;
    bool    image_ready     = false;
    int     count_im_buffer = 0; // count dropped frames

    auto imu_callback = [&](const rs2::frame& frame) {
      std::unique_lock<std::mutex> lock(imu_mutex);

      if (rs2::frameset fs = frame.as<rs2::frameset>()) {
        count_im_buffer++;

        double new_timestamp_image = fs.get_timestamp() * 1e-3;
        if (std::abs(timestamp_image - new_timestamp_image) < 0.001) {
          count_im_buffer--;
          return;
        }

        rs2::video_frame color_frame = fs.get_fisheye_frame(1);
        // rs2::video_frame color_frame_right = fs.get_fisheye_frame(2);
        imCV = cv::Mat(
          cv::Size(width_img, height_img),
          CV_8U,
          (void*)(color_frame.get_data()),
          cv::Mat::AUTO_STEP
        );

        timestamp_image = new_timestamp_image;
        // fs.get_timestamp()*1e-3;
        image_ready = true;

        while (v_gyro_timestamp.size() > v_accel_timestamp_sync.size()) {
          int    index       = v_accel_timestamp_sync.size();
          double target_time = v_gyro_timestamp[index];

          rs2_vector interp_data = ORB_SLAM3::RealSense::interpolateMeasure(
            target_time,
            current_accel_data,
            current_accel_timestamp,
            prev_accel_data,
            prev_accel_timestamp
          );
          v_accel_data_sync.push_back(interp_data);
          v_accel_timestamp_sync.push_back(target_time);
        }

        lock.unlock();
        cond_image_rec.notify_all();
      } else if (rs2::motion_frame m_frame = frame.as<rs2::motion_frame>()) {
        if (m_frame.get_profile().stream_name() == "Gyro") {
          // It runs at 200Hz
          v_gyro_data.push_back(m_frame.get_motion_data());
          v_gyro_timestamp.push_back((m_frame.get_timestamp() + offset) * 1e-3);
        } else if (m_frame.get_profile().stream_name() == "Accel") {
          // It runs at 60Hz
          prev_accel_timestamp = current_accel_timestamp;
          prev_accel_data      = current_accel_data;

          current_accel_data      = m_frame.get_motion_data();
          current_accel_timestamp = (m_frame.get_timestamp() + offset) * 1e-3;

          while (v_gyro_timestamp.size() > v_accel_timestamp_sync.size()) {
            int    index       = v_accel_timestamp_sync.size();
            double target_time = v_gyro_timestamp[index];

            rs2_vector interp_data = ORB_SLAM3::RealSense::interpolateMeasure(
              target_time,
              current_accel_data,
              current_accel_timestamp,
              prev_accel_data,
              prev_accel_timestamp
            );

            v_accel_data_sync.push_back(interp_data);
            v_accel_timestamp_sync.push_back(target_time);
          }
        }
      }
    };

    rs2::pipeline_profile pipe_profile = pipe.start(cfg, imu_callback);

    rs2::stream_profile cam_stream = pipe_profile.get_stream(RS2_STREAM_FISHEYE, 1);
    rs2::stream_profile imu_stream = pipe_profile.get_stream(RS2_STREAM_GYRO);
    float*              Rbc        = cam_stream.get_extrinsics_to(imu_stream).rotation;
    float*              tbc        = cam_stream.get_extrinsics_to(imu_stream).translation;

    rs2_intrinsics intrinsics_cam = cam_stream.as<rs2::video_stream_profile>().get_intrinsics();
    width_img                     = intrinsics_cam.width;
    height_img                    = intrinsics_cam.height;

    spdlog::info(
      R"(
      Camera parameters:
        Intrinsics:
          fx: {:.6f}
          fy: {:.6f}
          cx: {:.6f}
          cy: {:.6f}
        Resolution: {}x{}
        Distortion coefficients: [{:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}]
        Model: {}
    )",
      intrinsics_cam.fx,
      intrinsics_cam.fy,
      intrinsics_cam.ppx,
      intrinsics_cam.ppy,
      intrinsics_cam.width,
      intrinsics_cam.height,
      intrinsics_cam.coeffs[0],
      intrinsics_cam.coeffs[1],
      intrinsics_cam.coeffs[2],
      intrinsics_cam.coeffs[3],
      intrinsics_cam.coeffs[4],
      intrinsics_cam.model
    );

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;

    double  timestamp;
    cv::Mat im;

    // Clear IMU vectors
    v_gyro_data.clear();
    v_gyro_timestamp.clear();
    v_accel_data_sync.clear();
    v_accel_timestamp_sync.clear();

    double t_resize = 0.f;
    double t_track  = 0.f;

    std::chrono::steady_clock::time_point time_Start_Process;
    while (!SLAM.isShutDown()) {
      std::vector<rs2_vector> vGyro;
      std::vector<double>     vGyro_times;
      std::vector<rs2_vector> vAccel;
      std::vector<double>     vAccel_times;

      {
        std::unique_lock<std::mutex> lk(imu_mutex);
        while (!image_ready) {
          cond_image_rec.wait(lk);
        }

        std::chrono::steady_clock::time_point time_Start_Process = std::chrono::steady_clock::now();

        if (count_im_buffer > 1) {
          spdlog::warn("Dropped frames: {}", count_im_buffer - 1);
        }
        count_im_buffer = 0;

        while (v_gyro_timestamp.size() > v_accel_timestamp_sync.size()) {
          int    index       = v_accel_timestamp_sync.size();
          double target_time = v_gyro_timestamp[index];

          rs2_vector interp_data = ORB_SLAM3::RealSense::interpolateMeasure(
            target_time,
            current_accel_data,
            current_accel_timestamp,
            prev_accel_data,
            prev_accel_timestamp
          );

          v_accel_data_sync.push_back(interp_data);
          v_accel_timestamp_sync.push_back(target_time);
        }

        if (imageScale == 1.f) {
          im = imCV.clone();
        } else {
#ifdef REGISTER_TIMES
          std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
#endif
          int width  = imCV.cols * imageScale;
          int height = imCV.rows * imageScale;
          cv::resize(imCV, im, cv::Size(width, height));
#ifdef REGISTER_TIMES
          std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
          t_resize = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                       t_End_Resize - t_Start_Resize
          )
                       .count();
          SLAM.InsertResizeTime(t_resize);
#endif
        }

        // Copy the IMU data
        vGyro        = v_gyro_data;
        vGyro_times  = v_gyro_timestamp;
        vAccel       = v_accel_data_sync;
        vAccel_times = v_accel_timestamp_sync;
        timestamp    = timestamp_image;

        // Clear IMU vectors
        v_gyro_data.clear();
        v_gyro_timestamp.clear();
        v_accel_data_sync.clear();
        v_accel_timestamp_sync.clear();

        image_ready = false;
      }

      for (int i = 0; i < vGyro.size(); ++i) {
        ORB_SLAM3::IMU::Point lastPoint(
          vAccel[i].x,
          vAccel[i].y,
          vAccel[i].z,
          vGyro[i].x,
          vGyro[i].y,
          vGyro[i].z,
          vGyro_times[i]
        );
        vImuMeas.push_back(lastPoint);

        if(std::isnan(vAccel[i].x) || std::isnan(vAccel[i].y) || std::isnan(vAccel[i].z) ||
               std::isnan(vGyro[i].x) || std::isnan(vGyro[i].y) || std::isnan(vGyro[i].z) ||
               std::isnan(vGyro_times[i])){
          spdlog::error("NAN values found in IMU data");
          return 1;
        }
      }

      std::chrono::steady_clock::time_point t_Start_Track = std::chrono::steady_clock::now();
      // Pass the image to the SLAM system
      SLAM.TrackMonocular(im, timestamp, vImuMeas);

      std::chrono::steady_clock::time_point t_End_Track = std::chrono::steady_clock::now();

#ifdef REGISTER_TIMES
      t_track = t_resize
              + std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                  t_End_Track - t_Start_Track
              )
                  .count();
      SLAM.InsertTrackTime(t_track);
#endif

      double timeProcess = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                             t_Start_Track - time_Start_Process
      )
                             .count();
      double timeSLAM = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                          t_End_Track - t_Start_Track
      )
                          .count();

      // Clear the previous IMU measurements to load the new ones
      vImuMeas.clear();
    }

    SLAM.Shutdown();
  } catch (const std::exception& e) {
    spdlog::error("Error when running ORB-SLAM3: {}", e.what());
  } catch (...) {
    spdlog::error("Unknown error when running ORB-SLAM3");
  }

  return 0;
}

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
#include <librealsense2/rsutil.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/cfg/argv.h>
#include <spdlog/cfg/env.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>
#include "Common/RealSense.h"
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
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    double offset = 0; // ms

    rs2::context     ctx;
    rs2::device_list devices = ctx.query_devices();
    rs2::device      selected_device;
    if (devices.size() == 0) {
      spdlog::error("No device connected, please connect a RealSense device");
      return 1;
    } else {
      selected_device = devices[0];
    }

    std::vector<rs2::sensor> sensors = selected_device.query_sensors();
    int                      index   = 0;
    // We can now iterate the sensors and print their names
    for (rs2::sensor sensor : sensors) {
      if (sensor.supports(RS2_CAMERA_INFO_NAME)) {
        ++index;
        if (index == 1) {
          sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
          sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT, 5000);
          sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0); // switch off emitter
        }
        ORB_SLAM3::RealSense::get_sensor_option(sensor);
        if (index == 2) {
          // RGB camera (not used here...)
          sensor.set_option(RS2_OPTION_EXPOSURE, 100.f);
        }
      }
    }

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 30);

    // IMU callback
    std::mutex              imu_mutex;
    std::condition_variable cond_image_rec;

    cv::Mat imCV, imRightCV;
    int     width_img, height_img;
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

        rs2::video_frame ir_frameL = fs.get_infrared_frame(1);
        rs2::video_frame ir_frameR = fs.get_infrared_frame(2);

        imCV = cv::Mat(
          cv::Size(width_img, height_img),
          CV_8U,
          (void*)(ir_frameL.get_data()),
          cv::Mat::AUTO_STEP
        );
        imRightCV = cv::Mat(
          cv::Size(width_img, height_img),
          CV_8U,
          (void*)(ir_frameR.get_data()),
          cv::Mat::AUTO_STEP
        );

        timestamp_image = fs.get_timestamp() * 1e-3;
        image_ready     = true;

        lock.unlock();
        cond_image_rec.notify_all();
      }
    };

    rs2::pipeline_profile pipe_profile = pipe.start(cfg, imu_callback);

    rs2::stream_profile cam_left  = pipe_profile.get_stream(RS2_STREAM_INFRARED, 1);
    rs2::stream_profile cam_right = pipe_profile.get_stream(RS2_STREAM_INFRARED, 2);

    float* Rlr = cam_right.get_extrinsics_to(cam_left).rotation;
    float* tlr = cam_right.get_extrinsics_to(cam_left).translation;

    rs2_intrinsics intrinsics_left = cam_left.as<rs2::video_stream_profile>().get_intrinsics();
    width_img                      = intrinsics_left.width;
    height_img                     = intrinsics_left.height;

    spdlog::info(
      R"(
      Left camera parameters:
        Intrinsics:
          fx: {:.6f}
          fy: {:.6f}
          cx: {:.6f}
          cy: {:.6f}
        Resolution: {}x{}
        Distortion coefficients: [{:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}]
        Model: {}
    )",
      intrinsics_left.fx,
      intrinsics_left.fy,
      intrinsics_left.ppx,
      intrinsics_left.ppy,
      intrinsics_left.width,
      intrinsics_left.height,
      intrinsics_left.coeffs[0],
      intrinsics_left.coeffs[1],
      intrinsics_left.coeffs[2],
      intrinsics_left.coeffs[3],
      intrinsics_left.coeffs[4],
      intrinsics_left.model
    );

    rs2_intrinsics intrinsics_right = cam_right.as<rs2::video_stream_profile>().get_intrinsics();
    width_img                       = intrinsics_right.width;
    height_img                      = intrinsics_right.height;

    spdlog::info(
      R"(
      Right camera parameters:
        Intrinsics:
          fx: {:.6f}
          fy: {:.6f}
          cx: {:.6f}
          cy: {:.6f}
        Resolution: {}x{}
        Distortion coefficients: [{:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}]
        Model: {}
    )",
      intrinsics_right.fx,
      intrinsics_right.fy,
      intrinsics_right.ppx,
      intrinsics_right.ppy,
      intrinsics_right.width,
      intrinsics_right.height,
      intrinsics_right.coeffs[0],
      intrinsics_right.coeffs[1],
      intrinsics_right.coeffs[2],
      intrinsics_right.coeffs[3],
      intrinsics_right.coeffs[4],
      intrinsics_right.model
    );

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System
          SLAM(vocabulary_file, settings_file, ORB_SLAM3::Sensor::Stereo, true, 0, output_dir);
    float imageScale = SLAM.GetImageScale();

    double  timestamp;
    cv::Mat im, imRight;

    double t_resize = 0.f;
    double t_track  = 0.f;

    while (!SLAM.isShutDown()) {
      std::vector<rs2_vector> vGyro;
      std::vector<double>     vGyro_times;
      std::vector<rs2_vector> vAccel;
      std::vector<double>     vAccel_times;

      {
        std::unique_lock<std::mutex> lk(imu_mutex);
        if (!image_ready) {
          cond_image_rec.wait(lk);
        }

        std::chrono::steady_clock::time_point time_Start_Process = std::chrono::steady_clock::now();

        if (count_im_buffer > 1) {
          spdlog::warn("Dropped frames: {}", count_im_buffer - 1);
        }
        count_im_buffer = 0;

        timestamp = timestamp_image;
        im        = imCV.clone();
        imRight   = imRightCV.clone();

        image_ready = false;
      }

      if (imageScale != 1.f) {
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
#endif
        int width  = im.cols * imageScale;
        int height = im.rows * imageScale;
        cv::resize(im, im, cv::Size(width, height));
        cv::resize(imRight, imRight, cv::Size(width, height));

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
        t_resize = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                     t_End_Resize - t_Start_Resize
        )
                     .count();
        SLAM.InsertResizeTime(t_resize);
#endif
      }

#ifdef REGISTER_TIMES
      std::chrono::steady_clock::time_point t_Start_Track = std::chrono::steady_clock::now();
#endif
      // Stereo images are already rectified.
      SLAM.TrackStereo(im, imRight, timestamp);
#ifdef REGISTER_TIMES
      std::chrono::steady_clock::time_point t_End_Track = std::chrono::steady_clock::now();
      t_track                                           = t_resize
              + std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                  t_End_Track - t_Start_Track
              )
                  .count();
      SLAM.InsertTrackTime(t_track);
#endif
    }
  } catch (const std::exception& e) {
    spdlog::error("Error when running ORB-SLAM3: {}", e.what());
  } catch (...) {
    spdlog::error("Unknown error when running ORB-SLAM3");
  }

  return 0;
}

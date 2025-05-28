//
// The MIT License (MIT)
//
// Copyright (c) 2023 Your Name/Company (as this is new code)
// (Portions based on existing Livox DRIVER code may retain original copyright)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "lds_lvx_reader.h"
#include <chrono> // For std::chrono::milliseconds
#include <iostream> // For temporary printf/cout

// Placeholder for Livox SDK header(s) needed for LVX file reading.
// #include "livox_sdk_lvx.h" 

// Placeholder for ROS logging if available/needed, otherwise use std::cout or printf
// For ROS1:
// #include <ros/console.h>
// #define LVX_LOG_INFO ROS_INFO
// #define LVX_LOG_ERROR ROS_ERROR
// #define LVX_LOG_WARN ROS_WARN
// For ROS2 (assuming a node context is passed or available globally):
// #include "rclcpp/rclcpp.hpp"
// #define LVX_LOG_INFO(...) RCLCPP_INFO(rclcpp::get_logger("LdsLvxReader"), __VA_ARGS__)
// #define LVX_LOG_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("LdsLvxReader"), __VA_ARGS__)
// #define LVX_LOG_WARN(...) RCLCPP_WARN(rclcpp::get_logger("LdsLvxReader"), __VA_ARGS__)
// For now, using std::cout as a placeholder for logging
#define LVX_LOG_INFO(msg) std::cout << "[INFO] LdsLvxReader: " << msg << std::endl
#define LVX_LOG_ERROR(msg) std::cerr << "[ERROR] LdsLvxReader: " << msg << std::endl
#define LVX_LOG_WARN(msg) std::cout << "[WARN] LdsLvxReader: " << msg << std::endl


namespace livox_ros {

LdsLvxReader::LdsLvxReader(double publish_freq, const std::string& lvx_file_path)
    : Lds(publish_freq, kSourceLvxFile), // Call base constructor with correct data_src
      lvx_file_path_(lvx_file_path),
      stop_flag_(false) {
  LVX_LOG_INFO("LdsLvxReader constructor called. File path: " + lvx_file_path_);
  // Initialize Livox SDK specific members if necessary
  // lvx_sdk_handle_ = nullptr; 
}

LdsLvxReader::~LdsLvxReader() {
  LVX_LOG_INFO("LdsLvxReader destructor called.");
  PrepareExit(); // Ensure thread is stopped and resources are released
}

bool LdsLvxReader::Init() {
  LVX_LOG_INFO("Initializing LVX reader for file: " + lvx_file_path_);
  std::lock_guard<std::mutex> lock(sdk_mutex_); // Protect SDK calls

  // Placeholder: Livox SDK call to open the LVX file
  // For example:
  // lvx_sdk_handle_ = LivoxSdkLvxOpenFile(lvx_file_path_.c_str());
  // if (lvx_sdk_handle_ == nullptr) {
  //   LVX_LOG_ERROR("Failed to open LVX file: " + lvx_file_path_ + " using Livox SDK.");
  //   return false;
  // }

  // Placeholder: Check file validity, read metadata, etc. using SDK
  // if (!LivoxSdkLvxValidateFile(lvx_sdk_handle_)) {
  //   LVX_LOG_ERROR("LVX file is invalid or not supported: " + lvx_file_path_);
  //   LivoxSdkLvxCloseFile(lvx_sdk_handle_); // Close if opened
  //   lvx_sdk_handle_ = nullptr;
  //   return false;
  // }

  // For demonstration, we'll assume success if the file path is not empty.
  // In a real implementation, this would depend on the SDK's success.
  if (lvx_file_path_.empty()) {
    LVX_LOG_ERROR("LVX file path is empty.");
    return false;
  }
  
  // Simulate successful SDK initialization for now
  // lvx_sdk_handle_ = reinterpret_cast<void*>(0xDEADBEEF); // Dummy handle

  LVX_LOG_INFO("LVX file opened and validated (simulated): " + lvx_file_path_);
  return true;
}

void LdsLvxReader::StartRead() {
  if (!read_thread_.joinable()) { // Prevent starting multiple threads
    stop_flag_.store(false);
    read_thread_ = std::thread(&LdsLvxReader::ReadLoop, this);
    LVX_LOG_INFO("LVX reading thread started.");
  } else {
    LVX_LOG_WARN("LVX reading thread already running.");
  }
}

void LdsLvxReader::PrepareExit() {
  LVX_LOG_INFO("PrepareExit called. Stopping LVX reading thread...");
  stop_flag_.store(true);
  if (read_thread_.joinable()) {
    read_thread_.join();
    LVX_LOG_INFO("LVX reading thread joined.");
  }

  std::lock_guard<std::mutex> lock(sdk_mutex_); // Protect SDK calls
  // Placeholder: Livox SDK call to close the LVX file
  // if (lvx_sdk_handle_ != nullptr) {
  //   LivoxSdkLvxCloseFile(lvx_sdk_handle_);
  //   lvx_sdk_handle_ = nullptr;
  //   LVX_LOG_INFO("LVX file closed (simulated).");
  // }
}

void LdsLvxReader::ReadLoop() {
  LVX_LOG_INFO("LVX ReadLoop started.");

  // Calculate delay based on publish frequency.
  // This is a simple way to control playback speed.
  // A more sophisticated approach would use timestamps from the LVX file.
  long long VPublishIntervalNs = static_cast<long long>((1.0 / GetLdsFrequency()) * 1e9);
  std::chrono::nanoseconds publish_duration(VPublishIntervalNs);
  
  // Simulate reading frames. In a real scenario, this loop would use SDK calls.
  int frame_count = 0; // Dummy frame counter for simulation

  while (!stop_flag_.load()) {
    auto VStartTime = std::chrono::high_resolution_clock::now();

    PointFrame point_frame; // This would be populated by the SDK
    bool VHasFrame = false;

    { // Scope for SDK mutex
      std::lock_guard<std::mutex> lock(sdk_mutex_);
      // Placeholder: Livox SDK call to read the next frame
      // For example:
      // LivoxSdkLvxReadNextFrame(lvx_sdk_handle_, &point_frame, &VHasFrame);
      // if (!VHasFrame) { // End of file or error
      //    LVX_LOG_INFO("End of LVX file or read error.");
      //    break; 
      // }

      // Simulate reading a frame for now
      if (frame_count < 100) { // Simulate 100 frames
        // Create a dummy PointFrame for demonstration
        point_frame.lidar_num = 1; // Assume one lidar in the LVX for simplicity
        point_frame.base_time[0] = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                       std::chrono::system_clock::now().time_since_epoch()).count();
        point_frame.lidar_point[0].handle = 0; // Dummy handle
        point_frame.lidar_point[0].lidar_type = kLivoxLidarType; // Example type
        point_frame.lidar_point[0].points_num = 10; // 10 dummy points
        
        // Allocate dummy points - actual point data structure would be more complex
        // For simplicity, we don't populate PointXyzlt content here.
        // In a real scenario, the SDK would fill this.
        // point_frame.lidar_point[0].points = new PointXyzlt[10]; 


        VHasFrame = true;
        frame_count++;
      } else {
        LVX_LOG_INFO("Simulated end of LVX file.");
        VHasFrame = false; // Simulate end of file
      }
    } // End of SDK mutex scope

    if (VHasFrame) {
      StorageLvxPointData(&point_frame); // Use base class method to store data

      // Clean up dummy points if allocated (important for real implementation)
      // delete[] point_frame.lidar_point[0].points;
      // point_frame.lidar_point[0].points = nullptr;

    } else {
      // End of file or error, break the loop
      break;
    }

    auto VEndTime = std::chrono::high_resolution_clock::now();
    auto VElapsedNs = std::chrono::duration_cast<std::chrono::nanoseconds>(VEndTime - VStartTime);
    
    if (VElapsedNs < publish_duration) {
      std::this_thread::sleep_for(publish_duration - VElapsedNs);
    }
  }
  LVX_LOG_INFO("LVX ReadLoop finished.");
}

}  // namespace livox_ros

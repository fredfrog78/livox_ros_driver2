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

#ifndef LIVOX_ROS_DRIVER_LDS_LVX_READER_H_
#define LIVOX_ROS_DRIVER_LDS_LVX_READER_H_

#include <string>
#include <thread>
#include <atomic>
#include <mutex>

#include "lds.h"
#include "comm/comm.h" // For kSourceLvxFile and PointFrame

// Placeholder for Livox SDK header(s) needed for LVX file reading.
// For example, it might be something like:
// #include "livox_sdk_lvx.h" 

namespace livox_ros {

class LdsLvxReader final : public Lds {
public:
  LdsLvxReader(double publish_freq, const std::string& lvx_file_path);
  ~LdsLvxReader();

  LdsLvxReader(const LdsLvxReader &) = delete;
  LdsLvxReader &operator=(const LdsLvxReader &) = delete;

  /**
   * @brief Initializes the LVX file reader.
   * Opens the LVX file and prepares for reading.
   * @return true if initialization is successful, false otherwise.
   */
  bool Init();

  /**
   * @brief Starts the LVX file reading thread.
   * The thread will read data and push it into the system.
   */
  void StartRead();

  /**
   * @brief Prepares the LVX reader for exit.
   * Signals the reading thread to stop, closes the LVX file, and joins the thread.
   */
  void PrepareExit() override;

private:
  /**
   * @brief The main loop for reading data from the LVX file.
   * This function is executed in a separate thread.
   */
  void ReadLoop();

  std::string lvx_file_path_;
  std::atomic<bool> stop_flag_{false};
  std::thread read_thread_;
  std::mutex sdk_mutex_; // To protect Livox SDK calls if they are not thread-safe

  // Placeholder for Livox SDK specific file handle or context
  // void* lvx_sdk_handle_ = nullptr; 
  // For example: LivoxLvxFileHandle lvx_file_handle_ = nullptr;
};

}  // namespace livox_ros

#endif // LIVOX_ROS_DRIVER_LDS_LVX_READER_H_

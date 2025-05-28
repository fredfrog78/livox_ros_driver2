//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
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

#include <iostream>
#include <chrono>
#include <vector>
#include <csignal>
#include <thread>

#include "include/livox_ros_driver2.h"
#include "include/ros_headers.h"
#include "driver_node.h"
#include "lddc.h"
#include "lds_lidar.h"
#include "lds_lvx_reader.h" // For LVX file replay

using namespace livox_ros;

#ifdef BUILDING_ROS1
int main(int argc, char **argv) {
  /** Ros related */
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::init(argc, argv, "livox_lidar_publisher");

  // ros::NodeHandle livox_node;
  livox_ros::DriverNode livox_node; // This is a wrapper around the actual node handle for ROS1

  DRIVER_INFO(livox_node, "Livox Ros Driver2 Version: %s", LIVOX_ROS_DRIVER2_VERSION_STRING);

  /** Init default system parameter */
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq  = 10.0; /* Hz */
  int output_type      = kOutputToRos;
  std::string frame_id = "livox_frame";
  bool lidar_bag = true; // Used in Lddc constructor for ROS1
  bool imu_bag   = false; // Used in Lddc constructor for ROS1

  livox_node.GetNode().getParam("xfer_format", xfer_format);
  livox_node.GetNode().getParam("multi_topic", multi_topic);
  livox_node.GetNode().getParam("data_src", data_src);
  livox_node.GetNode().getParam("publish_freq", publish_freq);
  livox_node.GetNode().getParam("output_data_type", output_type);
  livox_node.GetNode().getParam("frame_id", frame_id);
  livox_node.GetNode().getParam("enable_lidar_bag", lidar_bag);
  livox_node.GetNode().getParam("enable_imu_bag", imu_bag);

  // Using DRIVER_INFO for consistency, assuming livox_node can be used for logging
  DRIVER_INFO(livox_node, "data source:%u.", data_src);

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.5) {
    publish_freq = 0.5;
  }

  livox_node.future_ = livox_node.exit_signal_.get_future();

  /** Lidar data distribute control and lidar data source set */
  livox_node.lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_type,
                        publish_freq, frame_id, lidar_bag, imu_bag);
  livox_node.lddc_ptr_->SetRosNode(&livox_node); // Pass the wrapper

  if (data_src == kSourceRawLidar) {
    DRIVER_INFO(livox_node, "Data Source is raw lidar.");

    std::string user_config_path;
    livox_node.getParam("user_config_path", user_config_path);
    DRIVER_INFO(livox_node, "Config file : %s", user_config_path.c_str());

    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
    livox_node.lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

    if ((read_lidar->InitLdsLidar(user_config_path))) {
      DRIVER_INFO(livox_node, "Init lds lidar successfully!");
    } else {
      DRIVER_ERROR(livox_node, "Init lds lidar failed!");
      return -1; 
    }
  } else if (data_src == kSourceLvxFile) {
    DRIVER_INFO(livox_node, "Data Source is LVX file.");
    std::string lvx_file_path;
    if (!livox_node.getParam("cmdline_file_path", lvx_file_path) || lvx_file_path.empty()) {
        DRIVER_ERROR(livox_node, "LVX file path (param 'cmdline_file_path') not specified or empty for ROS1.");
        return -1; 
    }
    DRIVER_INFO(livox_node, "LVX file path: %s", lvx_file_path.c_str());

    LdsLvxReader* lvx_reader = new LdsLvxReader(publish_freq, lvx_file_path);
    if (lvx_reader->Init()) {
      livox_node.lddc_ptr_->RegisterLds(static_cast<Lds *>(lvx_reader));
      lvx_reader->StartRead(); 
      DRIVER_INFO(livox_node, "LVX file reader initialized and started.");
    } else {
      DRIVER_ERROR(livox_node, "Init LVX file reader failed for: %s", lvx_file_path.c_str());
      delete lvx_reader; 
      return -1; 
    }
  } else {
    DRIVER_ERROR(livox_node, "Invalid data src (%d), please check the launch file", data_src);
    return -1; 
  }

  livox_node.pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, &livox_node);
  livox_node.imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, &livox_node);
  
  ros::spin(); 
  
  return 0;
}

#elif defined BUILDING_ROS2
namespace livox_ros
{
DriverNode::DriverNode(const rclcpp::NodeOptions & node_options)
: Node("livox_driver_node", node_options)
{
  DRIVER_INFO(*this, "Livox Ros Driver2 Version: %s", LIVOX_ROS_DRIVER2_VERSION_STRING);

  /** Init default system parameter */
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar; // Default value
  double publish_freq = 10.0; /* Hz */
  int output_type = kOutputToRos;
  std::string frame_id = "livox_frame"; // Default value

  this->declare_parameter("xfer_format", xfer_format);
  this->declare_parameter("multi_topic", multi_topic);
  this->declare_parameter("data_src", data_src);
  this->declare_parameter("publish_freq", publish_freq);
  this->declare_parameter("output_data_type", output_type);
  this->declare_parameter("frame_id", frame_id);
  this->declare_parameter("user_config_path", "path_default");
  this->declare_parameter("cmdline_input_bd_code", "000000000000001");
  this->declare_parameter("lvx_file_path", "/home/livox/livox_test.lvx");

  this->get_parameter("xfer_format", xfer_format);
  this->get_parameter("multi_topic", multi_topic);
  this->get_parameter("data_src", data_src);
  this->get_parameter("publish_freq", publish_freq);
  this->get_parameter("output_data_type", output_type);
  this->get_parameter("frame_id", frame_id);

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.5) {
    publish_freq = 0.5;
  }

  future_ = exit_signal_.get_future();

  /** Lidar data distribute control and lidar data source set */
  // Note: lidar_bag and imu_bag are not used in ROS2 Lddc constructor in existing code
  lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_type, publish_freq, frame_id);
  lddc_ptr_->SetRosNode(this);

  if (data_src == kSourceRawLidar) {
    DRIVER_INFO(*this, "Data Source is raw lidar.");

    std::string user_config_path;
    this->get_parameter("user_config_path", user_config_path);
    DRIVER_INFO(*this, "Config file : %s", user_config_path.c_str());

    // cmdline_input_bd_code is specific to live lidar, so get it here
    // std::string cmdline_bd_code; 
    // this->get_parameter("cmdline_input_bd_code", cmdline_bd_code);

    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
    lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

    if ((read_lidar->InitLdsLidar(user_config_path))) {
      DRIVER_INFO(*this, "Init lds lidar success!");
    } else {
      DRIVER_ERROR(*this, "Init lds lidar fail!");
      // Consider rclcpp::shutdown(); or throw an exception to stop node.
    }
  } else if (data_src == kSourceLvxFile) {
    DRIVER_INFO(*this, "Data Source is LVX file.");
    std::string lvx_file_path;
    this->get_parameter("lvx_file_path", lvx_file_path); 
    
    if (lvx_file_path.empty() || lvx_file_path == "/home/livox/livox_test.lvx") {
        DRIVER_WARN(*this, "LVX file path is default or empty ('%s'). Ensure 'lvx_file_path' parameter is correctly set in the launch file if this is not intended.", lvx_file_path.c_str());
    }
    DRIVER_INFO(*this, "LVX file path: %s", lvx_file_path.c_str());

    LdsLvxReader* lvx_reader = new LdsLvxReader(publish_freq, lvx_file_path);
    if (lvx_reader->Init()) {
      lddc_ptr_->RegisterLds(static_cast<Lds*>(lvx_reader));
      lvx_reader->StartRead(); 
      DRIVER_INFO(*this, "LVX file reader initialized and started.");
    } else {
      DRIVER_ERROR(*this, "Init LVX file reader failed for: %s", lvx_file_path.c_str());
      delete lvx_reader; 
      // Consider rclcpp::shutdown(); or throw an exception to stop node.
    }
  } else {
    DRIVER_ERROR(*this, "Invalid data src (%d), please check the launch file", data_src);
    // Consider rclcpp::shutdown(); or throw an exception to stop node.
  }

  pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, this);
  imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, this);
}

}  // namespace livox_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(livox_ros::DriverNode)

#endif  // defined BUILDING_ROS2


void DriverNode::PointCloudDataPollThread()
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(1)); 
  do {
    lddc_ptr_->DistributePointCloudData(); 
    status = future_.wait_for(std::chrono::microseconds(0)); 
  } while (status == std::future_status::timeout);
}

void DriverNode::ImuDataPollThread()
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(1)); 
  do {
    lddc_ptr_->DistributeImuData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}

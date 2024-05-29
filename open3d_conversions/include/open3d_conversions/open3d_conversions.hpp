// Copyright 2020 Autonomous Robots Lab, University of Nevada, Reno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef OPEN3D_CONVERSIONS__OPEN3D_CONVERSIONS_HPP_
#define OPEN3D_CONVERSIONS__OPEN3D_CONVERSIONS_HPP_

// Eigen
#include <Eigen/Dense>

// Open3D
#include <open3d/Open3D.h>

// C++
#include <string>

// ROS2
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "open3d_conversions/visibility_control.h"

namespace open3d_conversions
{
/**
 * @brief Copy data from a open3d::geometry::PointCloud to a
 * sensor_msgs::msg::PointCloud2. The header field of the output
 * pointcloud is not modified, and must be filled out separately
 *
 * @param pointcloud Reference to the open3d geometry PointCloud
 * @param ros_pc2 Reference to the sensor_msgs PointCloud2
 */
void open3dToRos(
  const open3d::geometry::PointCloud & pointcloud,
  sensor_msgs::msg::PointCloud2 & ros_pc2);

/**
 * @brief Copy data from a open3d::geometry::Image to a
 * sensor_msgs::msg::Image. The header field of the output
 * image is not modified, and must be filled out separately
 *
 * @param image Reference to the open3d geometry Image
 * @param ros_img Reference to the sensor_msgs Image
 * @param encoding The encoding of the image to use in ROS format
 */
void open3dToRos(
  const open3d::geometry::Image & o3d_img,
  sensor_msgs::msg::Image & ros_img,
  std::string encoding);

/**
 * @brief Copy data from a open3d::camera::PinholeCameraIntrinsic
 * to a sensor_msgs::msg::CameraInfo. The header field of the output
 * camera info is not modified, and must be filled out separately
 *
 * @param intrinsic Reference to the camera intrinsic in open3d format
 * @param ros_img Reference to the sensor_msgs CameraInfo
 *
 * @note Open3D does not record distortion parameters for its images, so
 * the sensor_msgs Image be labelled as "plumb_bob" with D values of 0
 */
void open3dToRos(
  const open3d::camera::PinholeCameraIntrinsic & intrinsic,
  sensor_msgs::msg::CameraInfo & camera_info);

/**
 * @brief Copy data from a sensor_msgs::msg::PointCloud2 to a
 * open3d::geometry::PointCloud
 *
 * @param ros_pc2 Reference to the sensor_msgs PointCloud2
 * @param o3d_pc Reference to the open3d geometry PointCloud
 * @param skip_colors If true, only xyz fields will be copied
 */
void rosToOpen3d(
  const sensor_msgs::msg::PointCloud2::SharedPtr & ros_pc2,
  open3d::geometry::PointCloud & o3d_pc,
  bool skip_colors = false);

/**
 * @brief Copy data from a sensor_msgs::msg::Image to a
 * open3d::geometry::Image
 *
 * @param ros_img Reference to the sensor_msgs Image
 * @param o3d_img Reference to the open3d geometry Image
 */
void rosToOpen3d(
  const sensor_msgs::msg::Image & ros_img,
  open3d::geometry::Image & o3d_img);

/**
 * @brief Copy data from a sensor_msgs::msg::CameraInfo
 * to a open3d::camera::PinholeCameraIntrinsic
 *
 * @param ros_img Reference to the sensor_msgs CameraInfo to populate
 * @param intrinsic Reference to the open3d PinholeCameraIntrinsic
 */
void rosToOpen3d(
  const sensor_msgs::msg::CameraInfo & camera_info,
  open3d::camera::PinholeCameraIntrinsic & intrinsic);

/**
 * @brief Move data from a open3d::geometry::Image to a
 * sensor_msgs::msg::Image. The header field of the output
 * image is not modified, and must be filled out separately
 *
 * @param o3d_img Reference to the open3d geometry Image
 * @param ros_img Reference to the sensor_msgs Image
 * @param encoding The encoding of the image to use in ROS format
 *
 * @note The Open3D image will be in a well defined state after this
 * operation with width = height = 0, and an empty data vector
 */
void moveOpen3dToRos(
  open3d::geometry::Image && o3d_img,
  sensor_msgs::msg::Image & ros_img,
  std::string encoding);

/**
 * @brief Move data from a sensor_msgs::msg::Image to a
 * open3d::geometry::Image
 *
 * @param ros_img Reference to the sensor_msgs Image
 * @param o3d_img Reference to the open3d geometry Image
 *
 * @note The sensor_msgs Image will be a well defined state after this
 * operation with width = height = 0, and an empty data vector
 */
void moveRosToOpen3d(
  sensor_msgs::msg::Image && ros_img,
  open3d::geometry::Image & o3d_img);

}  // namespace open3d_conversions

#endif  // OPEN3D_CONVERSIONS__OPEN3D_CONVERSIONS_HPP_

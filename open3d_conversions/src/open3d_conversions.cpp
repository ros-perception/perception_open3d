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

// C++
#include <memory>
#include <string>
#include <sstream>

#include <rcpputils/endian.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "open3d_conversions/open3d_conversions.hpp"

// Verify that an encoding makes sense with a given image
static void checkEncodingValidity(
  const std::string & encoding,
  const open3d::geometry::Image & o3d_img)
{
  const int expected_num_channels = sensor_msgs::image_encodings::numChannels(encoding);
  const int expected_bytes_per_channel = sensor_msgs::image_encodings::bitDepth(encoding) / 8;

  if (expected_num_channels != o3d_img.num_of_channels_) {
    std::stringstream ss;
    ss << "Mismatch between Open3D image encoding and desired embedded encoding"
       << "You asked for \"" << encoding << "\" which has " << expected_num_channels
       << " channels but the provided image had " << o3d_img.num_of_channels_ << " channels";
    throw std::runtime_error(ss.str());
  }

  if (expected_bytes_per_channel != o3d_img.bytes_per_channel_) {
    std::stringstream ss;
    ss << "Mismatch between Open3D image encoding and desired embedded encoding"
       << "You asked for \"" << encoding << "\" which has " << expected_bytes_per_channel
       << " bytes per channel but the provided image had " << o3d_img.bytes_per_channel_
       << " bytes per channel";
    throw std::runtime_error(ss.str());
  }
}

namespace open3d_conversions
{
void open3dToRos(
  const open3d::geometry::PointCloud & pointcloud,
  sensor_msgs::msg::PointCloud2 & ros_pc2)
{
  sensor_msgs::PointCloud2Modifier modifier(ros_pc2);
  if (pointcloud.HasColors()) {
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  } else {
    modifier.setPointCloud2FieldsByString(1, "xyz");
  }
  modifier.resize(pointcloud.points_.size());
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_pc2, "z");
  if (pointcloud.HasColors()) {
    sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_r(ros_pc2, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_g(ros_pc2, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_b(ros_pc2, "b");
    for (size_t i = 0; i < pointcloud.points_.size(); i++, ++ros_pc2_x,
      ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g,
      ++ros_pc2_b)
    {
      const Eigen::Vector3d & point = pointcloud.points_[i];
      const Eigen::Vector3d & color = pointcloud.colors_[i];
      *ros_pc2_x = point(0);
      *ros_pc2_y = point(1);
      *ros_pc2_z = point(2);
      *ros_pc2_r = static_cast<int>(255 * color(0));
      *ros_pc2_g = static_cast<int>(255 * color(1));
      *ros_pc2_b = static_cast<int>(255 * color(2));
    }
  } else {
    for (size_t i = 0; i < pointcloud.points_.size();
      i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
    {
      const Eigen::Vector3d & point = pointcloud.points_[i];
      *ros_pc2_x = point(0);
      *ros_pc2_y = point(1);
      *ros_pc2_z = point(2);
    }
  }
}

void open3dToRos(
  const open3d::geometry::Image & o3d_img,
  sensor_msgs::msg::Image & ros_img,
  std::string encoding)
{
  checkEncodingValidity(encoding, o3d_img);
  ros_img.encoding = encoding;
  ros_img.height = o3d_img.height_;
  ros_img.width = o3d_img.width_;
  ros_img.step = o3d_img.BytesPerLine();
  ros_img.data = o3d_img.data_;
  ros_img.is_bigendian = rcpputils::endian::native == rcpputils::endian::big;
}

void open3dToRos(
  const open3d::camera::PinholeCameraIntrinsic & intrinsic,
  sensor_msgs::msg::CameraInfo & camera_info)
{
  // Intrinsic matrix is 3x3 row-major
  std::fill(camera_info.k.begin(), camera_info.k.end(), 0.0);
  std::fill(camera_info.p.begin(), camera_info.p.end(), 0.0);
  for (std::size_t i = 0; i < 9; i++) {
    camera_info.k[i] = intrinsic.intrinsic_matrix_(i / 3, i % 3);
  }
  // Assuming image from monocular camera
  for (std::size_t i = 0; i < 12; i++) {
    if (i % 4 < 3) {
      camera_info.p[i] = intrinsic.intrinsic_matrix_(i / 4, i % 4);
    }
  }

  // Open3d intrinsics do not contain distortion information
  camera_info.distortion_model = "plumb_bob";
  camera_info.d.resize(5);
  std::fill(camera_info.d.begin(), camera_info.d.end(), 0.0);
}

void rosToOpen3d(
  const sensor_msgs::msg::PointCloud2::SharedPtr & ros_pc2,
  open3d::geometry::PointCloud & o3d_pc, bool skip_colors)
{
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_x(*ros_pc2, "x");
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_y(*ros_pc2, "y");
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_z(*ros_pc2, "z");
  o3d_pc.points_.reserve(ros_pc2->height * ros_pc2->width);
  if (ros_pc2->fields.size() == 3 || skip_colors == true) {
    for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
      ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
    {
      o3d_pc.points_.push_back(
        Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
    }
  } else {
    o3d_pc.colors_.reserve(ros_pc2->height * ros_pc2->width);
    if (ros_pc2->fields[3].name == "rgb") {
      sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_r(*ros_pc2, "r");
      sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_g(*ros_pc2, "g");
      sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_b(*ros_pc2, "b");

      for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_x,
        ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b)
      {
        o3d_pc.points_.push_back(
          Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
        o3d_pc.colors_.push_back(
          Eigen::Vector3d(
            (static_cast<int>(*ros_pc2_r)) / 255.0,
            (static_cast<int>(*ros_pc2_g)) / 255.0,
            (static_cast<int>(*ros_pc2_b)) / 255.0));
      }
    } else if (ros_pc2->fields[3].name == "intensity") {
      sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_i(*ros_pc2,
        "intensity");
      for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
        ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_i)
      {
        o3d_pc.points_.push_back(
          Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
        o3d_pc.colors_.push_back(
          Eigen::Vector3d(*ros_pc2_i, *ros_pc2_i, *ros_pc2_i));
      }
    }
  }
}

void rosToOpen3d(
  const sensor_msgs::msg::Image & ros_img,
  open3d::geometry::Image & o3d_img)
{
  const int num_channels = sensor_msgs::image_encodings::numChannels(ros_img.encoding);
  const int bytes_per_channel = sensor_msgs::image_encodings::bitDepth(ros_img.encoding) / 8;

  o3d_img.Prepare(
    ros_img.width,
    ros_img.height,
    num_channels,
    bytes_per_channel
  );

  // Open3D requires the image to be in native endianness
  if (ros_img.is_bigendian && rcpputils::endian::native == rcpputils::endian::big ||
    !ros_img.is_bigendian && rcpputils::endian::native == rcpputils::endian::little ||
    bytes_per_channel == 1)
  {
    o3d_img.data_ = ros_img.data;
    return;
  }

  // Otherwise, flip the bytes of each channel entry
  o3d_img.data_.clear();
  o3d_img.data_.reserve(ros_img.data.size());
  const std::ptrdiff_t data_step = bytes_per_channel * num_channels;
  for (auto data_ptr = ros_img.data.begin(); data_ptr != ros_img.data.end();
    std::advance(data_ptr, data_step))
  {
    for (std::ptrdiff_t channel_idx = 0; channel_idx < num_channels; ++channel_idx) {
      const std::ptrdiff_t channel_start = channel_idx * bytes_per_channel;
      for (std::ptrdiff_t byte_idx = 0; byte_idx < bytes_per_channel; ++byte_idx) {
        const std::ptrdiff_t reverse_byte_idx = bytes_per_channel - byte_idx - 1;
        o3d_img.data_.push_back(*(data_ptr + channel_start + reverse_byte_idx));
      }
    }
  }
}

void rosToOpen3d(
  const sensor_msgs::msg::CameraInfo & camera_info,
  open3d::camera::PinholeCameraIntrinsic & intrinsic)
{
  intrinsic.width_ = camera_info.width;
  intrinsic.height_ = camera_info.height;
  for (std::size_t row = 0; row < 3; row++) {
    for (std::size_t col = 0; col < 3; col++) {
      intrinsic.intrinsic_matrix_(row, col) = camera_info.k[3 * row + col];
    }
  }
}

void moveOpen3dToRos(
  open3d::geometry::Image && o3d_img,
  sensor_msgs::msg::Image & ros_img,
  std::string encoding)
{
  checkEncodingValidity(encoding, o3d_img);
  ros_img.encoding = encoding;
  ros_img.height = o3d_img.height_;
  ros_img.width = o3d_img.width_;
  ros_img.step = o3d_img.BytesPerLine();
  ros_img.is_bigendian = rcpputils::endian::native == rcpputils::endian::big;
  ros_img.data = std::move(o3d_img.data_);

  // Make sure to leave everything in a well defined state
  o3d_img.Clear();
}

void moveRosToOpen3d(
  sensor_msgs::msg::Image && ros_img,
  open3d::geometry::Image & o3d_img)
{
  // If the endianness does not match, we will need to perform a copy anyway
  if ((ros_img.is_bigendian && rcpputils::endian::native == rcpputils::endian::little ||
    !ros_img.is_bigendian && rcpputils::endian::native == rcpputils::endian::big) &&
    sensor_msgs::image_encodings::bitDepth(ros_img.encoding) > 8)
  {
    rosToOpen3d(ros_img, o3d_img);
    return;
  }

  o3d_img.Prepare(
    ros_img.width,
    ros_img.height,
    sensor_msgs::image_encodings::numChannels(ros_img.encoding),
    sensor_msgs::image_encodings::bitDepth(ros_img.encoding) / 8
  );
  o3d_img.data_ = std::move(ros_img.data);

  // Make sure to leave everything in a well defined state
  ros_img.data.clear();
  ros_img.step = 0;
  ros_img.height = 0;
  ros_img.width = 0;
  ros_img.encoding.clear();
}

}  // namespace open3d_conversions

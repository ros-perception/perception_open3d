// Copyright 2020 Autonomous Robots Lab, University of Nevada, Reno

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// C++
#include <memory>
#include <string>
#include <sstream>

#include <sensor_msgs/image_encodings.hpp>
#include "open3d_conversions/open3d_conversions.hpp"

// This is the best we have prior to C++20 std::endian::native
static bool isLittleEndian(){
  const int32_t value = 0x01;
  const std::byte * least_significant_address = 
    reinterpret_cast<const std::byte *>(&value);
  return (*least_significant_address == std::byte{0x01});
}

namespace open3d_conversions
{
void open3dToRos(
  const open3d::geometry::PointCloud & pointcloud,
  sensor_msgs::msg::PointCloud2 & ros_pc2, std::string frame_id)
{
  sensor_msgs::PointCloud2Modifier modifier(ros_pc2);
  if (pointcloud.HasColors()) {
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  } else {
    modifier.setPointCloud2FieldsByString(1, "xyz");
  }
  modifier.resize(pointcloud.points_.size());
  ros_pc2.header.frame_id = frame_id;
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
  std::string encoding,
  std::string frame_id)
{
  const int expected_num_channels = sensor_msgs::image_encodings::numChannels(encoding);
  const int expected_bytes_per_channel = sensor_msgs::image_encodings::bitDepth(encoding)/8;

  // Verify that the encoding makes sense with the given image
  if (expected_num_channels != o3d_img.num_of_channels_){
    std::stringstream ss;
    ss << "Mismatch between Open3D image encoding and desired embedded encoding"
       << "You asked for \"" << encoding << "\" which has " << expected_num_channels
       << "channels but the provided image had " << o3d_img.num_of_channels_ << " channels";
    throw std::runtime_error(ss.str());
  }

  if (expected_bytes_per_channel != o3d_img.bytes_per_channel_){
    std::stringstream ss;
    ss << "Mismatch between Open3D image encoding and desired embedded encoding"
       << "You asked for \"" << encoding << "\" which has " << expected_bytes_per_channel
       << "bytes per channel but the provided image had " << o3d_img.bytes_per_channel_ 
       << " bytes per channel";
    throw std::runtime_error(ss.str());
  }

  ros_img.encoding = encoding;
  ros_img.header.frame_id = frame_id;
  ros_img.height = o3d_img.height_;
  ros_img.width  = o3d_img.width_;
  ros_img.step   = o3d_img.BytesPerLine();
  ros_img.data   = o3d_img.data_;
  ros_img.is_bigendian = !isLittleEndian();
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
  open3d::geometry::Image o3d_img)
{
  o3d_img.Prepare(
    ros_img.width,
    ros_img.height,
    sensor_msgs::image_encodings::numChannels(ros_img.encoding),
    sensor_msgs::image_encodings::bitDepth(ros_img.encoding)/8
  );
  assert(o3d_img.data_.size() == ros_img.data.size());
  std::memcpy(o3d_img.data_.data(), ros_img.data.data(), ros_img.data.size());
}

}  // namespace open3d_conversions

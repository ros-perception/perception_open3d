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
  // Clear the input pointcloud to prepare it to be set
  std_msgs::msg::Header pc_header = ros_pc2.header;
  ros_pc2 = sensor_msgs::msg::PointCloud2{};
  ros_pc2.header = pc_header;

  // We define a lambda for new fields to avoid code repetition
  // This only works because all our fields have the same structure
  auto add_new_field = [&ros_pc2](const std::string & field_name) {
      sensor_msgs::msg::PointField & new_field = ros_pc2.fields.emplace_back();
      new_field.count = 1;
      new_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      new_field.name = field_name;
      new_field.offset = ros_pc2.point_step;
      ros_pc2.point_step += 4;
    };

  add_new_field("x");
  add_new_field("y");
  add_new_field("z");

  if (pointcloud.HasColors()) {
    // Note that the RGB field is typically packed as a single FLOAT32 value
    add_new_field("rgb");
  }

  if (pointcloud.HasNormals()) {
    add_new_field("normal_x");
    add_new_field("normal_y");
    add_new_field("normal_z");
  }

  // Set the parameters known from the open3d pointcloud definition
  ros_pc2.height = 1;
  ros_pc2.width = pointcloud.points_.size();
  ros_pc2.row_step = ros_pc2.width * ros_pc2.point_step;
  ros_pc2.is_bigendian = rcpputils::endian::native == rcpputils::endian::big;
  ros_pc2.is_dense = true;
  ros_pc2.data.resize(ros_pc2.row_step);

  // For clarity we set up this lambda to cleanly set the value and advance the pointer
  auto * ros_it = ros_pc2.data.data();
  auto set_next_value = [&ros_it](const auto & value) {
      std::memcpy(ros_it, &value, sizeof(value));
      std::advance(ros_it, sizeof(value));
    };

  for (std::size_t idx = 0; idx < pointcloud.points_.size(); ++idx) {
    const Eigen::Vector3f point_xyz = pointcloud.points_[idx].cast<float>();
    if (point_xyz.hasNaN()) {ros_pc2.is_dense = false;}
    set_next_value(point_xyz.x());
    set_next_value(point_xyz.y());
    set_next_value(point_xyz.z());

    if (pointcloud.HasColors()) {
      const Eigen::Vector3d & point_rgb = pointcloud.colors_[idx];
      const uint8_t r = static_cast<uint8_t>(255 * point_rgb(0));
      const uint8_t g = static_cast<uint8_t>(255 * point_rgb(1));
      const uint8_t b = static_cast<uint8_t>(255 * point_rgb(2));
      const uint32_t rgb = (rcpputils::endian::native == rcpputils::endian::big) ?
        (b << 24 | g << 16 | r << 8) :
        (r << 16 | g << 8 | b);
      set_next_value(rgb);
    }

    if (pointcloud.HasNormals()) {
      const Eigen::Vector3f point_normal = pointcloud.normals_[idx].cast<float>();
      set_next_value(point_normal.x());
      set_next_value(point_normal.y());
      set_next_value(point_normal.z());
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
  const std::size_t num_points = ros_pc2->height * ros_pc2->width;

  // Standard XYZ coordinate
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_x(*ros_pc2, "x");
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_y(*ros_pc2, "y");
  sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_z(*ros_pc2, "z");
  o3d_pc.points_.reserve(num_points);
  for (std::size_t i = 0; i < num_points; ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
    o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
  }

  auto hasField = [&ros_pc2](const std::string& name) {
    return std::any_of(ros_pc2->fields.begin(), ros_pc2->fields.end(), [&name](const sensor_msgs::msg::PointField& field){return field.name == name;});
  };

  // Add normals if the input cloud has normals
  if (hasField("normal_x") && hasField("normal_y") && hasField("normal_z")) {
    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_nx(*ros_pc2, "normal_x");
    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_ny(*ros_pc2, "normal_y");
    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_nz(*ros_pc2, "normal_z");
    o3d_pc.normals_.reserve(num_points);
    for (std::size_t i = 0; i < num_points; ++i, ++ros_pc2_nx, ++ros_pc2_ny, ++ros_pc2_nz) {
      o3d_pc.normals_.push_back(Eigen::Vector3d(*ros_pc2_nx, *ros_pc2_ny, *ros_pc2_nz));
    }
  }

  // Add colors if the input cloud has colors
  if (hasField("rgb") || hasField("rgba")) {
    sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_r(*ros_pc2, "r");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_g(*ros_pc2, "g");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_b(*ros_pc2, "b");
    o3d_pc.colors_.reserve(num_points);
    for (std::size_t i = 0; i < num_points; ++i, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b) {
      o3d_pc.colors_.push_back(Eigen::Vector3d(
            (static_cast<int>(*ros_pc2_r)) / 255.0,
            (static_cast<int>(*ros_pc2_g)) / 255.0,
            (static_cast<int>(*ros_pc2_b)) / 255.0));
    }
  }   
  // Add intensity as a color if the input cloud has intesity
  else if (hasField("intensity")) {
    sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_i(*ros_pc2, "intensity");
    o3d_pc.colors_.reserve(num_points);
    for (std::size_t i = 0; i < num_points; ++i, ++ros_pc2_i) {
      o3d_pc.colors_.push_back(Eigen::Vector3d(*ros_pc2_i, *ros_pc2_i, *ros_pc2_i));
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

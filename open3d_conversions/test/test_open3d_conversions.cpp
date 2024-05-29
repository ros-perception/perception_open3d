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

// GTest
#include <gtest/gtest.h>

// Open3D
#include <open3d/Open3D.h>

// C++
#include <memory>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rcpputils/endian.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// open3d_conversions
#include "open3d_conversions/open3d_conversions.hpp"

TEST(ConversionFunctions, open3dToRos2_uncoloredPointcloud) {
  open3d::geometry::PointCloud o3d_pc;
  for (int i = 0; i < 5; ++i) {
    o3d_pc.points_.push_back(Eigen::Vector3d(0.5 * i, i * i, 10.5 * i));
  }
  sensor_msgs::msg::PointCloud2 ros_pc2;
  open3d_conversions::open3dToRos(o3d_pc, ros_pc2);
  EXPECT_EQ(ros_pc2.height * ros_pc2.width, o3d_pc.points_.size());
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_pc2, "z");
  for (int i = 0; i < 5; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
    EXPECT_EQ(*ros_pc2_x, 0.5 * i);
    EXPECT_EQ(*ros_pc2_y, i * i);
    EXPECT_EQ(*ros_pc2_z, 10.5 * i);
  }
}

TEST(ConversionFunctions, open3dToRos2_coloredPointcloud) {
  open3d::geometry::PointCloud o3d_pc;
  for (int i = 0; i < 5; ++i) {
    o3d_pc.points_.push_back(Eigen::Vector3d(0.5 * i, i * i, 10.5 * i));
    o3d_pc.colors_.push_back(
      Eigen::Vector3d(2 * i / 255.0, 5 * i / 255.0, 10 * i / 255.0));
  }
  sensor_msgs::msg::PointCloud2 ros_pc2;
  open3d_conversions::open3dToRos(o3d_pc, ros_pc2);
  EXPECT_EQ(ros_pc2.height * ros_pc2.width, o3d_pc.points_.size());
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_pc2, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_r(ros_pc2, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_g(ros_pc2, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_b(ros_pc2, "b");
  for (int i = 0; i < 5; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z,
    ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b)
  {
    EXPECT_EQ(*ros_pc2_x, 0.5 * i);
    EXPECT_EQ(*ros_pc2_y, i * i);
    EXPECT_EQ(*ros_pc2_z, 10.5 * i);
    EXPECT_EQ(*ros_pc2_r, 2 * i);
    EXPECT_EQ(*ros_pc2_g, 5 * i);
    EXPECT_EQ(*ros_pc2_b, 10 * i);
  }
}

TEST(ConversionFunctions, rosToOpen3d_uncoloredPointcloud) {
  sensor_msgs::msg::PointCloud2 ros_pc2;
  ros_pc2.header.frame_id = "ros";
  ros_pc2.height = 1;
  ros_pc2.width = 5;
  ros_pc2.is_bigendian = false;
  ros_pc2.is_dense = true;
  sensor_msgs::PointCloud2Modifier modifier(ros_pc2);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(5 * 1);
  sensor_msgs::PointCloud2Iterator<float> mod_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> mod_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> mod_z(ros_pc2, "z");

  for (int i = 0; i < 5; ++i, ++mod_x, ++mod_y, ++mod_z) {
    *mod_x = 0.5 * i;
    *mod_y = i * i;
    *mod_z = 10.5 * i;
  }

  const sensor_msgs::msg::PointCloud2::SharedPtr & ros_pc2_ptr =
    std::make_shared<sensor_msgs::msg::PointCloud2>(ros_pc2);
  open3d::geometry::PointCloud o3d_pc;
  open3d_conversions::rosToOpen3d(ros_pc2_ptr, o3d_pc);
  EXPECT_EQ(ros_pc2_ptr->height * ros_pc2_ptr->width, o3d_pc.points_.size());
  EXPECT_EQ(o3d_pc.HasColors(), false);
  for (unsigned int i = 0; i < 5; i++) {
    const Eigen::Vector3d & point = o3d_pc.points_[i];
    EXPECT_EQ(point(0), 0.5 * i);
    EXPECT_EQ(point(1), i * i);
    EXPECT_EQ(point(2), 10.5 * i);
  }
}

TEST(ConversionFunctions, rosToOpen3d_coloredPointcloud) {
  sensor_msgs::msg::PointCloud2 ros_pc2;
  ros_pc2.header.frame_id = "ros";
  ros_pc2.height = 1;
  ros_pc2.width = 5;
  ros_pc2.is_bigendian = false;
  ros_pc2.is_dense = true;
  sensor_msgs::PointCloud2Modifier modifier(ros_pc2);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(5 * 1);

  sensor_msgs::PointCloud2Iterator<float> mod_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> mod_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> mod_z(ros_pc2, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> mod_r(ros_pc2, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> mod_g(ros_pc2, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> mod_b(ros_pc2, "b");

  for (int i = 0; i < 5;
    ++i, ++mod_x, ++mod_y, ++mod_z, ++mod_r, ++mod_g, ++mod_b)
  {
    *mod_x = 0.5 * i;
    *mod_y = i * i;
    *mod_z = 10.5 * i;
    *mod_r = 2 * i;
    *mod_g = 5 * i;
    *mod_b = 10 * i;
  }

  const sensor_msgs::msg::PointCloud2::SharedPtr & ros_pc2_ptr =
    std::make_shared<sensor_msgs::msg::PointCloud2>(ros_pc2);
  open3d::geometry::PointCloud o3d_pc;
  open3d_conversions::rosToOpen3d(ros_pc2_ptr, o3d_pc);
  EXPECT_EQ(ros_pc2_ptr->height * ros_pc2_ptr->width, o3d_pc.points_.size());
  EXPECT_EQ(o3d_pc.HasColors(), true);
  for (unsigned int i = 0; i < 5; i++) {
    const Eigen::Vector3d & point = o3d_pc.points_[i];
    EXPECT_EQ(point(0), 0.5 * i);
    EXPECT_EQ(point(1), i * i);
    EXPECT_EQ(point(2), 10.5 * i);
    const Eigen::Vector3d & color = o3d_pc.colors_[i];
    EXPECT_EQ(color(0), 2 * i / 255.0);
    EXPECT_EQ(color(1), 5 * i / 255.0);
    EXPECT_EQ(color(2), 10 * i / 255.0);
  }
}

TEST(ConversionFunctions, open3dToRos2_wellFormattedImage) {
  open3d::geometry::Image o3d_image;
  o3d_image.bytes_per_channel_ = 2;
  o3d_image.height_ = 4;
  o3d_image.width_ = 5;
  o3d_image.num_of_channels_ = 3;

  for (std::size_t row = 0; row < o3d_image.height_; ++row) {
    for (std::size_t col = 0; col < o3d_image.width_; ++col) {
      // R
      o3d_image.data_.push_back(static_cast<uint8_t>(row));
      o3d_image.data_.push_back(static_cast<uint8_t>(col));
      // G
      o3d_image.data_.push_back(static_cast<uint8_t>(row * 5));
      o3d_image.data_.push_back(static_cast<uint8_t>(col * 3));
      // B
      o3d_image.data_.push_back(static_cast<uint8_t>(row + 2));
      o3d_image.data_.push_back(static_cast<uint8_t>(col + 6));
    }
  }

  sensor_msgs::msg::Image ros_image;
  open3d_conversions::open3dToRos(o3d_image, ros_image, "rgb16");

  EXPECT_EQ(
    o3d_image.bytes_per_channel_,
    sensor_msgs::image_encodings::bitDepth(ros_image.encoding) / 8
  );
  EXPECT_EQ(
    o3d_image.num_of_channels_,
    sensor_msgs::image_encodings::numChannels(ros_image.encoding)
  );
  EXPECT_EQ(o3d_image.width_, ros_image.width);
  EXPECT_EQ(o3d_image.height_, ros_image.height);
  EXPECT_EQ(o3d_image.BytesPerLine(), ros_image.step);
  EXPECT_EQ(ros_image.is_bigendian, rcpputils::endian::native == rcpputils::endian::big);
  EXPECT_EQ(ros_image.encoding, "rgb16");

  for (std::size_t row = 0; row < ros_image.height; ++row) {
    for (std::size_t col = 0; col < ros_image.width; ++col) {
      const std::size_t pixel_start_idx = (row * ros_image.width + col) *
        o3d_image.num_of_channels_ * o3d_image.bytes_per_channel_;

      const uint8_t R1 = ros_image.data.at(pixel_start_idx + 0);
      const uint8_t R2 = ros_image.data.at(pixel_start_idx + 1);
      const uint8_t G1 = ros_image.data.at(pixel_start_idx + 2);
      const uint8_t G2 = ros_image.data.at(pixel_start_idx + 3);
      const uint8_t B1 = ros_image.data.at(pixel_start_idx + 4);
      const uint8_t B2 = ros_image.data.at(pixel_start_idx + 5);

      EXPECT_EQ(R1, static_cast<uint8_t>(row));
      EXPECT_EQ(R2, static_cast<uint8_t>(col));
      EXPECT_EQ(G1, static_cast<uint8_t>(row * 5));
      EXPECT_EQ(G2, static_cast<uint8_t>(col * 3));
      EXPECT_EQ(B1, static_cast<uint8_t>(row + 2));
      EXPECT_EQ(B2, static_cast<uint8_t>(col + 6));
    }
  }
}

TEST(ConversionFunctions, open3dToRos2_moveWellFormattedImage) {
  open3d::geometry::Image o3d_image;
  o3d_image.bytes_per_channel_ = 2;
  o3d_image.height_ = 4;
  o3d_image.width_ = 5;
  o3d_image.num_of_channels_ = 3;

  for (std::size_t row = 0; row < o3d_image.height_; ++row) {
    for (std::size_t col = 0; col < o3d_image.width_; ++col) {
      // R
      o3d_image.data_.push_back(static_cast<uint8_t>(row));
      o3d_image.data_.push_back(static_cast<uint8_t>(col));
      // G
      o3d_image.data_.push_back(static_cast<uint8_t>(row * 5));
      o3d_image.data_.push_back(static_cast<uint8_t>(col * 3));
      // B
      o3d_image.data_.push_back(static_cast<uint8_t>(row + 2));
      o3d_image.data_.push_back(static_cast<uint8_t>(col + 6));
    }
  }

  sensor_msgs::msg::Image ros_image;
  open3d_conversions::moveOpen3dToRos(std::move(o3d_image), ros_image, "rgb16");

  // Moved-from image should be cleared
  EXPECT_EQ(o3d_image.width_, 0.0);
  EXPECT_EQ(o3d_image.width_, 0.0);
  EXPECT_EQ(o3d_image.bytes_per_channel_, 0);
  EXPECT_EQ(o3d_image.num_of_channels_, 0);
  EXPECT_TRUE(o3d_image.data_.empty());

  EXPECT_EQ(sensor_msgs::image_encodings::bitDepth(ros_image.encoding), 16);
  EXPECT_EQ(sensor_msgs::image_encodings::numChannels(ros_image.encoding), 3);
  EXPECT_EQ(ros_image.width, 5);
  EXPECT_EQ(ros_image.height, 4);
  EXPECT_EQ(ros_image.step, 30);
  EXPECT_EQ(ros_image.is_bigendian, rcpputils::endian::native == rcpputils::endian::big);
  EXPECT_EQ(ros_image.encoding, "rgb16");

  const std::size_t pixel_size = ros_image.step / ros_image.width;
  for (std::size_t row = 0; row < ros_image.height; ++row) {
    for (std::size_t col = 0; col < ros_image.width; ++col) {
      const std::size_t pixel_start_idx = (row * ros_image.width + col) * pixel_size;
      const uint8_t R1 = ros_image.data.at(pixel_start_idx + 0);
      const uint8_t R2 = ros_image.data.at(pixel_start_idx + 1);
      const uint8_t G1 = ros_image.data.at(pixel_start_idx + 2);
      const uint8_t G2 = ros_image.data.at(pixel_start_idx + 3);
      const uint8_t B1 = ros_image.data.at(pixel_start_idx + 4);
      const uint8_t B2 = ros_image.data.at(pixel_start_idx + 5);

      EXPECT_EQ(R1, static_cast<uint8_t>(row));
      EXPECT_EQ(R2, static_cast<uint8_t>(col));
      EXPECT_EQ(G1, static_cast<uint8_t>(row * 5));
      EXPECT_EQ(G2, static_cast<uint8_t>(col * 3));
      EXPECT_EQ(B1, static_cast<uint8_t>(row + 2));
      EXPECT_EQ(B2, static_cast<uint8_t>(col + 6));
    }
  }
}

TEST(ConversionFunctions, open3dToRos2_wronglyFormattedImage) {
  open3d::geometry::Image o3d_image;
  o3d_image.bytes_per_channel_ = 2;
  o3d_image.height_ = 4;
  o3d_image.width_ = 5;
  o3d_image.num_of_channels_ = 3;

  sensor_msgs::msg::Image ros_image;

  // Wrong number of channels
  EXPECT_THROW(
    open3d_conversions::open3dToRos(o3d_image, ros_image, "mono16"),
    std::runtime_error
  );

  // Wrong number of bytes per channel
  EXPECT_THROW(
    open3d_conversions::open3dToRos(o3d_image, ros_image, "rgb8"),
    std::runtime_error
  );
}

TEST(ConversionFunctions, open3dToRos2_moveWronglyFormattedImage) {
  open3d::geometry::Image o3d_image;
  o3d_image.bytes_per_channel_ = 2;
  o3d_image.height_ = 4;
  o3d_image.width_ = 5;
  o3d_image.num_of_channels_ = 3;
  o3d_image.data_.resize(o3d_image.BytesPerLine() * o3d_image.height_);

  sensor_msgs::msg::Image ros_image;

  // Wrong number of channels
  EXPECT_THROW(
    open3d_conversions::moveOpen3dToRos(std::move(o3d_image), ros_image, "mono16"),
    std::runtime_error
  );

  // The move should not have affected the image since the operation was not possible
  EXPECT_NE(o3d_image.data_.size(), 0);
  EXPECT_EQ(ros_image.data.size(), 0);

  // Wrong number of bytes per channel
  EXPECT_THROW(
    open3d_conversions::moveOpen3dToRos(std::move(o3d_image), ros_image, "rgb8"),
    std::runtime_error
  );

  EXPECT_NE(o3d_image.data_.size(), 0);
  EXPECT_EQ(ros_image.data.size(), 0);
}

TEST(ConversionFunctions, rosToOpen3d_wellFormattedImage) {
  sensor_msgs::msg::Image ros_image;
  ros_image.height = 4;
  ros_image.width = 5;
  ros_image.encoding = "16UC3";
  ros_image.is_bigendian = rcpputils::endian::native == rcpputils::endian::big;
  ros_image.step = ros_image.width * 3 * 2;

  for (std::size_t row = 0; row < ros_image.height; ++row) {
    for (std::size_t col = 0; col < ros_image.width; ++col) {
      // R
      ros_image.data.push_back(static_cast<uint8_t>(row));
      ros_image.data.push_back(static_cast<uint8_t>(col));
      // G
      ros_image.data.push_back(static_cast<uint8_t>(row * 5));
      ros_image.data.push_back(static_cast<uint8_t>(col * 3));
      // B
      ros_image.data.push_back(static_cast<uint8_t>(row + 2));
      ros_image.data.push_back(static_cast<uint8_t>(col + 6));
    }
  }

  open3d::geometry::Image o3d_image;
  open3d_conversions::rosToOpen3d(ros_image, o3d_image);

  EXPECT_EQ(
    o3d_image.bytes_per_channel_,
    sensor_msgs::image_encodings::bitDepth(ros_image.encoding) / 8
  );
  EXPECT_EQ(
    o3d_image.num_of_channels_,
    sensor_msgs::image_encodings::numChannels(ros_image.encoding)
  );
  EXPECT_EQ(o3d_image.width_, ros_image.width);
  EXPECT_EQ(o3d_image.height_, ros_image.height);
  EXPECT_EQ(o3d_image.BytesPerLine(), ros_image.step);

  for (std::size_t row = 0; row < o3d_image.height_; ++row) {
    for (std::size_t col = 0; col < o3d_image.width_; ++col) {
      const std::size_t pixel_start_idx = (row * o3d_image.width_ + col) *
        o3d_image.num_of_channels_ * o3d_image.bytes_per_channel_;

      const uint8_t R1 = o3d_image.data_.at(pixel_start_idx + 0);
      const uint8_t R2 = o3d_image.data_.at(pixel_start_idx + 1);
      const uint8_t G1 = o3d_image.data_.at(pixel_start_idx + 2);
      const uint8_t G2 = o3d_image.data_.at(pixel_start_idx + 3);
      const uint8_t B1 = o3d_image.data_.at(pixel_start_idx + 4);
      const uint8_t B2 = o3d_image.data_.at(pixel_start_idx + 5);

      EXPECT_EQ(R1, static_cast<uint8_t>(row));
      EXPECT_EQ(R2, static_cast<uint8_t>(col));
      EXPECT_EQ(G1, static_cast<uint8_t>(row * 5));
      EXPECT_EQ(G2, static_cast<uint8_t>(col * 3));
      EXPECT_EQ(B1, static_cast<uint8_t>(row + 2));
      EXPECT_EQ(B2, static_cast<uint8_t>(col + 6));
    }
  }
}

TEST(ConversionFunctions, rosToOpen3d_moveWellFormattedImage) {
  sensor_msgs::msg::Image ros_image;
  ros_image.height = 4;
  ros_image.width = 5;
  ros_image.encoding = "16UC3";
  ros_image.is_bigendian = rcpputils::endian::native == rcpputils::endian::big;
  ros_image.step = ros_image.width * 3 * 2;

  for (std::size_t row = 0; row < ros_image.height; ++row) {
    for (std::size_t col = 0; col < ros_image.width; ++col) {
      // R
      ros_image.data.push_back(static_cast<uint8_t>(row));
      ros_image.data.push_back(static_cast<uint8_t>(col));
      // G
      ros_image.data.push_back(static_cast<uint8_t>(row * 5));
      ros_image.data.push_back(static_cast<uint8_t>(col * 3));
      // B
      ros_image.data.push_back(static_cast<uint8_t>(row + 2));
      ros_image.data.push_back(static_cast<uint8_t>(col + 6));
    }
  }

  open3d::geometry::Image o3d_image;
  open3d_conversions::moveRosToOpen3d(std::move(ros_image), o3d_image);

  // Moved-from image should be cleared
  EXPECT_EQ(ros_image.width, 0);
  EXPECT_EQ(ros_image.height, 0);
  EXPECT_EQ(ros_image.step, 0);
  EXPECT_TRUE(ros_image.data.empty());
  EXPECT_TRUE(ros_image.encoding.empty());

  EXPECT_EQ(o3d_image.bytes_per_channel_, 2);
  EXPECT_EQ(o3d_image.num_of_channels_, 3);
  EXPECT_EQ(o3d_image.width_, 5);
  EXPECT_EQ(o3d_image.height_, 4);
  EXPECT_EQ(o3d_image.BytesPerLine(), 30);

  for (std::size_t row = 0; row < o3d_image.height_; ++row) {
    for (std::size_t col = 0; col < o3d_image.width_; ++col) {
      const std::size_t pixel_start_idx = (row * o3d_image.width_ + col) *
        o3d_image.num_of_channels_ * o3d_image.bytes_per_channel_;

      const uint8_t R1 = o3d_image.data_.at(pixel_start_idx + 0);
      const uint8_t R2 = o3d_image.data_.at(pixel_start_idx + 1);
      const uint8_t G1 = o3d_image.data_.at(pixel_start_idx + 2);
      const uint8_t G2 = o3d_image.data_.at(pixel_start_idx + 3);
      const uint8_t B1 = o3d_image.data_.at(pixel_start_idx + 4);
      const uint8_t B2 = o3d_image.data_.at(pixel_start_idx + 5);

      EXPECT_EQ(R1, static_cast<uint8_t>(row));
      EXPECT_EQ(R2, static_cast<uint8_t>(col));
      EXPECT_EQ(G1, static_cast<uint8_t>(row * 5));
      EXPECT_EQ(G2, static_cast<uint8_t>(col * 3));
      EXPECT_EQ(B1, static_cast<uint8_t>(row + 2));
      EXPECT_EQ(B2, static_cast<uint8_t>(col + 6));
    }
  }
}

TEST(ConversionFunctions, rosToOpen3d_differentEndianness) {
  sensor_msgs::msg::Image ros_image;
  ros_image.height = 4;
  ros_image.width = 5;
  ros_image.encoding = "bgr16";
  ros_image.is_bigendian = rcpputils::endian::native != rcpputils::endian::big;
  ros_image.step = ros_image.width * 3 * 2;

  for (std::size_t row = 0; row < ros_image.height; ++row) {
    for (std::size_t col = 0; col < ros_image.width; ++col) {
      // R
      ros_image.data.push_back(static_cast<uint8_t>(row));
      ros_image.data.push_back(static_cast<uint8_t>(col));
      // G
      ros_image.data.push_back(static_cast<uint8_t>(row * 5));
      ros_image.data.push_back(static_cast<uint8_t>(col * 3));
      // B
      ros_image.data.push_back(static_cast<uint8_t>(row + 2));
      ros_image.data.push_back(static_cast<uint8_t>(col + 6));
    }
  }

  open3d::geometry::Image o3d_image;
  open3d_conversions::rosToOpen3d(ros_image, o3d_image);

  EXPECT_EQ(
    o3d_image.bytes_per_channel_,
    sensor_msgs::image_encodings::bitDepth(ros_image.encoding) / 8
  );
  EXPECT_EQ(
    o3d_image.num_of_channels_,
    sensor_msgs::image_encodings::numChannels(ros_image.encoding)
  );
  EXPECT_EQ(o3d_image.width_, ros_image.width);
  EXPECT_EQ(o3d_image.height_, ros_image.height);
  EXPECT_EQ(o3d_image.BytesPerLine(), ros_image.step);

  for (std::size_t row = 0; row < o3d_image.height_; ++row) {
    for (std::size_t col = 0; col < o3d_image.width_; ++col) {
      const std::size_t pixel_start_idx = (row * o3d_image.width_ + col) *
        o3d_image.num_of_channels_ * o3d_image.bytes_per_channel_;

      const uint8_t R1 = o3d_image.data_.at(pixel_start_idx + 0);
      const uint8_t R2 = o3d_image.data_.at(pixel_start_idx + 1);
      const uint8_t G1 = o3d_image.data_.at(pixel_start_idx + 2);
      const uint8_t G2 = o3d_image.data_.at(pixel_start_idx + 3);
      const uint8_t B1 = o3d_image.data_.at(pixel_start_idx + 4);
      const uint8_t B2 = o3d_image.data_.at(pixel_start_idx + 5);

      // Note that these are flipped relative to how they were initialized
      EXPECT_EQ(R1, static_cast<uint8_t>(col));
      EXPECT_EQ(R2, static_cast<uint8_t>(row));
      EXPECT_EQ(G1, static_cast<uint8_t>(col * 3));
      EXPECT_EQ(G2, static_cast<uint8_t>(row * 5));
      EXPECT_EQ(B1, static_cast<uint8_t>(col + 6));
      EXPECT_EQ(B2, static_cast<uint8_t>(row + 2));
    }
  }
}

TEST(ConversionFunctions, rosToOpen3d_moveWithDifferentEndianness) {
  sensor_msgs::msg::Image ros_image;
  ros_image.height = 4;
  ros_image.width = 5;
  ros_image.encoding = "32SC2";
  ros_image.is_bigendian = rcpputils::endian::native != rcpputils::endian::big;
  ros_image.step = ros_image.width * 4 * 2;

  for (std::size_t row = 0; row < ros_image.height; ++row) {
    for (std::size_t col = 0; col < ros_image.width; ++col) {
      // Ch. 1
      ros_image.data.push_back(static_cast<uint8_t>(row));
      ros_image.data.push_back(static_cast<uint8_t>(col));
      ros_image.data.push_back(static_cast<uint8_t>(row + col));
      ros_image.data.push_back(static_cast<uint8_t>(row * col));
      // Ch. 2
      ros_image.data.push_back(static_cast<uint8_t>(row * 5));
      ros_image.data.push_back(static_cast<uint8_t>(col * 3));
      ros_image.data.push_back(static_cast<uint8_t>(row + 2));
      ros_image.data.push_back(static_cast<uint8_t>(col + 6));
    }
  }

  open3d::geometry::Image o3d_image;
  open3d_conversions::moveRosToOpen3d(std::move(ros_image), o3d_image);

  // Can compare against original image because a move was not possible
  EXPECT_EQ(
    o3d_image.bytes_per_channel_,
    sensor_msgs::image_encodings::bitDepth(ros_image.encoding) / 8
  );
  EXPECT_EQ(
    o3d_image.num_of_channels_,
    sensor_msgs::image_encodings::numChannels(ros_image.encoding)
  );
  EXPECT_EQ(o3d_image.width_, ros_image.width);
  EXPECT_EQ(o3d_image.height_, ros_image.height);
  EXPECT_EQ(o3d_image.BytesPerLine(), ros_image.step);

  for (std::size_t row = 0; row < o3d_image.height_; ++row) {
    for (std::size_t col = 0; col < o3d_image.width_; ++col) {
      const std::size_t pixel_start_idx = (row * o3d_image.width_ + col) *
        o3d_image.num_of_channels_ * o3d_image.bytes_per_channel_;

      const uint8_t Ch1_1 = o3d_image.data_.at(pixel_start_idx + 0);
      const uint8_t Ch1_2 = o3d_image.data_.at(pixel_start_idx + 1);
      const uint8_t Ch1_3 = o3d_image.data_.at(pixel_start_idx + 2);
      const uint8_t Ch1_4 = o3d_image.data_.at(pixel_start_idx + 3);
      const uint8_t Ch2_1 = o3d_image.data_.at(pixel_start_idx + 4);
      const uint8_t Ch2_2 = o3d_image.data_.at(pixel_start_idx + 5);
      const uint8_t Ch2_3 = o3d_image.data_.at(pixel_start_idx + 6);
      const uint8_t Ch2_4 = o3d_image.data_.at(pixel_start_idx + 7);

      // Note that these are flipped relative to how they were initialized
      EXPECT_EQ(Ch1_1, static_cast<uint8_t>(row * col));
      EXPECT_EQ(Ch1_2, static_cast<uint8_t>(row + col));
      EXPECT_EQ(Ch1_3, static_cast<uint8_t>(col));
      EXPECT_EQ(Ch1_4, static_cast<uint8_t>(row));
      EXPECT_EQ(Ch2_1, static_cast<uint8_t>(col + 6));
      EXPECT_EQ(Ch2_2, static_cast<uint8_t>(row + 2));
      EXPECT_EQ(Ch2_3, static_cast<uint8_t>(col * 3));
      EXPECT_EQ(Ch2_4, static_cast<uint8_t>(row * 5));
    }
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

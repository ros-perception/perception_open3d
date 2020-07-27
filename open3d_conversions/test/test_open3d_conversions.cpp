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

// GTest
#include <gtest/gtest.h>

//open3d_conversions
#include "open3d_conversions/open3d_conversions.h"

// Open3D
#include <open3d/Open3D.h>

// ROS
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// Boost
#include <boost/make_shared.hpp>

TEST(ConversionFunctions, open3dToRos_uncolored)
{
  open3d::geometry::PointCloud o3d_pc;
  for (int i = 0; i < 5; ++i)
  {
    o3d_pc.points_.push_back(Eigen::Vector3d(0.5 * i, i * i, 10.5 * i));
  }
  sensor_msgs::PointCloud2 ros_pc2;
  open3d_conversions::open3dToRos(o3d_pc, ros_pc2, "o3d_frame");
  EXPECT_EQ(ros_pc2.height * ros_pc2.width, o3d_pc.points_.size());
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_pc2, "z");
  for (int i = 0; i < 5; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
  {
    const Eigen::Vector3d &point = o3d_pc.points_[i];
    EXPECT_EQ(*ros_pc2_x, 0.5 * i);
    EXPECT_EQ(*ros_pc2_y, i * i);
    EXPECT_EQ(*ros_pc2_z, 10.5 * i);
  }
}

TEST(ConversionFunctions, open3dToRos_colored)
{
  open3d::geometry::PointCloud o3d_pc;
  for (int i = 0; i < 5; ++i)
  {
    o3d_pc.points_.push_back(Eigen::Vector3d(0.5 * i, i * i, 10.5 * i));
    o3d_pc.colors_.push_back(Eigen::Vector3d(2 * i / 255.0, 5 * i / 255.0, 10 * i / 255.0));
  }
  sensor_msgs::PointCloud2 ros_pc2;
  open3d_conversions::open3dToRos(o3d_pc, ros_pc2, "o3d_frame");
  EXPECT_EQ(ros_pc2.height * ros_pc2.width, o3d_pc.points_.size());
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_pc2, "x");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_pc2, "y");
  sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_pc2, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_r(ros_pc2, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_g(ros_pc2, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_b(ros_pc2, "b");
  for (int i = 0; i < 5; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b)
  {
    const Eigen::Vector3d &point = o3d_pc.points_[i];
    EXPECT_EQ(*ros_pc2_x, 0.5 * i);
    EXPECT_EQ(*ros_pc2_y, i * i);
    EXPECT_EQ(*ros_pc2_z, 10.5 * i);
    const Eigen::Vector3d &color = o3d_pc.points_[i];
    EXPECT_EQ(*ros_pc2_r, 2 * i);
    EXPECT_EQ(*ros_pc2_g, 5 * i);
    EXPECT_EQ(*ros_pc2_b, 10 * i);
  }
}

TEST(ConversionFunctions, rosToOpen3d_uncolored)
{
  sensor_msgs::PointCloud2 ros_pc2;
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

  for (int i = 0; i < 5; ++i, ++mod_x, ++mod_y, ++mod_z)
  {
    *mod_x = 0.5 * i;
    *mod_y = i * i;
    *mod_z = 10.5 * i;
  }

  const sensor_msgs::PointCloud2ConstPtr &ros_pc2_ptr = boost::make_shared<sensor_msgs::PointCloud2>(ros_pc2);
  open3d::geometry::PointCloud o3d_pc;
  open3d_conversions::rosToOpen3d(ros_pc2_ptr, o3d_pc);
  EXPECT_EQ(ros_pc2_ptr->height * ros_pc2_ptr->width, o3d_pc.points_.size());
  EXPECT_EQ(o3d_pc.HasColors(), false);
  for (unsigned int i = 0; i < 5; i++)
  {
    const Eigen::Vector3d &point = o3d_pc.points_[i];
    EXPECT_EQ(point(0), 0.5 * i);
    EXPECT_EQ(point(1), i * i);
    EXPECT_EQ(point(2), 10.5 * i);
  }
}

TEST(ConversionFunctions, rosToOpen3d_colored)
{
  sensor_msgs::PointCloud2 ros_pc2;
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

  for (int i = 0; i < 5; ++i, ++mod_x, ++mod_y, ++mod_z, ++mod_r, ++mod_g, ++mod_b)
  {
    *mod_x = 0.5 * i;
    *mod_y = i * i;
    *mod_z = 10.5 * i;
    *mod_r = 2 * i;
    *mod_g = 5 * i;
    *mod_b = 10 * i;
  }

  const sensor_msgs::PointCloud2ConstPtr &ros_pc2_ptr = boost::make_shared<sensor_msgs::PointCloud2>(ros_pc2);
  open3d::geometry::PointCloud o3d_pc;
  open3d_conversions::rosToOpen3d(ros_pc2_ptr, o3d_pc);
  EXPECT_EQ(ros_pc2_ptr->height * ros_pc2_ptr->width, o3d_pc.points_.size());
  EXPECT_EQ(o3d_pc.HasColors(), true);
  for (unsigned int i = 0; i < 5; i++)
  {
    const Eigen::Vector3d &point = o3d_pc.points_[i];
    EXPECT_EQ(point(0), 0.5 * i);
    EXPECT_EQ(point(1), i * i);
    EXPECT_EQ(point(2), 10.5 * i);
    const Eigen::Vector3d &color = o3d_pc.colors_[i];
    EXPECT_EQ(color(0), 2 * i / 255.0);
    EXPECT_EQ(color(1), 5 * i / 255.0);
    EXPECT_EQ(color(2), 10 * i / 255.0);
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
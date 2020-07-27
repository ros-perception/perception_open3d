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

#include "open3d_conversions/open3d_conversions.h"

namespace open3d_conversions
{
  void open3dToRos(const open3d::geometry::PointCloud &pointcloud, sensor_msgs::PointCloud2 &ros_pc2, std::string frame_id)
  {
    sensor_msgs::PointCloud2Modifier modifier(ros_pc2);
    if (pointcloud.HasColors())
    {
      modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    }
    else
    {
      modifier.setPointCloud2FieldsByString(1, "xyz");
    }
    modifier.resize(pointcloud.points_.size());
    ros_pc2.header.frame_id = frame_id;
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(ros_pc2, "x");
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(ros_pc2, "y");
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(ros_pc2, "z");
    if (pointcloud.HasColors())
    {
      sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_r(ros_pc2, "r");
      sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_g(ros_pc2, "g");
      sensor_msgs::PointCloud2Iterator<uint8_t> ros_pc2_b(ros_pc2, "b");
      for (size_t i = 0; i < pointcloud.points_.size();
           i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b)
      {
        const Eigen::Vector3d &point = pointcloud.points_[i];
        const Eigen::Vector3d &color = pointcloud.colors_[i];
        *ros_pc2_x = point(0);
        *ros_pc2_y = point(1);
        *ros_pc2_z = point(2);
        *ros_pc2_r = (int)(255 * color(0));
        *ros_pc2_g = (int)(255 * color(1));
        *ros_pc2_b = (int)(255 * color(2));
      }
    }
    else
    {
      for (size_t i = 0; i < pointcloud.points_.size(); i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
      {
        const Eigen::Vector3d &point = pointcloud.points_[i];
        *ros_pc2_x = point(0);
        *ros_pc2_y = point(1);
        *ros_pc2_z = point(2);
      }
    }
  }

  void rosToOpen3d(const sensor_msgs::PointCloud2ConstPtr &ros_pc2, open3d::geometry::PointCloud &o3d_pc, bool skip_colors)
  {
    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_x(*ros_pc2, "x");
    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_y(*ros_pc2, "y");
    sensor_msgs::PointCloud2ConstIterator<float> ros_pc2_z(*ros_pc2, "z");
    o3d_pc.points_.reserve(ros_pc2->height * ros_pc2->width);
    if (ros_pc2->fields.size() == 3 || skip_colors == true)
    {
      for (size_t i = 0; i < ros_pc2->height * ros_pc2->width; ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z)
      {
        o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
      }
    }
    else
    {
      o3d_pc.colors_.reserve(ros_pc2->height * ros_pc2->width);
      if (ros_pc2->fields[3].name == "rgb")
      {
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_r(*ros_pc2, "r");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_g(*ros_pc2, "g");
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_b(*ros_pc2, "b");

        for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
             ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_r, ++ros_pc2_g, ++ros_pc2_b)
        {
          o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
          o3d_pc.colors_.push_back(Eigen::Vector3d(((int)(*ros_pc2_r)) / 255.0, ((int)(*ros_pc2_g)) / 255.0,
                                                   ((int)(*ros_pc2_b)) / 255.0));
        }
      }
      else if (ros_pc2->fields[3].name == "intensity")
      {
        sensor_msgs::PointCloud2ConstIterator<uint8_t> ros_pc2_i(*ros_pc2, "intensity");
        for (size_t i = 0; i < ros_pc2->height * ros_pc2->width;
             ++i, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z, ++ros_pc2_i)
        {
          o3d_pc.points_.push_back(Eigen::Vector3d(*ros_pc2_x, *ros_pc2_y, *ros_pc2_z));
          o3d_pc.colors_.push_back(Eigen::Vector3d(*ros_pc2_i, *ros_pc2_i, *ros_pc2_i));
        }
      }
    }
  }
} // namespace open3d_conversions

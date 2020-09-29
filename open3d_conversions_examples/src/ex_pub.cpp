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

// Open3D
#include <open3d/Open3D.h>

// ROS
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// open3d_conversions
#include "open3d_conversions/open3d_conversions.h"

// C++
#include <string>

class PublisherExample
{
private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;

public:
  int count = 0;
  std::string path;
  sensor_msgs::PointCloud2 ros_pc2;
  open3d::geometry::PointCloud o3d_pc;

  PublisherExample(ros::NodeHandle& nh, std::string path) : nh_(nh)
  {
    pub_ = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);
    open3d::io::ReadPointCloud(path, o3d_pc);
    ros::Rate loop_rate(2);
    while (ros::ok())
    {
      pointCloudConversion(o3d_pc, ros_pc2);
      ros::spinOnce();
      loop_rate.sleep();
      ++count;
      ROS_INFO("Published %d pointclouds", count);
    }
  }

  virtual ~PublisherExample()
  {
  }

  void pointCloudConversion(const open3d::geometry::PointCloud& pointcloud, sensor_msgs::PointCloud2& ros_pointcloud)
  {
    open3d_conversions::open3dToRos(pointcloud, ros_pointcloud, "o3d_frame");
    ros_pointcloud.header.stamp = ros::Time::now();
    pub_.publish(ros_pointcloud);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "open3d_conversions_ex_pub");
  ros::NodeHandle nh;
  std::string path;
  int count = 0;
  if (argc == 2)
  {
    path = (std::string)argv[1];
  }
  else
  {
    std::string path_pkg = ros::package::getPath("open3d_conversions_examples");
    path = path_pkg + "/data/fragment.pcd";
  }

  PublisherExample publisherExample(nh, path);
}

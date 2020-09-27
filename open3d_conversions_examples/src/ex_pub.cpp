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
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/package.h>

// open3d_conversions
#include "open3d_conversions/open3d_conversions.h"

// C++
#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "open3d_conversions_ex_pub");
  ros::NodeHandle nh;
  ros::Rate loop_rate(2);
  ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);

  // Example XYZRGB Pointcloud
  sensor_msgs::PointCloud2 ros_pc2;
  open3d::geometry::PointCloud o3d_pc;
  if (argc == 2)
  {
    open3d::io::ReadPointCloud((std::string)argv[1], o3d_pc);
  }
  else
  {
    std::string path = ros::package::getPath("open3d_conversions_examples");
    open3d::io::ReadPointCloud(path + "/data/fragment.pcd", o3d_pc);
  }
  open3d_conversions::open3dToRos(o3d_pc, ros_pc2, "o3d_frame");
  int count = 0;
  while (ros::ok())
  {
    ros_pc2.header.stamp = ros::Time::now();
    pubCloud.publish(ros_pc2);
    ++count;
    ROS_INFO("Published %d pointclouds", count);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

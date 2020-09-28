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

// open3d_conversions
#include "open3d_conversions/open3d_conversions.h"

class SubscriberExample
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;

public:
  open3d::geometry::PointCloud pcd;

  SubscriberExample(ros::NodeHandle& nh) : nh_(nh)
  {
    sub_ = nh.subscribe("pointcloud_in", 1, &SubscriberExample::cloud_callback, this);
  }

  virtual ~SubscriberExample()
  {
  }

  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_data)
  {
    open3d::geometry::PointCloud pcd;
    open3d_conversions::rosToOpen3d(cloud_data, pcd);
    ROS_INFO("Recieved pointcloud with sequence number: %d", cloud_data->header.seq);
    // Do something with the Open3D pointcloud
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "open3d_conversions_ex_sub");
  ros::NodeHandle nh;
  SubscriberExample subscriberExample(nh);
  ros::spin();
}
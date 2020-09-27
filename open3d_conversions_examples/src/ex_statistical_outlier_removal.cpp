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

class SubscribeFilterPublish
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  double nb_neighbors_ = 20;
  double std_ratio_ = 2.0;

public:
  SubscribeFilterPublish(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh)
  {
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_out", 1);
    sub_ = nh_.subscribe("pointcloud_in", 1, &SubscribeFilterPublish::cloud_callback, this);
    pnh_.getParam("nb_neighbors", nb_neighbors_);
    pnh_.getParam("std_ratio", std_ratio_);
    ROS_INFO("Waiting for pointclouds...");
  }

  ~SubscribeFilterPublish() {}

  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_data)
  {
    ROS_INFO("Recieved pointcloud with sequence number: %i", cloud_data->header.seq);
    open3d::geometry::PointCloud pcd;
    open3d_conversions::rosToOpen3d(cloud_data, pcd);
    std::shared_ptr<open3d::geometry::PointCloud> filtered_pcd;
    std::vector<size_t> ind;
    std::tie(filtered_pcd, ind) = pcd.RemoveStatisticalOutliers(nb_neighbors_, std_ratio_);
    sensor_msgs::PointCloud2 ros_pc2;
    open3d_conversions::open3dToRos(*filtered_pcd, ros_pc2, cloud_data->header.frame_id);
    pub_.publish(ros_pc2);
    ROS_INFO("Published downsampled pointcloud with points original/downsampled: %lu/%lu", pcd.points_.size(), filtered_pcd->points_.size());
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "open3d_conversions_ex_statistical_outlier_removal");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  SubscribeFilterPublish sfp(nh, pnh);
  ros::spin();
}
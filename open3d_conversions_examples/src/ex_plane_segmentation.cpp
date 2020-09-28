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
  ros::Publisher pub_inlier_;
  ros::Publisher pub_outlier_;
  ros::Subscriber sub_;
  double distance_threshold_ = 0.01;
  int ransac_n_ = 3;
  int num_iterations_ = 1000;

public:
  SubscribeFilterPublish(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh)
  {
    pub_inlier_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_out/inlier_cloud", 1);
    pub_outlier_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_out/outlier_cloud", 1);
    sub_ = nh_.subscribe("pointcloud_in", 1, &SubscribeFilterPublish::cloud_callback, this);
    pnh_.getParam("distance_threshold", distance_threshold_);
    pnh_.getParam("ransac_n", ransac_n_);
    pnh_.getParam("num_iterations", num_iterations_);
    ROS_INFO("Waiting for pointclouds...");
  }

  ~SubscribeFilterPublish()
  {
  }

  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_data)
  {
    ROS_INFO("Recieved pointcloud with sequence number: %i", cloud_data->header.seq);
    open3d::geometry::PointCloud pcd;
    open3d_conversions::rosToOpen3d(cloud_data, pcd);
    Eigen::Vector4d plane_model;
    std::vector<size_t, std::allocator<size_t>> inliers;
    std::tie(plane_model, inliers) = pcd.SegmentPlane(distance_threshold_, ransac_n_, num_iterations_);
    ROS_INFO("Plane equation: %.2fx + %.2fy + %.2fz + %.2f = 0", plane_model[0], plane_model[1], plane_model[2],
             plane_model[3]);
    std::shared_ptr<open3d::geometry::PointCloud> inlier_cloud = pcd.SelectByIndex(inliers);
    Eigen::Vector3d inlier_color{ 1.0, 0, 0 };
    open3d::geometry::PointCloud painted_cloud = inlier_cloud->PaintUniformColor(inlier_color);
    std::shared_ptr<open3d::geometry::PointCloud> outlier_cloud = pcd.SelectByIndex(inliers, true);
    ROS_INFO("Number of inliers/outliers: %lu/%lu", inlier_cloud->points_.size(), outlier_cloud->points_.size());
    sensor_msgs::PointCloud2 ros_pc2_inlier, ros_pc2_outlier;
    open3d_conversions::open3dToRos(painted_cloud, ros_pc2_inlier, cloud_data->header.frame_id);
    pub_inlier_.publish(ros_pc2_inlier);
    open3d_conversions::open3dToRos(*outlier_cloud, ros_pc2_outlier, cloud_data->header.frame_id);
    pub_outlier_.publish(ros_pc2_outlier);
    ROS_INFO("Published inlier and outlier clouds");
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "open3d_conversions_ex_plane_segmentation");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  SubscribeFilterPublish sfp(nh, pnh);
  ros::spin();
}
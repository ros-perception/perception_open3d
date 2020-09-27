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
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/pipelines/registration/ColoredICP.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/Odometry.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

// open3d_conversions
#include "open3d_conversions/open3d_conversions.h"

// C++
#include <stdlib.h>
#include <Eigen/Core>
#include <vector>
#include <Eigen/Geometry>

class Icp
{
private:
  ros::NodeHandle nh;
  ros::Publisher pub_odom;
  ros::Publisher pub_icp_time;
  ros::Publisher pub_tf;
  ros::Publisher transformed_cloud_publisher;
  ros::Subscriber ros_pc_sub;
  open3d::pipelines::registration::RegistrationResult icp_result;
  Eigen::Matrix4d current_transform = Eigen::Matrix4d::Identity();
  ros::Time old_pc_ts;
  uint32_t old_pc_seq;
  uint32_t first_pc_seq = 0;
  bool flag = true;
  Eigen::Matrix4d prior = Eigen::Matrix4d::Identity();

public:
  int icp_max_iterations, statistical_outlier_filter_num_neighbors, num_initialization_pointclouds,
    radius_outlier_filter_nb_points;
  double icp_threshold, downsample_voxel_size, statistical_outlier_filter_std_ratio, radius_outlier_filter_radius;
  std::string world_frame, odom_init_frame, odom_frame, depth_camera_frame;
  bool check_pc_seq, use_identity_prior, passthrough_prior, colorized_icp, apply_statistical_outlier_filter,
    apply_radius_outlier_filter;
  tf2_ros::Buffer tf_buffer;
  open3d::geometry::PointCloud old_pc;

  Icp(ros::NodeHandle no_ha) : nh(no_ha)
  {
    this->getParams(nh);
    pub_odom = nh.advertise<nav_msgs::Odometry>("odom_out", 1);
    pub_tf = nh.advertise<tf2_msgs::TFMessage>("/tf", 1);
    transformed_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("PointCloud2_out_topic", 1);
    ros_pc_sub = nh.subscribe("pointcloud_in_topic", 1, &Icp::pointCloudCb, this);
    tf2_ros::TransformListener tfListener(this->tf_buffer);
    ros::spin();
  }

  virtual ~Icp()
  {
  }

  void getParams(ros::NodeHandle nh)
  {
    nh.param<double>("icp_threshold", icp_threshold, 0.2);
    nh.param("icp_max_iterations", icp_max_iterations, 20);
    nh.param<double>("downsample_voxel_size", downsample_voxel_size, 0.01);
    nh.param<std::string>("world_frame", world_frame, "world");
    nh.param<std::string>("odom_init_frame", odom_init_frame, "mapper_init");
    nh.param<std::string>("odom_frame", odom_frame, "mapper");
    nh.param<std::string>("depth_camera_frame", depth_camera_frame, "l515_depth_optical_frame");
    nh.param("check_pointcloud_sequence", check_pc_seq, false);
    nh.param("use_identity_prior", use_identity_prior, false);
    nh.param("passthrough_prior", passthrough_prior, false);
    nh.param("use_colorized_icp", colorized_icp, false);
    nh.param("use_statistical_outlier_filter", apply_statistical_outlier_filter, false);
    nh.param("statistical_outlier_filter_num_neighbors", statistical_outlier_filter_num_neighbors, 20);
    nh.param<double>("statistical_outlier_filter_std_ratio", statistical_outlier_filter_std_ratio, 2.0);
    nh.param("use_radius_outlier_filter", apply_radius_outlier_filter, true);
    nh.param("radius_outlier_filter_nb_points", radius_outlier_filter_nb_points, 4);
    nh.param<double>("radius_outlier_filter_radius", radius_outlier_filter_radius, 0.1);
    nh.param("num_initialization_pointclouds", num_initialization_pointclouds, 2);
  }

  void pointCloudCb(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    ROS_INFO("New pointcloud received");
    if (this->first_pc_seq == 0)
    {
      this->first_pc_seq = msg->header.seq;
    }
    ros::Time new_pc_ts = msg->header.stamp;
    uint32_t new_pc_seq = msg->header.seq;
    if (new_pc_seq - this->first_pc_seq < this->num_initialization_pointclouds)
    {
      ROS_INFO("Skipping initialization pointcloud");
      return;
    }
    open3d::geometry::PointCloud o3d_new_pc;
    // change according to XYZ pointcloud or XYZRGB
    open3d_conversions::rosToOpen3d(msg, o3d_new_pc, false);
    std::shared_ptr<open3d::geometry::PointCloud> new_pc(new open3d::geometry::PointCloud);
    *new_pc = o3d_new_pc;
    // Preprocessing
    new_pc = new_pc->VoxelDownSample(this->downsample_voxel_size);
    std::tuple<std::shared_ptr<open3d::geometry::PointCloud>, std::vector<size_t> > temp;
    if (this->apply_statistical_outlier_filter)
    {
      temp = new_pc->RemoveStatisticalOutliers(this->statistical_outlier_filter_num_neighbors,
                                               this->statistical_outlier_filter_std_ratio);
      new_pc = std::get<0>(temp);
    }
    if (this->apply_radius_outlier_filter)
    {
      std::tuple<std::shared_ptr<open3d::geometry::PointCloud>, std::vector<size_t> > temp_t;
      temp_t = new_pc->RemoveRadiusOutliers(this->radius_outlier_filter_nb_points, this->radius_outlier_filter_radius);
      new_pc = std::get<0>(temp_t);
    }
    if (this->flag)
    {
      // First pointcloud
      this->flag = false;
      this->old_pc = *new_pc;
      this->old_pc_ts = new_pc_ts;
      this->old_pc_seq = new_pc_seq;
      this->broadcastTransformation(new_pc_ts);
      return;
    }

    if (this->check_pc_seq && (msg->header.seq != (this->old_pc_seq + 1)))
    {
      ROS_ERROR("skipped pointcloud");
      return;
    }

    if (this->use_identity_prior)
    {
      this->prior = Eigen::Matrix4d::Identity();
    }
    else
    {
      this->prior = this->msgToSE3(this->tfTimeTravel(new_pc_ts, this->old_pc_ts));
    }

    if (this->colorized_icp)
    {
      ROS_INFO("Starting colored ICP");
      this->old_pc.EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(10), false);
      new_pc->EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(10), false);
      this->icp_result = open3d::pipelines::registration::RegistrationColoredICP(
        this->old_pc, *new_pc, this->icp_threshold, this->prior,
        open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, this->icp_max_iterations));
      std::cout << "Difference between prior and ICP result:" << std::endl;
      this->printTransformationDifference(this->icp_result.transformation_, this->prior);
      ROS_INFO("Finished colored ICP");
    }
    else
    {
      ROS_INFO("Starting ICP");
      this->icp_result = open3d::pipelines::registration::RegistrationICP(
        this->old_pc, *new_pc, this->icp_threshold, this->prior,
        open3d::pipelines::registration::TransformationEstimationPointToPoint(),
        open3d::pipelines::registration::ICPConvergenceCriteria(this->icp_max_iterations));
      ROS_INFO("Finished ICP");
      std::cout << "Difference between prior and ICP result:" << std::endl;
      this->printTransformationDifference(this->icp_result.transformation_, this->prior);
    }

    if (this->passthrough_prior)
    {
      // Use the prior instead of the ICP computed transform
      current_transform = current_transform * this->se3Inverse(prior);
    }
    else
    {
      current_transform = current_transform * this->se3Inverse(icp_result.transformation_);
    }

    this->broadcastTransformation(new_pc_ts);
    open3d::geometry::PointCloud new_pc_transformed;
    sensor_msgs::PointCloud2 ros_pc2;
    new_pc_transformed = *new_pc;
    new_pc_transformed = new_pc_transformed.Transform(current_transform);
    open3d_conversions::open3dToRos(new_pc_transformed, ros_pc2, this->odom_init_frame);
    this->transformed_cloud_publisher.publish(ros_pc2);
    this->old_pc = *new_pc;
    old_pc_ts = new_pc_ts;
    old_pc_seq = new_pc_seq;
  }

  Eigen::Matrix4d se3Inverse(const Eigen::Matrix4d& mat)
  {
    Eigen::Matrix3d R = mat.block(0, 0, 3, 3);
    Eigen::MatrixXd t = mat.block(0, 3, 3, 1);
    Eigen::Matrix4d mat_inv = Eigen::Matrix4d::Zero();
    mat_inv.block<3, 3>(0, 0) = R.transpose();
    mat_inv.block<3, 1>(0, 2) = -R.transpose() * t;
    return mat_inv;
  }
  geometry_msgs::TransformStamped tfTimeTravel(const ros::Time& start_time, const ros::Time& end_time)
  {
    return tf_buffer.lookupTransform(this->depth_camera_frame, start_time, this->depth_camera_frame, end_time,
                                    this->world_frame, ros::Duration(0.2));
  }
  void broadcastTransformation(const ros::Time& stamp)
  {
    geometry_msgs::TransformStamped t;
    t.header.frame_id = this->odom_init_frame;
    t.header.stamp = stamp;
    t.child_frame_id = this->odom_frame;
    t.transform.translation.x = current_transform(0, 3);
    t.transform.translation.y = current_transform(1, 3);
    t.transform.translation.z = current_transform(2, 3);
    Eigen::MatrixXd q(4, 1);
    q = quaternionFromMatrix(current_transform);
    t.transform.rotation.x = q(0);
    t.transform.rotation.y = q(1);
    t.transform.rotation.z = q(2);
    t.transform.rotation.w = q(3);
    tf2_msgs::TFMessage tfs;
    tfs.transforms.push_back(t);
    this->pub_tf.publish(tfs);

    nav_msgs::Odometry o;
    o.header.frame_id = odom_init_frame;
    o.header.stamp = stamp;
    o.pose.pose.position.x = current_transform(0, 3);
    o.pose.pose.position.x = current_transform(1, 3);
    o.pose.pose.position.x = current_transform(2, 3);
    o.pose.pose.orientation.x = q(0);
    o.pose.pose.orientation.y = q(1);
    o.pose.pose.orientation.z = q(2);
    o.pose.pose.orientation.w = q(3);
    this->pub_odom.publish(o);
  }

  Eigen::MatrixXd quaternionFromMatrix(const Eigen::Matrix4d& m)
  {
    Eigen::Matrix3d mat;
    mat = m.block<3, 3>(0, 0);
    Eigen::Quaterniond q(mat);
    q.normalize();
    Eigen::MatrixXd res(4, 1);
    res << q.x(), q.y(), q.z(), q.w();
    return res;
  }

  Eigen::Matrix4d msgToSE3(const geometry_msgs::Pose& msg)
  {
    tf2::Quaternion quat(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    quat = quat.normalized();
    tf2::Matrix3x3 matrix(quat);
    tf2::Vector3 row_1 = matrix.getRow(1);
    tf2::Vector3 row_2 = matrix.getRow(2);
    tf2::Vector3 row_3 = matrix.getRow(3);
    Eigen::Matrix4d ret_val;
    ret_val.block(0, 0, 3, 3) << row_1.getX(), row_1.getY(), row_1.getZ(), row_2.getX(), row_2.getY(), row_2.getZ(),
      row_3.getX(), row_3.getY(), row_3.getZ();
    ret_val.block(0, 3, 3, 1) << msg.position.x, msg.position.y, msg.position.z;
    return ret_val;
  }
  Eigen::Matrix4d msgToSE3(const geometry_msgs::PoseStamped& msg)
  {
    return this->msgToSE3(msg.pose);
  }
  Eigen::Matrix4d msgToSE3(const geometry_msgs::Transform& msg)
  {
    Eigen::MatrixXd q(4, 1);
    q << msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w;
    tf2::Quaternion quat(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w);
    quat = quat.normalized();
    tf2::Matrix3x3 matrix(quat);
    tf2::Vector3 row_1 = matrix.getRow(1);
    tf2::Vector3 row_2 = matrix.getRow(2);
    tf2::Vector3 row_3 = matrix.getRow(3);
    Eigen::Matrix4d ret_val;
    ret_val.block(0, 0, 3, 3) << row_1.getX(), row_1.getY(), row_1.getZ(), row_2.getX(), row_2.getY(), row_2.getZ(),
      row_3.getX(), row_3.getY(), row_3.getZ();
    ret_val.block(0, 3, 3, 1) << msg.translation.x, msg.translation.y, msg.translation.z;
    return ret_val;
  }
  Eigen::Matrix4d msgToSE3(const geometry_msgs::TransformStamped& msg)
  {
    return this->msgToSE3(msg.transform);
  }

  void printTransformationDifference(const Eigen::Matrix4d_u& o3d_mat, const Eigen::Matrix4d& mat)
  {
    Eigen::Matrix4d to3d_mat = this->se3Inverse(o3d_mat) * mat;
    this->printMatrixEulerTransformation(to3d_mat);
  }
  void printMatrixEulerTransformation(Eigen::Matrix4d rt_matrix)
  {
    Eigen::Matrix3d R = rt_matrix.block(0, 0, 3, 3);
    Eigen::Vector3d xyz = R.eulerAngles(0, 1, 2);
    std::cout << "Rotation: " << xyz(0) << " " << xyz(1) << " " << xyz(2) << std::endl;
    std::cout << "Translation: " << rt_matrix(0, 3) << " " << rt_matrix(1, 3) << " " << rt_matrix(2, 3) << std::endl;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "perception_open3d_ex_icp");
  ros::NodeHandle nh;
  Icp icp(nh);
}

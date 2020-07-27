#ifndef OPEN3D_CONVERSIONS_HPP_
#define OPEN3D_CONVERSIONS_HPP_

// Open3D
#include <open3d/Open3D.h>

// ROS
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// Eigen
#include <Eigen/Dense>

// C++
#include <string>

namespace open3d_conversions
{
  /**
   * @brief Copy data from a open3d::geometry::PointCloud to a sensor_msgs::PointCloud2 
   * 
   * @param pointcloud Reference to the open3d PointCloud
   * @param ros_pc2 Reference to the sensor_msgs PointCloud2
   * @param frame_id The string to be placed in the frame_id of the PointCloud2
   */
  void open3dToRos(const open3d::geometry::PointCloud &pointcloud, sensor_msgs::PointCloud2 &ros_pc2,
                   std::string frame_id = "open3d_pointcloud");

  /**
   * @brief Copy data from a sensor_msgs::PointCloud2 to a open3d::geometry::PointCloud
   * 
   * @param ros_pc2 Reference to the sensor_msgs PointCloud2
   * @param o3d_pc Reference to the open3d PointCloud
   * @param skip_colors If true, only xyz fields will be copied
   */
  void rosToOpen3d(const sensor_msgs::PointCloud2ConstPtr &ros_pc2, open3d::geometry::PointCloud &o3d_pc,
                   bool skip_colors = false);
} // namespace open3d_conversions

#endif // OPEN3D_CONVERSIONS_HPP_

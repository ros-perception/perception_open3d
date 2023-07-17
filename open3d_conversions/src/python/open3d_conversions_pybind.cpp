#include <pybind11/pybind11.h> 
#include <pybind11/detail/internals.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <Eigen/Core>
#include "open3d_conversions/open3d_conversions.h"

PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Vector3d>);

PYBIND11_MODULE(open3d_conversions_py, m) {
    namespace py = pybind11;

    m.def("open3dToRos",
          [](const open3d::geometry::PointCloud& pointcloud, sensor_msgs::PointCloud2& ros_pc2,
             std::string frame_id) {
              open3d_conversions::open3dToRos(pointcloud, ros_pc2, frame_id);
          },
          py::arg("pointcloud"), py::arg("ros_pc2"), py::arg("frame_id") = "open3d_pointcloud");

    m.def("rosToOpen3d",
          [](const sensor_msgs::PointCloud2ConstPtr& ros_pc2, open3d::geometry::PointCloud& o3d_pc,
             bool skip_colors) {
              open3d_conversions::rosToOpen3d(ros_pc2, o3d_pc, skip_colors);
          },
          py::arg("ros_pc2"), py::arg("o3d_pc"), py::arg("skip_colors") = false);

    m.def("open3dToRos_t",
          [](const open3d::t::geometry::PointCloud& pointcloud, sensor_msgs::PointCloud2& ros_pc2,
             std::string frame_id, int t_num_fields) {
              open3d_conversions::open3dToRos(pointcloud, ros_pc2, frame_id, t_num_fields);
          },
          py::arg("pointcloud"), py::arg("ros_pc2"), py::arg("frame_id") = "open3d_pointcloud",
          py::arg("t_num_fields") = 2);

    m.def("rosToOpen3d_t",
          [](const sensor_msgs::PointCloud2ConstPtr& ros_pc2, open3d::t::geometry::PointCloud& o3d_pc,
             bool skip_colors) {
              open3d_conversions::rosToOpen3d(ros_pc2, o3d_pc, skip_colors);
          },
          py::arg("ros_pc2"), py::arg("o3d_pc"), py::arg("skip_colors") = false);
    
    m.def("addPointField",
          [](sensor_msgs::PointCloud2& cloud_msg, const std::string& name,
             int count, int datatype, int offset) {
              addPointField(cloud_msg, name, count, datatype, offset);
          },
          py::arg("cloud_msg"), py::arg("name"), py::arg("count"),py::arg("datatype"), py::arg("offset"));

     m.def("sizeOfPointField",
          [](int datatype) {
              sizeOfPointField(datatype);
          },
          py::arg("datatype"));
}
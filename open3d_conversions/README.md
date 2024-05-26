# open3d_conversions

This package provides functions that can convert pointclouds and images from ROS to Open3D and vice-versa.

## Dependencies

* Eigen3
* Open3D

## System Requirements

* Ubuntu 20.04+: GCC 5+

## Installation

### Open3D

* Instructions to setup Open3D can be found [here](http://www.open3d.org/docs/release/compilation.html).

### open3d_conversions

* In case you are building this package from source, time taken for the conversion functions will be much larger if it is not built in `Release` mode.

## Pointcloud Conversion

There are two pointcloud conversion functions provided in this library:

```cpp
void open3d_conversions::open3dToRos(
  const open3d::geometry::PointCloud & pointcloud,
  sensor_msgs::msg::PointCloud2 & ros_pc2, 
  std::string frame_id = "open3d_pointcloud");

void open3d_conversions::rosToOpen3d(
  const sensor_msgs::msg::PointCloud2::SharedPtr & ros_pc2,
  open3d::geometry::PointCloud & o3d_pc, 
  bool skip_colors=false);
```

* As Open3D pointclouds only contain `points`, `colors` and `normals`, the interface currently supports XYZ, XYZRGB pointclouds. XYZI pointclouds are handled by placing the `intensity` value in the `colors_`.
* On creating a ROS pointcloud from an Open3D pointcloud, the user is expected to set the timestamp in the header and pass the `frame_id` to the conversion function.

## Image Conversion

There are four image conversion functions provided in this library:

```cpp
void open3dToRos(
  const open3d::geometry::Image & o3d_img,
  sensor_msgs::msg::Image & ros_img,
  std::string encoding,
  std::string frame_id = "open3d_image");

void open3dToRos(
  const open3d::camera::PinholeCameraIntrinsic & intrinsic,
  sensor_msgs::msg::CameraInfo & camera_info,
  std::string frame_id = "open3d_image");

void rosToOpen3d(
  const sensor_msgs::msg::Image & ros_img,
  open3d::geometry::Image & o3d_img);

void rosToOpen3d(
  const sensor_msgs::msg::CameraInfo & camera_info,
  open3d::camera::PinholeCameraIntrinsic & intrinsic);
```
Additionally, two explicit move-only versions of the image conversions are provided for efficiency:

```cpp
void moveOpen3dToRos(
  open3d::geometry::Image && o3d_img,
  sensor_msgs::msg::Image & ros_img,
  std::string encoding,
  std::string frame_id = "open3d_image");

void moveRosToOpen3d(
  sensor_msgs::msg::Image && ros_img,
  open3d::geometry::Image & o3d_img);
```

* As Open3D only stores camera info as a pinhole camera model, distorsion information cannot be inferred from an Open3D -> ROS conversion. Such a conversions sets the distorsion model to "plumb_bob" with all five **D** parameters equal to zero.

## Documentation

Documentation can be generated using Doxygen and the configuration file by executing  `doxygen Doxyfile` in the package.

## Contact

Feel free to contact us for any questions:

* [Pranay Mathur](mailto:matnay17@gmail.com)
* [Nikhil Khedekar](mailto:nkhedekar@nevada.unr.edu)
* [Kostas Alexis](mailto:kalexis@unr.edu)

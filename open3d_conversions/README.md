# open3d_conversions

This package provides functions that can convert pointclouds from ROS to Open3D and vice-versa.

## Dependencies

* Eigen3
* Open3D

## System Requirements

* Ubuntu 18.04+: GCC 5+

## Installation

### Open3D

* Instructions to setup Open3D can be found [here](http://www.open3d.org/docs/release/compilation.html).

### open3d_conversions

* In case you are building this package from source, time taken for the conversion functions will be much larger if it is not built in `Release` mode.

## Usage

There are two functions provided in this library:

```cpp
void open3d_conversions::open3dToRos(const open3d::geometry::PointCloud& pointcloud, sensor_msgs::PointCloud2& ros_pc2, std::string frame_id = "open3d_pointcloud");

void open3d_conversions::rosToOpen3d(const sensor_msgs::PointCloud2ConstPtr& ros_pc2, open3d::geometry::PointCloud& o3d_pc, bool skip_colors=false);
```

* As Open3D pointclouds only contain `points`, `colors` and `normals`, the interface currently supports XYZ, XYZRGB pointclouds. XYZI pointclouds are handled by placing the `intensity` value in the `colors_`.
* On creating a ROS pointcloud from an Open3D pointcloud, the user is expected to set the timestamp in the header and pass the `frame_id` to the conversion function.

## Documentation

Documentation can be generated using Doxygen and the configuration file by executing  `doxygen Doxyfile` in the package.

## Contact

Feel free to contact us for any questions:

* [Pranay Mathur](mailto:matnay17@gmail.com)
* [Nikhil Khedekar](mailto:nkhedekar@nevada.unr.edu)
* [Kostas Alexis](mailto:kalexis@unr.edu)

# open3d_conversions_examples

This package contains a set of examples using functions provided by the open3d_conversions library

## Dependencies

* Eigen3
* Open3D
* open3d_conversions
* tf2_ros
* tf2

## System Requirements

* Ubuntu 18.04+: GCC 5+, Clang 7+

## Usage

* Publishers

    ```
    rosrun perception_open3d_examples ex_pub
    ```

* Subscribers

    ```
    rosrun perception_open3d_examples ex_sub
    ```

* Conversion times (Will read ```data/fragment.pcd``` by default if optional path to pcd file not provided)

    ```
    rosrun perception_open3d_examples ex_conv_times <path_to_pcd_file/filename.pcd>
    ```

* ICP (Contains an example on carrying out ICP alignment. Requires playing your rosbag with `--clock` in a seperate terminal)

    ```
    roslaunch perception_open3d_examples ex_icp.launch
    ```

* Downsampling (Voxel grid filtering)

    ```
    rosrun open3d_conversions_examples ex_downsample
    rosrun open3d_conversions_examples ex_pub
    ```

* Statistical Outlier Filtering

    ```
    rosrun open3d_conversions_examples ex_statistical_outlier_removal
    rosrun open3d_conversions_examples ex_pub
    ```

* Plane Segmentation

    ```
    rosrun open3d_conversions_examples ex_plane_segmentation
    rosrun open3d_conversions_examples ex_pub
    ```

* Paint Uniform (Paint all points a single color)

    ```
    rosrun open3d_conversions_examples ex_paint_uniform
    rosrun open3d_conversions_examples ex_pub
    ```

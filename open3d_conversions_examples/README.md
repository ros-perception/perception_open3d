# open3d_conversions_examples

This package contains a set of examples using functions provided by the open3d_conversions library

## Dependencies

* Eigen3
* Open3D
* open3d_conversions

## System Requirements

* Ubuntu 18.04+: GCC 5+, Clang 7+

## Usage

* Publishers (Will read `data/fragment.pcd` by default if optional path to pcd file not provided)

    ```
    rosrun open3d_conversions_examples ex_pub <path_to_pcd_file/filename.pcd>
    ```

* Subscribers

    ```
    rosrun open3d_conversions_examples ex_sub
    ```

* Conversion times (Will read `data/fragment.pcd` by default if optional path to pcd file not provided)

    ```
    rosrun open3d_conversions_examples ex_conv_times <path_to_pcd_file/filename.pcd>
    ```

* Downsampling (Voxel grid filtering)


    ```
    rosrun open3d_conversions_examples ex_downsample
    rosrun open3d_conversions_examples ex_pub
    ```
    - Parameters:
        - voxel_size: Voxel size to downsample into

* Statistical Outlier Filtering

    ```
    rosrun open3d_conversions_examples ex_statistical_outlier_removal
    rosrun open3d_conversions_examples ex_pub
    ```
    - Parameters:
        - nb_neighbors – Number of neighbors around the target point.
        - std_ratio - Standard deviation ratio.

* Plane Segmentation

    ```
    rosrun open3d_conversions_examples ex_plane_segmentation
    rosrun open3d_conversions_examples ex_pub
    ```
    - Parameters:
        - distance_threshold – Max distance a point can be from the plane model, and still be considered an inlier.
        - ransac_n  – Number of initial points to be considered inliers in each iteration.
        - num_iterations – Number of iterations.

* Paint Uniform (Paint all points a single color)

    ```
    rosrun open3d_conversions_examples ex_paint_uniform
    rosrun open3d_conversions_examples ex_pub
    ```
     - Parameters:
        - r - Intensity of Red in the PointCloud
        - g - Intensity of Green in the PointCloud
        - b - Intensity of Blue in the PointCloud


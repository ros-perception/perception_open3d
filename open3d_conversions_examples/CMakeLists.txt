cmake_minimum_required(VERSION 3.5) 
project(open3d_conversions_examples) 
set(CMAKE_CXX_STANDARD 14)

#System Dependencies
find_package(Eigen3 REQUIRED) 
find_package(Open3D REQUIRED)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  std_msgs 
  sensor_msgs 
  roslib
  open3d_conversions
)

include_directories(${EIGEN3_INCLUDE_DIR}) 
include_directories(${Open3D_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
CATKIN_DEPENDS roscpp std_msgs sensor_msgs roslib open3d_conversions
DEPENDS Open3D
)

#Examples
add_executable(ex_conv_times src/ex_conv_times.cpp) 
target_link_libraries(ex_conv_times ${catkin_LIBRARIES} ${Open3D_LIBRARIES}) 
target_include_directories(ex_conv_times PUBLIC ${Open3D_INCLUDE_DIRS})

add_executable(ex_sub src/ex_sub.cpp) 
target_link_libraries(ex_sub ${catkin_LIBRARIES} ${Open3D_LIBRARIES}) 
target_include_directories(ex_sub PUBLIC ${Open3D_INCLUDE_DIRS})

add_executable(ex_pub src/ex_pub.cpp)
target_link_libraries(ex_pub ${catkin_LIBRARIES} ${Open3D_LIBRARIES})
target_include_directories(ex_pub PUBLIC ${Open3D_INCLUDE_DIRS})

add_executable(ex_downsample src/ex_downsample.cpp)
target_link_libraries(ex_downsample ${catkin_LIBRARIES} ${Open3D_LIBRARIES})
target_include_directories(ex_downsample PUBLIC ${Open3D_INCLUDE_DIRS})

add_executable(ex_statistical_outlier_removal src/ex_statistical_outlier_removal.cpp)
target_link_libraries(ex_statistical_outlier_removal ${catkin_LIBRARIES} ${Open3D_LIBRARIES})
target_include_directories(ex_statistical_outlier_removal PUBLIC ${Open3D_INCLUDE_DIRS})

add_executable(ex_paint_uniform src/ex_paint_uniform.cpp)
target_link_libraries(ex_paint_uniform ${catkin_LIBRARIES} ${Open3D_LIBRARIES})
target_include_directories(ex_paint_uniform PUBLIC ${Open3D_INCLUDE_DIRS})

add_executable(ex_plane_segmentation src/ex_plane_segmentation.cpp)
target_link_libraries(ex_plane_segmentation ${catkin_LIBRARIES} ${Open3D_LIBRARIES})
target_include_directories(ex_plane_segmentation PUBLIC ${Open3D_INCLUDE_DIRS})

# Install
install(TARGETS ex_pub ex_sub ex_conv_times
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY data/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data)

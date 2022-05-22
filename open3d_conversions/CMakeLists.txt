cmake_minimum_required(VERSION 3.5)
project(open3d_conversions)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# system dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(open3d_vendor REQUIRED)
find_package(Open3D REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Open3D_INCLUDE_DIRS}
)

add_library(open3d_conversions src/open3d_conversions.cpp)
target_include_directories(open3d_conversions PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  open3d_conversions
  "rclcpp"
  "sensor_msgs"
)
target_link_libraries(open3d_conversions ${Open3D_LIBRARIES})
target_compile_definitions(open3d_conversions PUBLIC "OPEN3D_CONVERSIONS_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS open3d_conversions
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_add_gtest(test_open3d_conversions test/test_open3d_conversions.cpp)
  target_link_libraries(test_open3d_conversions open3d_conversions ${Open3D_LIBRARIES})
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(open3d_conversions)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(Open3D rclcpp)

ament_package()

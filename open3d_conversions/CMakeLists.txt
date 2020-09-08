cmake_minimum_required(VERSION 3.5)
project(open3d_conversions)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

# System Dependancies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Open3D REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

#open3d_conversions package()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Open3D_INCLUDE_DIRS}
)

add_library(open3d_conversions src/open3d_conversions.cpp)
ament_target_dependencies(open3d_conversions rclcpp ${open3d_conversions_EXPORTED_TARGETS})
target_link_libraries(open3d_conversions ${Open3D_LIBRARIES})

install(TARGETS open3d_conversions
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_add_gtest(test_open3d_conversions test/test_open3d_conversions.cpp)
  target_link_libraries(test_open3d_conversions open3d_conversions ${Open3D_LIBRARIES})
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rclcpp ${Open3D_LIBRARIES})
ament_export_include_directories(include)
ament_export_libraries(open3d_conversions)
ament_package()

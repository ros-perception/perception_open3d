cmake_minimum_required(VERSION 3.0.2)
project(open3d_conversions)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Open3D REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES open3d_conversions
 CATKIN_DEPENDS roscpp sensor_msgs
 DEPENDS EIGEN3 Open3D
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Open3D_INCLUDE_DIRS}
)

# C++ library
add_library(open3d_conversions src/open3d_conversions.cpp)
add_dependencies(open3d_conversions ${open3d_conversions_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(open3d_conversions ${catkin_LIBRARIES} ${Open3D_LIBRARIES})

# Install
install(TARGETS open3d_conversions
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/open3d_conversions/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Tests
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_open3d_conversions test/test_open3d_conversions.cpp)
  target_link_libraries(test_open3d_conversions open3d_conversions ${catkin_LIBRARIES} ${Open3D_LIBRARIES})
endif()
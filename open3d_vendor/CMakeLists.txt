cmake_minimum_required(VERSION 3.5)

project(open3d_vendor)

find_package(ament_cmake REQUIRED)

option(FORCE_BUILD_VENDOR_PKG
  "Build open3d from source, even if system-installed package is available"
  OFF)

if(NOT FORCE_BUILD_VENDOR_PKG)
  find_package(Open3D QUIET)
endif()

macro(build_open3d)
  set(extra_cmake_args)

  include(ExternalProject)
  externalproject_add(open3d-0.15.1
    GIT_REPOSITORY https://github.com/isl-org/Open3D.git
    GIT_TAG ed30e3b61fbe031e106fa64030bec3f698b316b4  # v0.15.1
    UPDATE_COMMAND ""
    TIMEOUT 300
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install
      -DCMAKE_INSTALL_INCLUDEDIR=include/${PROJECT_NAME}
      -DBUILD_SHARED_LIBS=ON
      -DBUILD_EXAMPLES=OFF
      -DBUILD_PYTHON_MODULE=OFF
  )

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}
    USE_SOURCE_PERMISSIONS
  )
endmacro()

if(NOT Open3D_FOUND)
  build_open3d()
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

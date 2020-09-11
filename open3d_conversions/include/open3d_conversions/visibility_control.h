// Copyright 2020 Autonomous Robots Lab, University of Nevada, Reno

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef OPEN3D_CONVERSIONS__VISIBILITY_CONTROL_H_
#define OPEN3D_CONVERSIONS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define OPEN3D_CONVERSIONS_EXPORT __attribute__ ((dllexport))
    #define OPEN3D_CONVERSIONS_IMPORT __attribute__ ((dllimport))
  #else
    #define OPEN3D_CONVERSIONS_EXPORT __declspec(dllexport)
    #define OPEN3D_CONVERSIONS_IMPORT __declspec(dllimport)
  #endif
  #ifdef OPEN3D_CONVERSIONS_BUILDING_LIBRARY
    #define OPEN3D_CONVERSIONS_PUBLIC OPEN3D_CONVERSIONS_EXPORT
  #else
    #define OPEN3D_CONVERSIONS_PUBLIC OPEN3D_CONVERSIONS_IMPORT
  #endif
  #define OPEN3D_CONVERSIONS_PUBLIC_TYPE OPEN3D_CONVERSIONS_PUBLIC
  #define OPEN3D_CONVERSIONS_LOCAL
#else
  #define OPEN3D_CONVERSIONS_EXPORT __attribute__ ((visibility("default")))
  #define OPEN3D_CONVERSIONS_IMPORT
  #if __GNUC__ >= 4
    #define OPEN3D_CONVERSIONS_PUBLIC __attribute__ ((visibility("default")))
    #define OPEN3D_CONVERSIONS_LOCAL  __attribute__ ((visibility("default")))
  #else
    #define OPEN3D_CONVERSIONS_PUBLIC
    #define OPEN3D_CONVERSIONS_LOCAL
  #endif
  #define OPEN3D_CONVERSIONS_PUBLIC_TYPE
#endif

#endif  // OPEN3D_CONVERSIONS__VISIBILITY_CONTROL_H_

#ifndef V4L2_CAMERA__VISIBILITY_CONTROL_H_
#define V4L2_CAMERA__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define V4L2_CAMERA_EXPORT __attribute__ ((dllexport))
    #define V4L2_CAMERA_IMPORT __attribute__ ((dllimport))
  #else
    #define V4L2_CAMERA_EXPORT __declspec(dllexport)
    #define V4L2_CAMERA_IMPORT __declspec(dllimport)
  #endif
  #ifdef V4L2_CAMERA_BUILDING_LIBRARY
    #define V4L2_CAMERA_PUBLIC V4L2_CAMERA_EXPORT
  #else
    #define V4L2_CAMERA_PUBLIC V4L2_CAMERA_IMPORT
  #endif
  #define V4L2_CAMERA_PUBLIC_TYPE V4L2_CAMERA_PUBLIC
  #define V4L2_CAMERA_LOCAL
#else
  #define V4L2_CAMERA_EXPORT __attribute__ ((visibility("default")))
  #define V4L2_CAMERA_IMPORT
  #if __GNUC__ >= 4
    #define V4L2_CAMERA_PUBLIC __attribute__ ((visibility("default")))
    #define V4L2_CAMERA_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define V4L2_CAMERA_PUBLIC
    #define V4L2_CAMERA_LOCAL
  #endif
  #define V4L2_CAMERA_PUBLIC_TYPE
#endif

// For ROS2 camera package
#ifdef ROS2_V4L2_CAMERA_BUILDING_LIBRARY
  #define ROS2_V4L2_CAMERA_PUBLIC V4L2_CAMERA_EXPORT
#else
  #define ROS2_V4L2_CAMERA_PUBLIC V4L2_CAMERA_IMPORT
#endif

#endif  // V4L2_CAMERA__VISIBILITY_CONTROL_H_

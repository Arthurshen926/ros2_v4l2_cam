cmake_minimum_required(VERSION 3.5)
project(v4l2_camera)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)


install(
  DIRECTORY include/
  DESTINATION include
)


add_executable(camera_launcher_node src/camera_launcher_node.cpp)
target_include_directories(camera_launcher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(camera_launcher_node
  pthread
  )
target_compile_options(camera_launcher_node PRIVATE -Werror)
ament_target_dependencies(camera_launcher_node rclcpp sensor_msgs std_msgs OpenCV)

# Multi-camera component library
add_library(multi_camera_components
  src/camera_launcher_component.cpp
  src/multi_camera_component.cpp)
target_include_directories(multi_camera_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${YAML_CPP_INCLUDE_DIRS})
target_link_libraries(multi_camera_components ${YAML_CPP_LIBRARIES})
ament_target_dependencies(multi_camera_components
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "std_msgs"
  "image_transport"
  "OpenCV")
target_compile_options(multi_camera_components PRIVATE -Werror)

# Add compile definitions for visibility macros
target_compile_definitions(multi_camera_components PRIVATE "ROS2_V4L2_CAMERA_BUILDING_LIBRARY")

# Register components
rclcpp_components_register_nodes(multi_camera_components 
  "v4l2_camera::CameraLauncherComponent"
  "v4l2_camera::MultiCameraComponent")

# Multi-camera standalone executable
add_executable(multi_camera_standalone src/multi_camera_standalone.cpp)
target_include_directories(multi_camera_standalone PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(multi_camera_standalone multi_camera_components)
target_compile_options(multi_camera_standalone PRIVATE -Werror)
ament_target_dependencies(multi_camera_standalone rclcpp sensor_msgs std_msgs image_transport OpenCV)

# Simple example executable
add_executable(simple_multi_camera_example src/simple_multi_camera_example.cpp)
target_include_directories(simple_multi_camera_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(simple_multi_camera_example PRIVATE -Werror)
ament_target_dependencies(simple_multi_camera_example rclcpp sensor_msgs)



install(TARGETS camera_launcher_node multi_camera_standalone simple_multi_camera_example
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS multi_camera_components
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_libraries(
  v4l2_camera
  multi_camera_components
)

ament_export_dependencies(
  "camera_info_manager"
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)

cmake_minimum_required(VERSION 3.8)
project(generic_cam)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(flux_cpp REQUIRED)
find_package(flux_std_adapters REQUIRED)
find_package(JPEG REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET
  gstreamer-1.0>=1.20
  gstreamer-app-1.0>=1.20
)

add_library(generic_cam_core SHARED
  src/frame_decoder.cpp
  src/stream_monitor.cpp
  src/v4l2_device.cpp
)
target_include_directories(generic_cam_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
target_link_libraries(generic_cam_core PRIVATE PkgConfig::GST JPEG::JPEG)

add_library(generic_cam_component SHARED src/generic_cam.cpp)
target_include_directories(generic_cam_component PRIVATE ${flux_std_adapters_INCLUDE_DIRS})
target_link_libraries(generic_cam_component
  generic_cam_core
  camera_calibration_parsers::camera_calibration_parsers
  diagnostic_updater::diagnostic_updater
  flux_cpp::flux_cpp
  rclcpp::rclcpp
  rclcpp_components::component
  ${diagnostic_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
)
rclcpp_components_register_node(generic_cam_component
  PLUGIN "generic_cam::GenericCam"
  EXECUTABLE generic_cam_node
)

install(TARGETS generic_cam_core generic_cam_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # clang-format and ruff of .pre-commit-config.yaml own the style.
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_rate_status test/test_rate_status.cpp)
  target_link_libraries(test_rate_status generic_cam_core)
  ament_add_gtest(test_stream_monitor test/test_stream_monitor.cpp)
  target_link_libraries(test_stream_monitor generic_cam_core)
  ament_add_gtest(test_frame_decoder test/test_frame_decoder.cpp)
  target_link_libraries(test_frame_decoder generic_cam_core JPEG::JPEG)
  ament_add_gtest(test_v4l2_device test/test_v4l2_device.cpp)
  target_link_libraries(test_v4l2_device generic_cam_core)
  ament_add_gtest(test_capture_clock test/test_capture_clock.cpp)
  target_include_directories(test_capture_clock PRIVATE src)

  # Reproduces the numbers in doc/generic_cam.md; needs a running camera, so it is not a test.
  add_executable(latency_probe test/latency_probe.cpp)
  target_include_directories(latency_probe PRIVATE ${flux_std_adapters_INCLUDE_DIRS})
  target_link_libraries(latency_probe
    flux_cpp::flux_cpp
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
  )
  install(TARGETS latency_probe DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()

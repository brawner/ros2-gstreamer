cmake_minimum_required(VERSION 3.8)
project(ros2_gstreamer)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PkgConfig REQUIRED)

pkg_check_modules(GLIB REQUIRED glib-2.0)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER-APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GSTREAMER-AUDIO REQUIRED gstreamer-audio-1.0)
pkg_check_modules(GSTREAMER-PBUTILS REQUIRED gstreamer-pbutils-1.0)
pkg_check_modules(GSTREAMER-FFT REQUIRED gstreamer-fft-1.0)

include_directories(
  ${GLIB_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTREAMER-APP_INCLUDE_DIRS}
  ${GSTREAMER-AUDIO_INCLUDE_DIRS}
  ${GSTREAMER-PBUTILS_INCLUDE_DIRS}
  ${GSTREAMER-FFT_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  include
  ${image_transport_INCLUDE_DIRS}
)

set(all_gstreamer_libraries
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER-APP_LIBRARIES}
  ${GSTREAMER-AUDIO_LIBRARIES}
  ${GSTREAMER-PBUTILS_LIBRARIES}
  ${GSTREAMER-FFT_LIBRARIES}
)

set(plugin_dependencies rclcpp image_transport sensor_msgs)

add_library(rclcpp_publisher src/gst_rclcpp_publisher.cpp)
target_link_libraries(rclcpp_publisher ${all_gstreamer_libraries})
ament_target_dependencies(rclcpp_publisher ${plugin_dependencies})

add_library(rclcpp_subscriber src/gst_rclcpp_subscriber.cpp)
target_link_libraries(rclcpp_subscriber ${all_gstreamer_libraries})
ament_target_dependencies(rclcpp_subscriber ${plugin_dependencies})

cmake_minimum_required(VERSION 3.8)
project(camera_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_library(Raspicam_LIBRARIES raspicam_cv)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node rclcpp sensor_msgs cv_bridge)

# Link raspicam library
target_link_libraries(camera_node raspicam raspicam_cv ${OpenCV_LIBRARIES})

install(TARGETS
  camera_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()


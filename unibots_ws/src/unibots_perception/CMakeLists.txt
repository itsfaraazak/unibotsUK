cmake_minimum_required(VERSION 3.8)
project(unibots_perception)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(unibots_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ncnn REQUIRED)

add_executable(perception_node
  src/perception_node.cpp
  src/detector.cpp
)

target_include_directories(perception_node PRIVATE
  src/
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(perception_node PUBLIC
  rclcpp::rclcpp
  ament_index_cpp::ament_index_cpp
  ${sensor_msgs_TARGETS}
  ${cv_bridge_TARGETS}
  ${unibots_msgs_TARGETS}
  ncnn
  ${OpenCV_LIBS}
)

install(TARGETS perception_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models)

ament_package()

cmake_minimum_required(VERSION 3.20)
project(unibots_bt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Default to Release so the build type drives optimisation (-O3 -DNDEBUG).
# Deploy overrides CMAKE_CXX_FLAGS with Pi5 -mcpu/-flto flags (see deploy/flags.cmake).
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unibots_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)

add_executable(bt_game_node src/bt_game_node.cpp)

target_include_directories(bt_game_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Nix/Lyrical: no ament_target_dependencies — link the *_TARGETS variables.
target_link_libraries(bt_game_node PUBLIC
  rclcpp::rclcpp
  behaviortree_cpp::behaviortree_cpp
  ament_index_cpp::ament_index_cpp
  ${unibots_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

install(TARGETS bt_game_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY bt/
  DESTINATION share/${PROJECT_NAME}/bt)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()

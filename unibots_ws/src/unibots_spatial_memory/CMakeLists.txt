cmake_minimum_required(VERSION 3.20)
project(unibots_spatial_memory)

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
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Eigen3 is header-only; locate it via the Nix store path.
find_package(Eigen3 REQUIRED NO_MODULE
  HINTS "/nix/store/kki9hn7p0dc4186z31z5qz0kvaxmjk0s-eigen-3.4.1/share/eigen3/cmake")

add_executable(spatial_memory_node src/spatial_memory_node.cpp)

target_link_libraries(spatial_memory_node PUBLIC
  rclcpp::rclcpp
  ${unibots_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  Eigen3::Eigen
)

install(TARGETS spatial_memory_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()

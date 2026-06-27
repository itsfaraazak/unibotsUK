cmake_minimum_required(VERSION 3.10)
project(unibots_orchestrator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unibots_msgs REQUIRED)

add_executable(match_orchestrator src/match_orchestrator.cpp)

# Nix/Lyrical: ament_target_dependencies is not exported. Link the per-package
# ${pkg_TARGETS} variables directly (see unibots_perception/CMakeLists.txt).
target_link_libraries(match_orchestrator
  ${rclcpp_TARGETS}
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${unibots_msgs_TARGETS}
)

install(TARGETS match_orchestrator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()

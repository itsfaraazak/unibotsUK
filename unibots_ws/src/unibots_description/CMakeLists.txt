cmake_minimum_required(VERSION 3.12)
project(unibots_description)

find_package(ament_cmake REQUIRED)

# Install resource directories into the package share.
install(
  DIRECTORY
    urdf
    config
    launch
    rviz
    meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

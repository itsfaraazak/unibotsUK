cmake_minimum_required(VERSION 3.12)
project(unibots_sim)

find_package(ament_cmake REQUIRED)

# Install resource directories into the package share so launch files can
# locate the world, the model:// includes, and the launch scripts.
install(
  DIRECTORY
    worlds
    models
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

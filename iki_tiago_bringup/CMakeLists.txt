cmake_minimum_required(VERSION 3.8)
project(iki_tiago_bringup)

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  components
  sim_components
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()


cmake_minimum_required(VERSION 3.8)
project(pbr_stage)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  floorplans
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

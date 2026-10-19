cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  models
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

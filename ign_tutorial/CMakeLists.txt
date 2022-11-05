cmake_minimum_required(VERSION 3.8)
project(ign_tutorial)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
 launch
 models
 config
 DESTINATION share/${PROJECT_NAME}
)

ament_package()

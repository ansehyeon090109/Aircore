cmake_minimum_required(VERSION 3.5)
project(aircore_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(
  DIRECTORY launch urdf worlds meshes config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

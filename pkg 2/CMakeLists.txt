cmake_minimum_required(VERSION 3.5)
project(aircore_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(
  DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

# 자율주행(SLAM/Nav2) 관련 파일은 Car/ 아래에 모아뒀지만, 설치되는 share
# 경로는 그대로 유지해서 launch 파일 내부의 상대경로 로직은 손대지 않아도 됨.
install(
  DIRECTORY Car/launch Car/worlds Car/config Car/maps
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS Car/scripts/wander.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

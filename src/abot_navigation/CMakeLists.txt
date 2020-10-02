cmake_minimum_required(VERSION 3.0.2)
project(abot_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

LINK_DIRECTORIES("usr/local/lib")

# NODES
add_executable(bumper_mutex src/bumper_mutex.cpp)
target_link_libraries(bumper_mutex ${catkin_LIBRARIES})

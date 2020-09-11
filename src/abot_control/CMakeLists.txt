cmake_minimum_required(VERSION 2.8.3)
project(abot_control)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS
    controller_manager
    hardware_interface
    roscpp
    trajectory_msgs
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

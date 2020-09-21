cmake_minimum_required(VERSION 2.8.3)
project(abot_base)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  roscpp
  roslaunch
  hardware_interface
  rplidar_ros
)

catkin_package(
  CATKIN_DEPENDS 
  	controller_manager 
  	roscpp 
  	hardware_interface
    rplidar_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
  src/abot_base.cpp
)

target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

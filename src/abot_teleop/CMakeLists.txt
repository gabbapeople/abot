cmake_minimum_required(VERSION 3.0.2)
project(abot_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  joy
)

set(wiringPi_include "/usr/local/include")

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

LINK_DIRECTORIES("usr/local/lib")

# NODES
add_executable(abot_teleop src/abot_teleop.cpp)
target_link_libraries(abot_teleop ${catkin_LIBRARIES})

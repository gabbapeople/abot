cmake_minimum_required(VERSION 3.0.2)
project(abot_navigation)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  rplidar_ros
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

LINK_DIRECTORIES("usr/local/lib")

# NODES
add_executable(send_goal src/send_robot_goal.cpp)
target_link_libraries(send_goal ${catkin_LIBRARIES})

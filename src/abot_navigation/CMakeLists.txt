cmake_minimum_required(VERSION 3.0.2)
project(abot_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rplidar_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

LINK_DIRECTORIES("usr/local/lib")

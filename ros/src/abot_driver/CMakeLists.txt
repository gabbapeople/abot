cmake_minimum_required(VERSION 3.0.2)
project(abot_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

set(wiringPi_include "/usr/local/include")

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${wiringPi_include}
)

# TESTS
add_executable(friction_test src/test/friction_test.cpp)
target_link_libraries(friction_test ${catkin_LIBRARIES} -lwiringPi -lpthread -lcrypt -lm -lrt)

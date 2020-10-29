cmake_minimum_required(VERSION 3.0.2)
project(abot_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# TESTS
add_executable(dc_motor_wiring_pi_test src/test/dc_motor_wiring_pi_test.cpp)
target_link_libraries(dc_motor_wiring_pi_test ${catkin_LIBRARIES} -lwiringPi -lpthread -lcrypt -lm -lrt)

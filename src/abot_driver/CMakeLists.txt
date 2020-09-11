cmake_minimum_required(VERSION 3.0.2)
project(abot_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

set(wiringPi_include "/usr/local/include")

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${wiringPi_include}
)

LINK_DIRECTORIES("usr/local/lib")

# TESTS
add_executable(encoder_wiring_pi_test src/test/encoder_wiring_pi_test.cpp)
target_link_libraries(encoder_wiring_pi_test ${catkin_LIBRARIES} -lwiringPi -lpthread -lcrypt -lm -lrt)

add_executable(dc_motor_wiring_pi_test src/test/dc_motor_wiring_pi_test.cpp)
target_link_libraries(dc_motor_wiring_pi_test ${catkin_LIBRARIES} -lwiringPi -lpthread -lcrypt -lm -lrt)

# NODES
add_executable(encoders src/encoders.cpp)
target_link_libraries(encoders ${catkin_LIBRARIES} -lwiringPi -lpthread -lcrypt -lm -lrt)

add_executable(dc_motors src/dc_motors.cpp)
target_link_libraries(dc_motors ${catkin_LIBRARIES} -lwiringPi -lpthread -lcrypt -lm -lrt)

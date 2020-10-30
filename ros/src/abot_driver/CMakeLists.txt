cmake_minimum_required(VERSION 3.0.2)
project(abot_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(motors_friction_test src/test/motors_friction_test.cpp)
target_link_libraries(motors_friction_test ${catkin_LIBRARIES} -lwiringPi -lpthread -lcrypt -lm -lrt)

add_executable(encoders_velocity_test src/test/encoders_velocity_test.cpp)
target_link_libraries(encoders_velocity_test ${catkin_LIBRARIES} -lwiringPi -lpthread -lcrypt -lm -lrt)

add_executable(encoders src/encoders.cpp)
target_link_libraries(encoders ${catkin_LIBRARIES} -lwiringPi -lpthread -lcrypt -lm -lrt)

add_executable(dc_motors src/dc_motors.cpp)
target_link_libraries(dc_motors ${catkin_LIBRARIES} -lwiringPi -lpthread -lcrypt -lm -lrt)

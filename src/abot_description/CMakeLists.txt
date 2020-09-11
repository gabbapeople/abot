cmake_minimum_required(VERSION 2.8.3)
project(abot_description)

find_package(catkin REQUIRED COMPONENTS
  rviz
  tf
  urdf
  xacro
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

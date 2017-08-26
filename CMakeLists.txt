cmake_minimum_required(VERSION 2.8.3)
project(ymg_planner_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	base_local_planner
	costmap_2d
	geometry_msgs
	nav_core
	nav_msgs
	pluginlib
	dynamic_reconfigure
	roscpp
	tf
	)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
	cfg/YmgLP.cfg
)

catkin_package(
	#  INCLUDE_DIRS include
	#  LIBRARIES ymggp_ros
	#  CATKIN_DEPENDS costmap_2d geometry_msgs nav_core nav_msgs pluginlib roscpp tf
	#  DEPENDS system_lib
	)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	)


add_library(ymggp
	src/ymggp_ros.cpp
	)
target_link_libraries(ymggp
	${catkin_LIBRARIES}
	)

add_library(ymglp
	src/ymglp.cpp
	src/ymglp_ros.cpp
	)
target_link_libraries(ymglp
	${catkin_LIBRARIES}
	)
add_dependencies(ymglp
	dwa_local_planner_gencfg
	)


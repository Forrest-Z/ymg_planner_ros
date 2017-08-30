cmake_minimum_required(VERSION 2.8.3)
project(ymg_global_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  tf
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES ymg_global_planner
#  CATKIN_DEPENDS costmap_2d geometry_msgs nav_core nav_msgs pluginlib roscpp tf
#  DEPENDS system_lib
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

add_library(ymggp
	src/ymggp_ros.cpp
	src/ymggp.cpp
	)
target_link_libraries(ymggp
	${catkin_LIBRARIES}
	)

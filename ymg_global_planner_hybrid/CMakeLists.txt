cmake_minimum_required(VERSION 2.8.3)
project(ymg_global_planner_hybrid)

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
  navfn
  ymg_global_planner
	pluginlib
  roscpp
  tf
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES ymg_global_planner_hybrid
#  CATKIN_DEPENDS costmap_2d geometry_msgs nav_core nav_msgs navfn pluginlib roscpp tf
#  DEPENDS system_lib
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

add_library(ymggp_hybrid
	src/ymggp_hybrid_ros.cpp
	)
target_link_libraries(ymggp_hybrid
	${catkin_LIBRARIES}
	ymggp
	)

cmake_minimum_required(VERSION 2.8.3)
project(ymg_global_planner_hybrid)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  navfn
  base_local_planner
  ymg_local_planner
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
	ymggp
	${catkin_LIBRARIES}
	)

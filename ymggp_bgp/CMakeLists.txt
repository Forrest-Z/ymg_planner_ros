cmake_minimum_required(VERSION 2.8.3)
project(ymggp_bgp)

find_package(catkin REQUIRED COMPONENTS
  angles
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  global_planner
  nav_core
  nav_msgs
  navfn
  pluginlib
  roscpp
  tf
	base_local_planner
  ymg_local_planner
  ymg_global_planner
)

generate_dynamic_reconfigure_options(
	cfg/YmgGPBGP.cfg
)

catkin_package(
	INCLUDE_DIRS include
#  LIBRARIES ymggp_hyb_bgp
#  CATKIN_DEPENDS angles costmap_2d dynamic_reconfigure geometry_msgs global_planner nav_core nav_msgs navfn pluginlib roscpp tf ymg_local_planner
#  DEPENDS system_lib
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

add_library(ymggp_bgp
	src/ymggp_bgp.cpp
	)
add_dependencies(ymggp_bgp
	ymggp_bgp_gencfg
	)
target_link_libraries(ymggp_bgp
	ymggp
	${catkin_LIBRARIES}
	)

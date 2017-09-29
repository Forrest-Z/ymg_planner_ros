#include <ymg_local_planner/map_grid_cost_function_kai.h>

namespace base_local_planner {

MapGridCostFunctionKai::MapGridCostFunctionKai (costmap_2d::Costmap2D* costmap,
		bool is_local_goal_function, double forward_point_dist)
	:/*{{{*/
    costmap_(costmap),
    map_(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()),
    forward_point_dist_(forward_point_dist),
    is_local_goal_function_(is_local_goal_function),
    stop_on_failure_(false)
	{}/*}}}*/

void MapGridCostFunctionKai::setTargetPoses(std::vector<geometry_msgs::PoseStamped> target_poses)
{/*{{{*/
  target_poses_ = target_poses;
}/*}}}*/

bool MapGridCostFunctionKai::prepare ()
{/*{{{*/
  map_.resetPathDist();

  if (is_local_goal_function_) {
    map_.setLocalGoal(*costmap_, target_poses_);
  }
	else {
    map_.setTargetCells(*costmap_, target_poses_);
  }
  return true;
}/*}}}*/

double MapGridCostFunctionKai::getCellCosts (unsigned int px, unsigned int py)
{/*{{{*/
  double grid_dist = map_(px, py).target_dist;
  return grid_dist;
}/*}}}*/

// this function gets the distance from the endpoint of the trajectory to the global path and returns scaled value.
double MapGridCostFunctionKai::scoreTrajectory (Trajectory &traj)
{/*{{{*/
	if (getScale() == 0.0) {
		return 0.0;
	}

	if (!traj.getPointsSize()) {
		return -6.0;
	}

  double cost = 0.0;
  double foot_x, foot_y, foot_th;
  unsigned int foot_cell_x, foot_cell_y;
  double foot_grid_dist;

	traj.getEndpoint(foot_x, foot_y, foot_th);

	if ( ! costmap_->worldToMap(foot_x, foot_y, foot_cell_x, foot_cell_y)) {
		ROS_WARN("Off Map (foot) %f, %f", foot_x, foot_y);
		return -5.0;
	}
	
	foot_grid_dist = getCellCosts(foot_cell_x, foot_cell_y);
	cost = foot_grid_dist;

	// if forward_point_distance set positive, calc heading score
	if (0.0 < forward_point_dist_) {
		double head_x, head_y;
		unsigned int head_cell_x, head_cell_y;
		double head_grid_dist;

		int sign = 1;
		if (traj.xv_<0.0) sign = -1;

		head_x = foot_x + sign*forward_point_dist_ * cos(foot_th);
		head_y = foot_y + sign*forward_point_dist_ * sin(foot_th);

		if ( ! costmap_->worldToMap(head_x, head_y, head_cell_x, head_cell_y)) {
			ROS_WARN("Off Map (head) %f, %f", head_x, head_y);
			return -4.0;
		}

		head_grid_dist = getCellCosts(head_cell_x, head_cell_y);
		cost = (foot_grid_dist + head_grid_dist) / 2.0;
	}

	//if a point on this trajectory has no clear path to the goal... it may be invalid
	if (stop_on_failure_) {
		if (foot_grid_dist == map_.obstacleCosts()) {
			ROS_INFO("stop on failure. obstacle.");
			return -3.0;
		} else if (foot_grid_dist == map_.unreachableCellCosts()) {
			ROS_INFO("stop on failure. unreachable.");
			return -2.0;
		}
	}


  return cost * getScale();
}/*}}}*/

} /* namespace base_local_planner */

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

bool MapGridCostFunctionKai::getCost (Trajectory& traj, double& cost)
{/*{{{*/
	double x, y, th;
	unsigned int cell_x, cell_y;

	traj.getEndpoint(x, y, th);

	if ( ! costmap_->worldToMap(x, y, cell_x, cell_y)) {
		ROS_WARN("Off Map (foot) %f, %f", x, y);
		cost = -1.0;
		return false;
	}

	cost = getCellCosts(cell_x, cell_y);
	return true;
}/*}}}*/

bool MapGridCostFunctionKai::getForwardCost (Trajectory& traj, double& cost)
{/*{{{*/
	if (forward_point_dist_ < 0.0) {
		return false;
	}

	double base_x, base_y, base_th;
	double x, y;
	unsigned int cell_x, cell_y;
	traj.getEndpoint(base_x, base_y, base_th);

	int sign = 1;
	if (traj.xv_<0.0) sign = -1;

	x = base_x + sign*forward_point_dist_ * cos(base_th);
	y = base_y + sign*forward_point_dist_ * sin(base_th);

	if ( ! costmap_->worldToMap(x, y, cell_x, cell_y)) {
		ROS_WARN("Off Map (head) %f, %f", x, y);
		cost = -1.0;
		return false;
	}

	cost = getCellCosts(cell_x, cell_y);
	return true;
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

	double cost ,b_cost, f_cost;
	if (!getCost(traj, b_cost)) {
		return -5.0;
	}

	if (getForwardCost(traj, f_cost)) {
		cost = (b_cost + f_cost) / 2.0;
	}
	else {
		cost = b_cost;
	}

	//if a point on this trajectory has no clear path to the goal... it may be invalid
	if (stop_on_failure_) {
		if (b_cost == map_.obstacleCosts()) {
			ROS_INFO("stop on failure. obstacle.");
			return -3.0;
		} else if (b_cost == map_.unreachableCellCosts()) {
			ROS_INFO("stop on failure. unreachable.");
			return -2.0;
		}
	}


	return cost * getScale();
}/*}}}*/

} /* namespace base_local_planner */

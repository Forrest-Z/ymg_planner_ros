#include <ymg_local_planner/map_grid_cost_function_kai.h>

namespace base_local_planner {

MapGridCostFunctionKai::MapGridCostFunctionKai (costmap_2d::Costmap2D* costmap,
		bool is_local_goal_function, double forward_point_distance)
	:/*{{{*/
    costmap_(costmap),
    map_(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()),
    forward_point_distance_(forward_point_distance),
    is_local_goal_function_(is_local_goal_function),
    stop_on_failure_(true),
		valid_traj_ratio_(1.0)
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
  } else {
    map_.setTargetCells(*costmap_, target_poses_);
  }
  return true;
}/*}}}*/

double MapGridCostFunctionKai::getCellCosts (unsigned int px, unsigned int py)
{/*{{{*/
  double grid_dist = map_(px, py).target_dist;
  return grid_dist;
}/*}}}*/

double MapGridCostFunctionKai::scoreTrajectory (Trajectory &traj)
{/*{{{*/
  double cost = 0.0;
  double foot_x, foot_y, foot_th;
  unsigned int foot_cell_x, foot_cell_y;
  double foot_grid_dist;

	if (!traj.getPointsSize()) {
		// ROS_ERROR("trajectory size is zero");
		return -6.0;
	}

	// int score_traj_index = (traj.getPointsSize()-1) * valid_traj_ratio_; 
	// std::cout<<"traj_size - score_traj_index = "<<traj.getPointsSize()<<" - "<<score_traj_index<<std::endl;
	// traj.getPoint(score_traj_index, foot_x, foot_y, foot_th);
	traj.getEndpoint(foot_x, foot_y, foot_th);

	if ( ! costmap_->worldToMap(foot_x, foot_y, foot_cell_x, foot_cell_y)) {
		ROS_WARN("Off Map (foot) %f, %f", foot_x, foot_y);
		return -5.0;
	}
	
	foot_grid_dist = getCellCosts(foot_cell_x, foot_cell_y);
	cost = foot_grid_dist;

	// if forward_point_distance set positive, calc heading score
	if (0.0 < forward_point_distance_) {
		double head_x, head_y;
		unsigned int head_cell_x, head_cell_y;
		double head_grid_dist;

		head_x = foot_x + forward_point_distance_ * cos(foot_th);
		head_y = foot_y + forward_point_distance_ * sin(foot_th);

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
			return -3.0;
		} else if (foot_grid_dist == map_.unreachableCellCosts()) {
			return -2.0;
		}
	}


  return cost;
}/*}}}*/

} /* namespace base_local_planner */

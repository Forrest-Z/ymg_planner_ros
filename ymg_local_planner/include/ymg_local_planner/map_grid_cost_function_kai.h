#ifndef MAP_GRID_COST_FUNCTION_KAI_H_
#define MAP_GRID_COST_FUNCTION_KAI_H_

#include <base_local_planner/trajectory_cost_function.h>

#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/map_grid.h>

namespace base_local_planner {

/**
 * This class provides cost based on a map_grid of a small area of the world.
 * The map_grid covers a the costmap, the costmap containing the information
 * about sensed obstacles. The map_grid is used by setting
 * certain cells to distance 0, and then propagating distances around them,
 * filling up the area reachable around them.
 *
 * The approach using grid_maps is used for computational efficiency, allowing to
 * score hundreds of trajectories very quickly.
 *
 * This can be used to favor trajectories which stay on a given path, or which
 * approach a given goal.
 * @param costmap_ros Reference to object giving updates of obstacles around robot
 * @param is_local_goal_function, scores for local goal rather than whole path
 */
class MapGridCostFunctionKai: public base_local_planner::TrajectoryCostFunction {
public:
  MapGridCostFunctionKai(costmap_2d::Costmap2D* costmap,
			bool is_local_goal_function = false, double forward_point_distance = -1.0);

  ~MapGridCostFunctionKai() {}

  /**
   * set line segments on the grid with distance 0, resets the grid
   */
  void setTargetPoses(std::vector<geometry_msgs::PoseStamped> target_poses);
  
  void setForwardPointDistance(double forward_point_distance) {forward_point_distance_ = forward_point_distance;}
  
	void setValidTrajRatio(double valid_traj_ratio) {valid_traj_ratio_ = valid_traj_ratio;}

  /** @brief If true, failures along the path cause the entire path to be rejected.
   *
   * Default is true. */
  void setStopOnFailure(bool stop_on_failure) {stop_on_failure_ = stop_on_failure;}

  /**
   * propagate distances
   */
  bool prepare();

  double scoreTrajectory(Trajectory &traj);

  /**
   * return a value that indicates cell is in obstacle
   */
  double obstacleCosts() {
    return map_.obstacleCosts();
  }

  /**
   * returns a value indicating cell was not reached by wavefront
   * propagation of set cells. (is behind walls, regarding the region covered by grid)
   */
  double unreachableCellCosts() {
    return map_.unreachableCellCosts();
  }

  // used for easier debugging
  double getCellCosts(unsigned int cx, unsigned int cy);

private:
  std::vector<geometry_msgs::PoseStamped> target_poses_;
  costmap_2d::Costmap2D* costmap_;

  base_local_planner::MapGrid map_;

  double forward_point_distance_;
  // if true, we look for a suitable local goal on path, else we use the full path for costs
  bool is_local_goal_function_;
  bool stop_on_failure_;

	double valid_traj_ratio_;
};

} /* namespace base_local_planner */
#endif /* MAP_GRID_COST_FUNCTION_H_ */

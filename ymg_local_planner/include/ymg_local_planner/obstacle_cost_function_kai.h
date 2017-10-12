#ifndef OBSTACLE_COST_FUNCTION_KAI_H_
#define OBSTACLE_COST_FUNCTION_KAI_H_

#include <base_local_planner/trajectory_cost_function.h>

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d.h>
#include <ymg_local_planner/util_functions.h>

namespace base_local_planner {

/**
 * class ObstacleCostFunctionKai
 * @brief Uses costmap 2d to assign negative costs if robot footprint
 * is in obstacle on any point of the trajectory.
 */
class ObstacleCostFunctionKai : public TrajectoryCostFunction {

public:
  ObstacleCostFunctionKai(costmap_2d::Costmap2D* costmap,
			double forward_point_dist = -1.0, double sim_granularity = 0.025);
  ~ObstacleCostFunctionKai();

  bool prepare();
  double scoreTrajectory(Trajectory &traj);
  double scoreTrajectory(Trajectory &traj, bool scaling_flag);

  void setParams(double max_vel_abs, double max_scaling_factor, double scaling_speed);
  void setFootprint(std::vector<geometry_msgs::Point> footprint_spec);
	void setSimGranularity (double sim_granularity) { sim_granularity_ = sim_granularity; }
	void setForwardPointDist (double forward_point_dist) { forward_point_dist_ = forward_point_dist; }

  // helper functions, made static for easy unit testing
  static double getScalingFactor(Trajectory &traj, double scaling_speed, double max_vel_abs, double max_scaling_factor);
  static double footprintCost(
      const double& x,
      const double& y,
      const double& th,
      double scale,
      std::vector<geometry_msgs::Point> footprint_spec,
      costmap_2d::Costmap2D* costmap,
      base_local_planner::WorldModel* world_model);

private:
  costmap_2d::Costmap2D* costmap_;
  std::vector<geometry_msgs::Point> footprint_spec_;
  base_local_planner::WorldModel* world_model_;
  double max_vel_abs_;

  //footprint scaling with velocity;
	bool scaling_flag_;
  double max_scaling_factor_, scaling_speed_;
	double sim_granularity_, forward_point_dist_;
	bool isZero(double x);
};

} /* namespace base_local_planner */
#endif /* OBSTACLE_COST_FUNCTION_H_ */

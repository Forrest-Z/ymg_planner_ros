#ifndef OBSTACLE_COST_FUNCTION_KAI_H_
#define OBSTACLE_COST_FUNCTION_KAI_H_

#include <base_local_planner/trajectory_cost_function.h>

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d.h>

namespace base_local_planner {

/**
 * class ObstacleCostFnctionKai
 * @brief Uses costmap 2d to assign negative costs if robot footprint
 * is in obstacle on any point of the trajectory.
 */
class ObstacleCostFnctionKai : public TrajectoryCostFunction {

public:
  ObstacleCostFnctionKai(costmap_2d::Costmap2D* costmap,
			double additional_sim_time = -1.0, double sim_granularity = 0.025);
  ~ObstacleCostFnctionKai();

  bool prepare();
  double scoreTrajectory(Trajectory &traj);

  void setSumScores(bool score_sums){ sum_scores_=score_sums; }

  void setParams(double max_trans_vel, double max_scaling_factor, double scaling_speed);
  void setFootprint(std::vector<geometry_msgs::Point> footprint_spec);

  // helper functions, made static for easy unit testing
  static double getScalingFactor(Trajectory &traj, double scaling_speed, double max_trans_vel, double max_scaling_factor);
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
  double max_trans_vel_;
  bool sum_scores_;
  //footprint scaling with velocity;
  double max_scaling_factor_, scaling_speed_;

	double additional_sim_time_, sim_granularity_;
};

} /* namespace base_local_planner */
#endif /* OBSTACLE_COST_FUNCTION_H_ */

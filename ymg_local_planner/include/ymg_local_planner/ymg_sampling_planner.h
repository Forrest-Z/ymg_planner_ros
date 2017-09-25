#ifndef YMG_SAMPLING_PLANNER_H_
#define YMG_SAMPLING_PLANNER_H_

#include <vector>
#include <Eigen/Core>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/trajectory_cost_function.h>
#include <base_local_planner/trajectory_sample_generator.h>
#include <base_local_planner/trajectory_search.h>

namespace ymglp {

/**
 * @class YmgSamplingPlanner
 * @brief Generates a local plan using the given generator and cost functions.
 * Assumes less cost are best, and negative costs indicate infinite costs
 *
 * This is supposed to be a simple and robust implementation of
 * the TrajectorySearch interface. More efficient search may well be
 * possible using search heuristics, parallel search, etc.
 */
class YmgSamplingPlanner {

public:

  ~YmgSamplingPlanner() {}
  YmgSamplingPlanner() {}
  YmgSamplingPlanner(
			base_local_planner::TrajectoryCostFunction* pdist_critics,
			base_local_planner::TrajectoryCostFunction* obstacle_critics);

  void initialize(
      base_local_planner::LocalPlannerLimits* limits,
      const Eigen::Vector3f& pos,
      const Eigen::Vector3f& vel,
      const Eigen::Vector3f& vsamples);

	void setParameters(
		double sim_time, double sim_granularity, double angular_sim_granularity, double sim_period);

  /**
   * Calls generator until generator has no more samples or max_samples is reached.
   * For each generated traj, calls critics in turn. If any critic returns negative
   * value, that value is assumed as costs, else the costs are the sum of all critics
   * result. Returns true and sets the traj parameter to the first trajectory with
   * minimal non-negative costs if sampling yields trajectories with non-negative costs,
   * else returns false.
   *
   * @param traj The container to write the result to
   * @param all_explored pass NULL or a container to collect all trajectories for debugging (has a penalty)
   */
  bool findBestTrajectory(
			base_local_planner::Trajectory& traj, std::vector<base_local_planner::Trajectory>* all_explored = 0);


private:
	bool generateTrajectory(
			Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f sample_target_vel,
			base_local_planner::Trajectory& traj);

	Eigen::Vector3f computeNewPositions(
			const Eigen::Vector3f& pos, const Eigen::Vector3f& vel, double dt);

  double sim_time_, sim_granularity_, angular_sim_granularity_, sim_period_;
  base_local_planner::LocalPlannerLimits* limits_;
	Eigen::Vector3f pos_, vel_, vsamples_;
	Eigen::Vector3f max_vel_, min_vel_;
	double pdist_tolerance_;

	base_local_planner::TrajectoryCostFunction* pdist_critics_;
	base_local_planner::TrajectoryCostFunction* obstacle_critics_;

  int max_samples_;
};




} // namespace

#endif /* SIMPLE_SCORED_SAMPLING_PLANNER_H_ */

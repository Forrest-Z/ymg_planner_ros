#ifndef YMG_SAMPLING_PLANNER_H_
#define YMG_SAMPLING_PLANNER_H_

#include <vector>
#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/trajectory_cost_function.h>
#include <base_local_planner/trajectory_sample_generator.h>
#include <base_local_planner/trajectory_search.h>
#include <ymg_local_planner/map_grid_cost_function_kai.h>
#include <ymg_local_planner/obstacle_cost_function_kai.h>

namespace ymglp {

class YmgSamplingPlanner {

	public:

		~YmgSamplingPlanner() {}
		YmgSamplingPlanner() {}
		YmgSamplingPlanner(
				base_local_planner::MapGridCostFunctionKai* path_critic,
				base_local_planner::ObstacleCostFunctionKai* obstacle_critic);

		void initialize(
				base_local_planner::LocalPlannerLimits* limits,
				const Eigen::Vector3f& pos,
				const Eigen::Vector3f& vel,
				const Eigen::Vector3f& vsamples);

		void setParameters(double sim_time, double sim_granularity, double angular_sim_granularity, double sim_period)
		{/*{{{*/
			sim_time_ = sim_time;
			sim_granularity_ = sim_granularity;
			angular_sim_granularity_ = angular_sim_granularity;
			sim_period_ = sim_period;
		}/*}}}*/

		void setTolerance(double path_tolerance, int obstacle_tolerance)
		{/*{{{*/
			path_tolerance_ = path_tolerance;
			obstacle_tolerance_ = obstacle_tolerance;
		}/*}}}*/

		bool findBestTrajectory(
				base_local_planner::Trajectory& traj, std::vector<base_local_planner::Trajectory>* all_explored = 0);


	private:

		base_local_planner::Trajectory generateClosestTrajectory(double vel_x);

		bool generateTrajectory(
				Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f sample_target_vel,
				base_local_planner::Trajectory& traj);

		Eigen::Vector3f computeNewPositions(
				const Eigen::Vector3f& pos, const Eigen::Vector3f& vel, double dt);

		bool reverse_order_;
		double sim_time_, sim_granularity_, angular_sim_granularity_, sim_period_;
		base_local_planner::LocalPlannerLimits* limits_;
		Eigen::Vector3f pos_, vel_, vsamples_;
		Eigen::Vector3f max_vel_, min_vel_;

		double path_tolerance_;
		int obstacle_tolerance_;
		base_local_planner::MapGridCostFunctionKai* path_critic_;
		base_local_planner::ObstacleCostFunctionKai* obstacle_critic_;

};   // class YmgSamplingPlanner

} // namespace ymglp

#endif

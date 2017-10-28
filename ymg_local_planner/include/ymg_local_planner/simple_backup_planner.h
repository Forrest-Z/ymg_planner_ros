#ifndef SIMPLE_BACKUP_PLANNER_H_
#define SIMPLE_BACKUP_PLANNER_H_

#include <vector>
#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/trajectory_cost_function.h>
#include <base_local_planner/trajectory_sample_generator.h>
#include <base_local_planner/trajectory_search.h>
#include <ymg_local_planner/map_grid_cost_function_kai.h>
#include <ymg_local_planner/util_functions.h>
#include <ymg_local_planner/obstacle_cost_function_kai.h>

namespace ymglp {

class SimpleBackupPlanner {

	public:

		SimpleBackupPlanner() {}
		SimpleBackupPlanner(
				base_local_planner::ObstacleCostFunctionKai* obstacle_critic);

		void initialize(
				base_local_planner::LocalPlannerLimits* limits,
				const Eigen::Vector3f& pos,
				const Eigen::Vector3f& vel);

		void setParameters(
				double sim_time, double sim_granularity, double angular_sim_granularity, double sim_period,
				int obstacle_tolerance, double backup_vel);

		bool findBestTrajectory(
				base_local_planner::Trajectory& traj, std::vector<base_local_planner::Trajectory>* all_explored = 0);


	private:

		base_local_planner::Trajectory generateClosestTrajectory(double vel_x);

		bool generateTrajectory(
				Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f sample_target_vel,
				base_local_planner::Trajectory& traj);

		Eigen::Vector3f computeNewPositions(
				const Eigen::Vector3f& pos, const Eigen::Vector3f& vel, double dt);

		bool is_param_set_;
		double sim_time_, sim_granularity_, angular_sim_granularity_, sim_period_, backup_vel_;
		int obstacle_tolerance_;

		base_local_planner::LocalPlannerLimits* limits_;
		Eigen::Vector3f pos_, vel_, target_vel_;

		base_local_planner::ObstacleCostFunctionKai* obstacle_critic_;

};   // class SimpleBackupPlanner

} // namespace ymglp

#endif

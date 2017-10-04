#ifndef DIRECTION_ADJUST_PLANNER_H_
#define DIRECTION_ADJUST_PLANNER_H_

#include <vector>
#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/trajectory_cost_function.h>
#include <base_local_planner/trajectory_sample_generator.h>
#include <base_local_planner/trajectory_search.h>

namespace ymglp {

class DirAdjustPlanner {

	public:

		~DirAdjustPlanner() {}
		DirAdjustPlanner() {}
		DirAdjustPlanner(base_local_planner::TrajectoryCostFunction* obstacle_critic);

		void initialize(
				base_local_planner::LocalPlannerLimits* limits,
				const Eigen::Vector3f& pos,
				const Eigen::Vector3f& vel,
				double direction_error);

		void setParameters(double sim_time, double sim_granularity, double angular_sim_granularity, double sim_period)
		{/*{{{*/
			sim_time_ = sim_time;
			sim_granularity_ = sim_granularity;
			angular_sim_granularity_ = angular_sim_granularity;
			sim_period_ = sim_period;
		}/*}}}*/

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
		Eigen::Vector3f pos_, vel_, target_vel_;

		base_local_planner::TrajectoryCostFunction* obstacle_critic_;
};  // class DirAdjustPlanner

} // namespace

#endif 

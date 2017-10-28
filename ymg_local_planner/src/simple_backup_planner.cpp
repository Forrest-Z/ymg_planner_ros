#include <ymg_local_planner/simple_backup_planner.h>

// #define DEBUG

namespace ymglp {

SimpleBackupPlanner::SimpleBackupPlanner(
		base_local_planner::ObstacleCostFunctionKai* obstacle_critic)
	: is_param_set_(false)
{/*{{{*/
	obstacle_critic_ = obstacle_critic;
}/*}}}*/

void SimpleBackupPlanner::setParameters(
		double sim_time, double sim_granularity, double angular_sim_granularity, double sim_period,
		int obstacle_tolerance, double backup_vel)
{/*{{{*/
	is_param_set_ = true;

	sim_time_ = sim_time;
	sim_granularity_ = sim_granularity;
	angular_sim_granularity_ = angular_sim_granularity;
	sim_period_ = sim_period;
	obstacle_tolerance_ = obstacle_tolerance;
	backup_vel_ = backup_vel;
}/*}}}*/

void SimpleBackupPlanner::initialize(
		base_local_planner::LocalPlannerLimits* limits,
		const Eigen::Vector3f& pos,
		const Eigen::Vector3f& vel)
{/*{{{*/
	pos_ = pos;
	vel_ = vel;

	Eigen::Vector3f acc_lim = limits->getAccLimits();

	target_vel_[0] = std::max(backup_vel_, vel[0] - acc_lim[0] * sim_period_);
	target_vel_[1] = 0.0;
	if (0.0 < vel[2]) {
		target_vel_[2] = std::max(0.0, vel[2] - acc_lim[2] * sim_period_);
	}
	else {
		target_vel_[2] = std::min(0.0, vel[2] + acc_lim[2] * sim_period_);
	}

}/*}}}*/

bool SimpleBackupPlanner::findBestTrajectory(
		base_local_planner::Trajectory& traj, std::vector<base_local_planner::Trajectory>* all_explored)
{/*{{{*/
	if (!is_param_set_) {
		ROS_WARN("[YSP] Has not set parameters. Please call setParameters() fcn.");
		return false;
	}

	if (obstacle_critic_->prepare() == false) {
		ROS_WARN("Obstacle scoring function failed to prepare");
		return false;
	}

	generateTrajectory(pos_, vel_, target_vel_, traj);
	traj.cost_ = obstacle_critic_->scoreTrajectory(traj);

	if (0.0 <= traj.cost_) {
		return true;
	}

	traj.resetPoints();
	traj.xv_ = 0.0;
	traj.yv_ = 0.0;
	traj.thetav_ = 0.0;
	traj.cost_ = 77.7;
	traj.addPoint(pos_[0], pos_[1], pos_[2]);

	return false;
}/*}}}*/

bool SimpleBackupPlanner::generateTrajectory(
		Eigen::Vector3f pos,
		Eigen::Vector3f vel,
		Eigen::Vector3f sample_target_vel,
		base_local_planner::Trajectory& traj)
{/*{{{*/
	traj.cost_   = -1.0;   // placed here in case we return early
	traj.resetPoints();   //trajectory might be reused so we'll make sure to reset it
	traj.addPoint(pos[0], pos[1], pos[2]);   // add now point

	int num_steps;
	double sim_time_distance = sample_target_vel[0] * sim_time_; // the distance the robot would travel in sim_time
	double sim_time_angle = fabs(sample_target_vel[2]) * sim_time_; // the angle the robot would rotate in sim_time
	num_steps = ceil(std::max(sim_time_distance / sim_granularity_, sim_time_angle / angular_sim_granularity_));

	//compute a timestep
	double dt = sim_time_ / num_steps;
	traj.time_delta_ = dt;

	traj.xv_     = sample_target_vel[0];
	traj.yv_     = sample_target_vel[1];
	traj.thetav_ = sample_target_vel[2];

	// simulate the trajectory and check for collisions, updating costs along the way
	for (int i = 0; i < num_steps; ++i) {
		pos = computeNewPositions(pos, sample_target_vel, dt);
		traj.addPoint(pos[0], pos[1], pos[2]);
	} // end for simulation steps

	return traj.getPointsSize() > 0; // true if trajectory has at least one point
}/*}}}*/

Eigen::Vector3f SimpleBackupPlanner::computeNewPositions(
		const Eigen::Vector3f& pos, const Eigen::Vector3f& vel, double dt)
{/*{{{*/
	Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
	new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
	new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
	new_pos[2] = pos[2] + vel[2] * dt;
	return new_pos;
}/*}}}*/

}// namespace

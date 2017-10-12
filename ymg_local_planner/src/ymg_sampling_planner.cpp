#include <ymg_local_planner/ymg_sampling_planner.h>
#include <ros/console.h>

namespace ymglp {

YmgSamplingPlanner::YmgSamplingPlanner(
		base_local_planner::MapGridCostFunctionKai* path_critic,
		base_local_planner::ObstacleCostFunctionKai* obstacle_critic,
		UtilFcn* utilfcn)
	: is_param_set_(false), reverse_order_(false)
{/*{{{*/
	path_critic_ = path_critic;
	utilfcn_ = utilfcn;
	obstacle_critic_ = obstacle_critic;
}/*}}}*/

void YmgSamplingPlanner::setParameters(
		double sim_time, double sim_granularity, double angular_sim_granularity, double sim_period,
		double path_tolerance, int obstacle_tolerance)
{/*{{{*/
	is_param_set_ = true;

	sim_time_ = sim_time;
	sim_granularity_ = sim_granularity;
	angular_sim_granularity_ = angular_sim_granularity;
	sim_period_ = sim_period;
	path_tolerance_ = path_tolerance;
	obstacle_tolerance_ = obstacle_tolerance;
}/*}}}*/

void YmgSamplingPlanner::initialize(
		base_local_planner::LocalPlannerLimits* limits,
		const Eigen::Vector3f& pos,
		const Eigen::Vector3f& vel,
		const Eigen::Vector3f& vsamples)
{/*{{{*/
	pos_ = pos;
	vel_ = vel;
	vsamples_ = vsamples;
	limits_ = limits;

	double max_vel_th = limits->max_rot_vel;
	double min_vel_th = -1.0 * max_vel_th;
	Eigen::Vector3f acc_lim = limits->getAccLimits();
	double min_vel_x = limits->min_vel_x;
	double max_vel_x = limits->max_vel_x;
	double min_vel_y = limits->min_vel_y;
	double max_vel_y = limits->max_vel_y;

	max_vel_[0] = std::min(max_vel_x, vel[0] + acc_lim[0] * sim_period_);
	max_vel_[1] = std::min(max_vel_y, vel[1] + acc_lim[1] * sim_period_);
	max_vel_[2] = std::min(max_vel_th, vel[2] + acc_lim[2] * sim_period_);

	min_vel_[0] = std::max(min_vel_x, vel[0] - acc_lim[0] * sim_period_);
	min_vel_[1] = std::max(min_vel_y, vel[1] - acc_lim[1] * sim_period_);
	min_vel_[2] = std::max(min_vel_th, vel[2] - acc_lim[2] * sim_period_);

	// ROS_INFO("vel range : %f to %f", min_vel_[0], max_vel_[0]);

	double max_vel_abs = std::max(fabs(max_vel_[0]), fabs(min_vel_[0]));
	utilfcn_->setSearchDist(max_vel_abs * sim_time_); 

	if (fabs(max_vel_x) < fabs(min_vel_x))
		reverse_order_ = true;
	else
		reverse_order_ = false;

	// ROS_INFO("max - min : %f - %f", max_vel_[0], min_vel_[0]);
}/*}}}*/

bool YmgSamplingPlanner::findBestTrajectory(
		base_local_planner::Trajectory& traj,
		std::vector<base_local_planner::Trajectory>* all_explored)
{/*{{{*/
	if (!is_param_set_) {
		ROS_WARN("[YSP] Has not set parameters. Please call setParameters() fcn.");
		return false;
	}

	if (path_critic_->prepare() == false) {
		ROS_WARN("Pdist scoring function failed to prepare");
		return false;
	}
	if (obstacle_critic_->prepare() == false) {
		ROS_WARN("Obstacle scoring function failed to prepare");
		return false;
	}

	double v_step = (max_vel_[0] - min_vel_[0]) / vsamples_[0];
	base_local_planner::Trajectory comp_traj, better_traj;
	better_traj.cost_ = DBL_MAX;

	double start_vel_x = max_vel_[0];
	if (reverse_order_) {
		start_vel_x = min_vel_[0];
		v_step *= -1;
	}

	double now_dist = utilfcn_->getForwardPointPathDist();

	// trajectory.cost_ is the distance from global path.
	double target_vel_x;
	for (int i=0; i<=vsamples_[0]; ++i) {
		target_vel_x = start_vel_x - i * v_step;

		comp_traj = generateClosestTrajectory(target_vel_x);
		// cannot generate trajectory
		if (comp_traj.cost_ < 0.0) {
			continue;
		}

		double obstacle_cost = obstacle_critic_->scoreTrajectory(comp_traj);
		// if the trajectory hit obstacles
		if (obstacle_cost < 0.0 || obstacle_tolerance_ < obstacle_cost) {
			continue;
		}

		// find closest path
		if (comp_traj.cost_ < path_tolerance_) {
			traj = comp_traj;
			// ROS_INFO("[YggSampPl] cost_p : %f", traj.cost_);
			return true;
		}

		// far from global path and can approach global path
		else if (comp_traj.cost_ < now_dist && comp_traj.cost_<better_traj.cost_) {
			better_traj = comp_traj;
		}
	}

	// find better path (closest to the global plan)
	if (!better_traj.cost_==DBL_MAX) {
		traj = better_traj;
		return true;
	}

	// connot find valid path, try to calc slowest velocity with no scaling
	double obstacle_cost = obstacle_critic_->scoreTrajectory(comp_traj, false);
	if (0.0 <= obstacle_cost && obstacle_cost<=obstacle_tolerance_) {
		ROS_INFO("[YSP] no scaling path found.");
		traj = comp_traj;
		return true;
	}

	traj.resetPoints();
	traj.xv_ = 0.0;
	traj.yv_ = 0.0;
	traj.thetav_ = 0.0;
	traj.cost_ = 77.7;
	traj.addPoint(pos_[0], pos_[1], pos_[2]);
	ROS_INFO("[YSP] failed to find valid path. (send zero velocity)");

	return false;
}/*}}}*/

base_local_planner::Trajectory YmgSamplingPlanner::generateClosestTrajectory(double vel_x)
{/*{{{*/
	base_local_planner::Trajectory best_traj;
	best_traj.cost_ = DBL_MAX;

	Eigen::Vector3f target_vel;
	target_vel[0] = vel_x;
	target_vel[1] = 0.0;   // velocity_y must be zero
	double w_step = (max_vel_[2] - min_vel_[2]) / vsamples_[2];

	for (int i=0; i<=vsamples_[2]; ++i) {
		target_vel[2] = max_vel_[2] - i * w_step;
		base_local_planner::Trajectory comp_traj;
		if(!generateTrajectory(pos_, vel_, target_vel, comp_traj)) {
			continue;
		}

		// comp_traj.cost_ = path_critic_->scoreTrajectory(comp_traj);
		if (0) {
		// if (UtilFcn::isZero(comp_traj.xv_)) {
			comp_traj.cost_ = utilfcn_->scoreTrajForwardDist(comp_traj, reverse_order_);
		}
		else {
			comp_traj.cost_ = utilfcn_->scoreTrajDist(comp_traj);
		}
		// ROS_INFO("[closest] cost : %f", comp_traj.cost_);
		if (0.0<=comp_traj.cost_ && comp_traj.cost_<best_traj.cost_) {
			best_traj = comp_traj;
		}
	}
	// ROS_INFO("roop end");

	// cannot find closest path
	if (best_traj.cost_ == DBL_MAX) {
		best_traj.cost_ = -1.0;
	}

	return best_traj;
}/*}}}*/

bool YmgSamplingPlanner::generateTrajectory(
		Eigen::Vector3f pos,
		Eigen::Vector3f vel,
		Eigen::Vector3f sample_target_vel,
		base_local_planner::Trajectory& traj)
{/*{{{*/
	traj.cost_   = -1.0;   // placed here in case we return early
	traj.resetPoints();   //trajectory might be reused so we'll make sure to reset it

	int num_steps;
	if (angular_sim_granularity_ < 0.0) {
		num_steps = ceil(sim_time_ / sim_granularity_);
	} else {
		//compute the number of steps we must take along this trajectory to be "safe"
		double sim_time_distance = sample_target_vel[0] * sim_time_; // the distance the robot would travel in sim_time if it did not change velocity
		double sim_time_angle = fabs(sample_target_vel[2]) * sim_time_; // the angle the robot would rotate in sim_time
		num_steps =
			ceil(std::max(sim_time_distance / sim_granularity_, sim_time_angle / angular_sim_granularity_));
	}

	//compute a timestep
	double dt = sim_time_ / num_steps;
	traj.time_delta_ = dt;

	traj.xv_     = sample_target_vel[0];
	traj.yv_     = sample_target_vel[1];
	traj.thetav_ = sample_target_vel[2];

	//simulate the trajectory and check for collisions, updating costs along the way
	for (int i = 0; i < num_steps; ++i) {

		//add the point to the trajectory so we can draw it later if we want
		traj.addPoint(pos[0], pos[1], pos[2]);

		//update the position of the robot using the velocities passed in
		pos = computeNewPositions(pos, sample_target_vel, dt);

	} // end for simulation steps

	return num_steps > 0; // true if trajectory has at least one point
}/*}}}*/

Eigen::Vector3f YmgSamplingPlanner::computeNewPositions(
		const Eigen::Vector3f& pos, const Eigen::Vector3f& vel, double dt)
{/*{{{*/
	Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
	new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
	new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
	new_pos[2] = pos[2] + vel[2] * dt;
	return new_pos;
}/*}}}*/

}// namespace

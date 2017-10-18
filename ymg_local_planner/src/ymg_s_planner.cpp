#include <ymg_local_planner/ymg_s_planner.h>
#include <ros/console.h>

// #define DEBUG

namespace ymglp {

YmgSPlanner::YmgSPlanner(
		base_local_planner::MapGridCostFunctionKai* path_critic,
		base_local_planner::ObstacleCostFunctionKai* obstacle_critic,
		UtilFcn* utilfcn)
	: is_param_set_(false), reverse_order_(false)
{/*{{{*/
	path_critic_ = path_critic;
	utilfcn_ = utilfcn;
	obstacle_critic_ = obstacle_critic;
}/*}}}*/

void YmgSPlanner::setParameters(
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

void YmgSPlanner::initialize(
		base_local_planner::LocalPlannerLimits* limits,
		const Eigen::Vector3f& pos,
		const Eigen::Vector3f& vel,
		const Eigen::Vector3f& vsamples)
{/*{{{*/
	pos_ = pos;
	vel_ = vel;
	vsamples_ = vsamples;
	limits_ = limits;

	target_curvature_ = getTragetCurvature();

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

bool YmgSPlanner::findBestTrajectory(
		base_local_planner::Trajectory& traj, std::vector<base_local_planner::Trajectory>* all_explored)
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

	Eigen::Vector3f target_vel;
	target_vel[1] = 0.0;   // velocity y must be zero
	
	if (max_vel_[2] < target_curvature_ * min_vel_[0]) {
		target_vel[0] = min_vel_[0];
		target_vel[2] = max_vel_[2];
	}
	else if (target_curvature_ * min_vel_[0] < min_vel_[2]) {
		target_vel[0] = min_vel_[0];
		target_vel[2] = min_vel_[2];
	}
	else {

		double target_omega = target_curvature_ * max_vel_[0];
		if (max_vel_[2] < target_omega) {
			target_vel[2] = max_vel_[2];
			target_vel[0] = target_vel[2] / target_curvature_;
		}
		else if (target_omega < min_vel_[2]) {
			target_vel[2] = min_vel_[2];
			target_vel[0] = target_vel[2] / target_curvature_;
		}
		else {
			target_vel[0] = max_vel_[0];
			target_vel[2] = target_vel[0] * target_curvature_;
		}

	}

	ROS_INFO("target_curvature : %f", target_curvature_);
	ROS_INFO("vel : %f, %f, %f", target_vel[0], target_vel[1], target_vel[2]);

	if(!generateTrajectory(pos_, vel_, target_vel, traj)) {
		traj.cost_ = -1.0;
	}
	else {
		traj.cost_ = 1.0;
	}
}/*}}}*/

double YmgSPlanner::getTragetCurvature()
{/*{{{*/
	Eigen::Vector2d goal, goal_r;
	utilfcn_->getLocalGoal(1.5, goal);
	utilfcn_->tfGlobal2Robot(goal, goal_r);

	double r_center_y = (goal_r[0]*goal_r[0] - goal_r[1]*goal_r[1]) / (2*goal_r[1]);
	double theta = atan2(r_center_y - goal_r[1], fabs(goal_r[0]));

	ROS_INFO("goal_r : %f, %f", goal_r[0], goal_r[1]);
	ROS_INFO("r_center_y : %f", r_center_y);

	return 1.0/r_center_y;
}/*}}}*/

bool YmgSPlanner::generateTrajectory(
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
		double sim_time_distance = sample_target_vel[0] * sim_time_; // the distance the robot would travel in sim_time
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

	// simulate the trajectory and check for collisions, updating costs along the way
	for (int i = 0; i < num_steps; ++i) {
		traj.addPoint(pos[0], pos[1], pos[2]);
		pos = computeNewPositions(pos, sample_target_vel, dt);
	} // end for simulation steps

	return num_steps > 0; // true if trajectory has at least one point
}/*}}}*/

Eigen::Vector3f YmgSPlanner::computeNewPositions(
		const Eigen::Vector3f& pos, const Eigen::Vector3f& vel, double dt)
{/*{{{*/
	Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
	new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
	new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
	new_pos[2] = pos[2] + vel[2] * dt;
	return new_pos;
}/*}}}*/

}// namespace

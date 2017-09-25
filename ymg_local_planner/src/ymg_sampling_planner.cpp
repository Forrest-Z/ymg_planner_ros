#include <ymg_local_planner/ymg_sampling_planner.h>

#include <ros/console.h>

namespace ymglp {

YmgSamplingPlanner::YmgSamplingPlanner(
		base_local_planner::TrajectoryCostFunction* pdist_critics,
		base_local_planner::TrajectoryCostFunction* obstacle_critics)
{/*{{{*/
	pdist_critics_ = pdist_critics;
	obstacle_critics_ = obstacle_critics;
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

}/*}}}*/

void YmgSamplingPlanner::setParameters(
		double sim_time, double sim_granularity, double angular_sim_granularity, double sim_period)
{/*{{{*/
	sim_time_ = sim_time;
	sim_granularity_ = sim_granularity;
	angular_sim_granularity_ = angular_sim_granularity;
	sim_period_ = sim_period;
}/*}}}*/

bool YmgSamplingPlanner::findBestTrajectory(
		base_local_planner::Trajectory& traj,
		std::vector<base_local_planner::Trajectory>* all_explored)
{/*{{{*/
	ROS_INFO("[ymg_sampling_planner] find_best_trajectory_fcn_running.");

	double v_step = (max_vel_[0] - min_vel_[0]) / vsamples_[0];
	double w_step = (max_vel_[2] - min_vel_[2]) / vsamples_[2];
	base_local_planner::Trajectory best_traj;

	Eigen::Vector3f target_vel;
	target_vel[1] = 0.0;   // vy must be zero
	for (int iv=0; iv<=vsamples_[0]; ++iv) {
		target_vel[0] = max_vel_[0] - iv * v_step;
		double min_dist = DBL_MAX;
		for (int iw=0; iw<=vsamples_[2]; ++iw) {
			target_vel[2] = max_vel_[2] - iw * w_step;

			base_local_planner::Trajectory comp_traj;
			bool result = generateTrajectory(pos_, vel_, target_vel, comp_traj);
			ROS_INFO("generate trajectory result : %d", result);
			double dist = pdist_critics_->scoreTrajectory(comp_traj);
			ROS_INFO("[ymg_sampling_planner] dist = %f", dist);
			if (dist < min_dist) {
				best_traj = comp_traj;
				min_dist =dist;
			}
		}
		double obstacle_cost = obstacle_critics_->scoreTrajectory(best_traj);
		ROS_INFO("[ymg_sampling_planner] obstacle = %f", obstacle_cost);
		if (min_dist < 0.1 && 0<=obstacle_cost && obstacle_cost<=128) {
			traj = best_traj;
			return true;
		}
	}

	ROS_INFO("[ymg_sampling_planner] falied to find valid path.");

	base_local_planner::Trajectory stop_traj;
	Eigen::Vector3f stop_vel;
	stop_vel[0] = 0.0;
	stop_vel[1] = 0.0;
	stop_vel[2] = 0.0;
	generateTrajectory(pos_, vel_, stop_vel, stop_traj);
	traj = stop_traj;

	return false;
}/*}}}*/

bool YmgSamplingPlanner::generateTrajectory(
	Eigen::Vector3f pos,
	Eigen::Vector3f vel,
	Eigen::Vector3f sample_target_vel,
	base_local_planner::Trajectory& traj)
{/*{{{*/
	double vmag = hypot(sample_target_vel[0], sample_target_vel[1]);
	double eps = 1e-4;
	traj.cost_   = -1.0; // placed here in case we return early
	//trajectory might be reused so we'll make sure to reset it
	traj.resetPoints();

	// make sure that the robot would at least be moving with one of
	// the required minimum velocities for translation and rotation (if set)
	if ((limits_->min_trans_vel >= 0 && vmag + eps < limits_->min_trans_vel) &&
			(limits_->min_rot_vel >= 0 && fabs(sample_target_vel[2]) + eps < limits_->min_rot_vel)) {
		return false;
	}
	// make sure we do not exceed max diagonal (x+y) translational velocity (if set)
	if (limits_->max_trans_vel >=0 && vmag - eps > limits_->max_trans_vel) {
		return false;
	}

	int num_steps;
	if (angular_sim_granularity_ < 0.0) {
		num_steps = ceil(sim_time_ / sim_granularity_);
	} else {
		//compute the number of steps we must take along this trajectory to be "safe"
		double sim_time_distance = vmag * sim_time_; // the distance the robot would travel in sim_time if it did not change velocity
		double sim_time_angle = fabs(sample_target_vel[2]) * sim_time_; // the angle the robot would rotate in sim_time
		num_steps =
				ceil(std::max(sim_time_distance / sim_granularity_, sim_time_angle / angular_sim_granularity_));
	}

	//compute a timestep
	double dt = sim_time_ / num_steps;
	traj.time_delta_ = dt;

	Eigen::Vector3f loop_vel;
	loop_vel = sample_target_vel;
	traj.xv_     = sample_target_vel[0];
	traj.yv_     = sample_target_vel[1];
	traj.thetav_ = sample_target_vel[2];

	//simulate the trajectory and check for collisions, updating costs along the way
	for (int i = 0; i < num_steps; ++i) {

		//add the point to the trajectory so we can draw it later if we want
		traj.addPoint(pos[0], pos[1], pos[2]);

		//update the position of the robot using the velocities passed in
		pos = computeNewPositions(pos, loop_vel, dt);

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

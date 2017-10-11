#include <ymg_local_planner/direction_adjust_planner.h>
#include <ros/console.h>

namespace ymglp {

DirAdjustPlanner::DirAdjustPlanner(
		base_local_planner::ObstacleCostFunctionKai* obstacle_critic,
		UtilFcn* utilfcn)
	: handle_latch_(false) 
{/*{{{*/
	rotate_direction_ = UNDEFINED;
	obstacle_critic_ = obstacle_critic;
	utilfcn_ = utilfcn;
}/*}}}*/

void DirAdjustPlanner::initialize(
		base_local_planner::LocalPlannerLimits* limits,
		const Eigen::Vector3f& pos,
		const Eigen::Vector3f& vel,
		const Eigen::Vector3f& vsamples)
{/*{{{*/
	pos_ = pos;
	vel_ = vel;
	limits_ = limits;
	vsamples_ = vsamples;

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

bool DirAdjustPlanner::haveToHandle()
{/*{{{*/
	double distance = utilfcn_->getDistance();
	double direction_error = utilfcn_->getDirectionError();
	// ROS_INFO("[DirAdjPlanner] dir_error = %f", direction_error);
	// ROS_INFO("[DirAdjPlanner] yaw_goal_tolerance = %f", yaw_goal_tolerance_);
	if (distance_tolerance_ < distance) {
		handle_latch_ = false;
	}
	else{
		if (direction_tolerance_ < fabs(direction_error)) {
			if (handle_latch_ == false)
				ROS_INFO("[DirAdjustPlanner] Adjusting direction.");
			handle_latch_ = true;
		}
		else if (fabs(direction_error) < yaw_goal_tolerance_) {
			handle_latch_ = false;
		}
	}

	if (!handle_latch_)
		rotate_direction_ = UNDEFINED;

	return handle_latch_;
}/*}}}*/

void DirAdjustPlanner::resetRotateDirection()
{/*{{{*/
	rotate_direction_ = UNDEFINED;
}/*}}}*/

bool DirAdjustPlanner::findBestTrajectory(
		base_local_planner::Trajectory& traj,
		std::vector<base_local_planner::Trajectory>* all_explored)
{/*{{{*/

	if (rotate_direction_ == UNDEFINED) {
		double direction_error = utilfcn_->getDirectionError();
		if (0.0 < direction_error) {
			ROS_INFO("[DirAdjPlanner] direction:CW (error : %f)", direction_error);
			rotate_direction_ = CW;
		}
		else {
			ROS_INFO("[DirAdjPlanner] direction:CCW (error : %f)", direction_error);
			rotate_direction_ = CCW;
		}
	}

	if (obstacle_critic_->prepare() == false) {
		ROS_WARN("Obstacle scoring function failed to prepare");
		return false;
	}

	Eigen::Vector3f target_vel;
	// decelerate x velocity and stop
	if      (0 < min_vel_[0]) target_vel[0] = min_vel_[0];
	else if (max_vel_[0] < 0) target_vel[0] = max_vel_[0];
	else                     target_vel[0] = 0.0;
	// y velocity must be zero
	target_vel[1] = 0.0;

	double theta_step = (max_vel_[2] - min_vel_[2]) / vsamples_[2];
	base_local_planner::Trajectory comp_traj;

	double path_cost, obstacle_cost;
	double x, y, th;

	traj.cost_ = -1;

	for (int i=0; i<=vsamples_[2]; ++i) {
		target_vel[2] = max_vel_[2] - i * theta_step;

		if (!generateTrajectory(pos_, vel_, target_vel, comp_traj)) {
			ROS_INFO("[DirAdjPlanner] Faild to generate Trajectory.");
			continue;
		}

		obstacle_cost = obstacle_critic_->scoreTrajectory(comp_traj);
		// ROS_INFO("obstacle_cost : %f", obstacle_cost);
		if (obstacle_cost < 0.0 || obstacle_tolerance_+0.5 < obstacle_cost) {
			continue;
		}

		comp_traj.getEndpoint(x, y, th);
		double direction_error;
		if (rotate_direction_ == CCW) {
			direction_error = UtilFcn::getDirectionErrorCCW(th, utilfcn_->getNearestDirection());
		}
		if (rotate_direction_ == CW) {
			direction_error = UtilFcn::getDirectionErrorCW(th, utilfcn_->getNearestDirection());
		}
		// ROS_INFO("direction error : %f", direction_error);

		if (traj.cost_ < 0.0 || fabs(direction_error) < traj.cost_) {
			traj = comp_traj;
			traj.cost_ = fabs(direction_error);
		}
	}

	// ROS_INFO("best_direction_error : %f", traj.cost_);

	if (rotate_direction_ == CCW && traj.thetav_ < 0.0) {
		rotate_direction_ = CW;
	}
	if (rotate_direction_ == CW && 0.0 < traj.thetav_) {
		rotate_direction_ = CCW;
	}

	return true;
}/*}}}*/

bool DirAdjustPlanner::generateTrajectory(
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

Eigen::Vector3f DirAdjustPlanner::computeNewPositions(
		const Eigen::Vector3f& pos, const Eigen::Vector3f& vel, double dt)
{/*{{{*/
	Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
	new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
	new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
	new_pos[2] = pos[2] + vel[2] * dt;
	return new_pos;
}/*}}}*/

}// namespace ymglp

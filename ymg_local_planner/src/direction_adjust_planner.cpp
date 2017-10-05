#include <ymg_local_planner/direction_adjust_planner.h>
#include <ros/console.h>

namespace ymglp {

DirAdjustPlanner::DirAdjustPlanner(base_local_planner::TrajectoryCostFunction* obstacle_critic)
	: handle_latch_(false), position_tolerance_(-1.0), direction_tolerance_(-1.0) 
{/*{{{*/
	rotate_direction_ = UNDEFINED;
	obstacle_critic_ = obstacle_critic;
}/*}}}*/

void DirAdjustPlanner::initialize(
		base_local_planner::LocalPlannerLimits* limits,
		const Eigen::Vector3f& pos,
		const Eigen::Vector3f& vel,
		const Eigen::Vector3f& vsamples,
		double direction_error)
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

	if (rotate_direction_ == UNDEFINED) {
		if (0.0 < direction_error)
			rotate_direction_ == CW;
		else
			rotate_direction_ == CCW;
	}
}/*}}}*/

// XXX has not implemented yet
bool DirAdjustPlanner::haveToHandle(const tf::Stamped<tf::Pose>& robot_pose,
		const std::vector<geometry_msgs::PoseStamped>& global_plan)
{/*{{{*/
	return false;
}/*}}}*/

bool DirAdjustPlanner::haveToHandle(double position_error, double direction_error)
{/*{{{*/
	if (position_tolerance_ < fabs(position_error)) {
		handle_latch_ = false;
	}
	else{
		if (direction_tolerance_ < fabs(direction_error))
			if (handle_latch_ == false)
				ROS_INFO_NAMED("DirAdjustPlanner", "Adjusting direction.");
		handle_latch_ = true;
		if (fabs(direction_error) < yaw_goal_tolerance_) 
			handle_latch_ = false;
	}

	if (handle_latch_)
		rotate_direction_ = UNDEFINED;

	return handle_latch_;
}/*}}}*/

bool DirAdjustPlanner::findBestTrajectory(
		base_local_planner::Trajectory& traj,
		std::vector<base_local_planner::Trajectory>* all_explored)
{/*{{{*/
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
	double start_vel_theta = max_vel_[2];
	if (rotate_direction_ == CW) {
		theta_step *= -1;
		start_vel_theta = min_vel_[2];
	}

	for (int i=0; i<=vsamples_[2]; ++i) {
		target_vel_[2] = start_vel_theta - i * theta_step;
		if (generateTrajectory(pos_, vel_, target_vel_, traj)) {
			traj.cost_ = obstacle_critic_->scoreTrajectory(traj);
			if (0.0 < traj.cost_ && traj.cost_ < obstacle_tolerance_) {
				if (0.0 < target_vel_[2])
					rotate_direction_ = CCW;
				else
					rotate_direction_ = CW;
				return true;
			}
		}
	}

	traj.cost_ = -1;
	return false;
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

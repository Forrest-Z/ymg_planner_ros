#include <ymg_local_planner/simple_trajectory_generator_kai.h>

#include <cmath>

#include <base_local_planner/velocity_iterator.h>

namespace base_local_planner {

void SimpleTrajectoryGeneratorKai::initialise(
    const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel,
    const Eigen::Vector3f& goal,
    base_local_planner::LocalPlannerLimits* limits,
    const Eigen::Vector3f& vsamples,
    std::vector<Eigen::Vector3f> additional_samples)
{/*{{{*/
  initialise(pos, vel, goal, limits, vsamples);
  // add static samples if any
  sample_params_.insert(sample_params_.end(), additional_samples.begin(), additional_samples.end());
}/*}}}*/


void SimpleTrajectoryGeneratorKai::initialise(
    const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel,
    const Eigen::Vector3f& goal,
    base_local_planner::LocalPlannerLimits* limits,
    const Eigen::Vector3f& vsamples)
{/*{{{*/
  /*
   * We actually generate all velocity sample vectors here, from which to generate trajectories later on
   */
  double max_vel_th = limits->max_rot_vel;
  double min_vel_th = -1.0 * max_vel_th;
  Eigen::Vector3f acc_lim = limits->getAccLimits();
  pos_ = pos;
  vel_ = vel;
  limits_ = limits;
  next_sample_index_ = 0;
  sample_params_.clear();

  double min_vel_x = limits->min_vel_x;
  double max_vel_x = limits->max_vel_x;
  double min_vel_y = limits->min_vel_y;
  double max_vel_y = limits->max_vel_y;

  // if sampling number is zero in any dimension, we don't generate samples generically
  if (vsamples[0] * vsamples[1] * vsamples[2] > 0) {
    //compute the feasible velocity space based on the rate at which we run
    Eigen::Vector3f max_vel = Eigen::Vector3f::Zero();
    Eigen::Vector3f min_vel = Eigen::Vector3f::Zero();

		max_vel[0] = std::min(max_vel_x, vel[0] + acc_lim[0] * sim_period_);
		max_vel[1] = std::min(max_vel_y, vel[1] + acc_lim[1] * sim_period_);
		max_vel[2] = std::min(max_vel_th, vel[2] + acc_lim[2] * sim_period_);

		min_vel[0] = std::max(min_vel_x, vel[0] - acc_lim[0] * sim_period_);
		min_vel[1] = std::max(min_vel_y, vel[1] - acc_lim[1] * sim_period_);
		min_vel[2] = std::max(min_vel_th, vel[2] - acc_lim[2] * sim_period_);

    Eigen::Vector3f vel_samp = Eigen::Vector3f::Zero();
    VelocityIterator x_it(min_vel[0], max_vel[0], vsamples[0]);
    VelocityIterator y_it(min_vel[1], max_vel[1], vsamples[1]);
    VelocityIterator th_it(min_vel[2], max_vel[2], vsamples[2]);
    for(; !x_it.isFinished(); x_it++) {
      vel_samp[0] = x_it.getVelocity();
      for(; !y_it.isFinished(); y_it++) {
        vel_samp[1] = y_it.getVelocity();
        for(; !th_it.isFinished(); th_it++) {
          vel_samp[2] = th_it.getVelocity();
          //ROS_DEBUG("Sample %f, %f, %f", vel_samp[0], vel_samp[1], vel_samp[2]);
          sample_params_.push_back(vel_samp);
        }
        th_it.reset();
      }
      y_it.reset();
    }
  }
}/*}}}*/

void SimpleTrajectoryGeneratorKai::setParameters(
    double sim_time,
    double sim_granularity,
    double angular_sim_granularity,
    double sim_period)
{/*{{{*/
  sim_time_ = sim_time;
  sim_granularity_ = sim_granularity;
  angular_sim_granularity_ = angular_sim_granularity;
  sim_period_ = sim_period;
}/*}}}*/

/**
 * Whether this generator can create more trajectories
 */
bool SimpleTrajectoryGeneratorKai::hasMoreTrajectories()
{/*{{{*/
  return next_sample_index_ < sample_params_.size();
}/*}}}*/

/**
 * Create and return the next sample trajectory
 */
bool SimpleTrajectoryGeneratorKai::nextTrajectory(Trajectory &comp_traj)
{/*{{{*/
  bool result = false;
  if (hasMoreTrajectories()) {
    if (generateTrajectory(
        pos_,
        vel_,
        sample_params_[next_sample_index_],
        comp_traj)) {
      result = true;
    }
  }
  next_sample_index_++;
  return result;
}/*}}}*/

/**
 * @param pos current position of robot
 * @param vel desired velocity for sampling
 */
bool SimpleTrajectoryGeneratorKai::generateTrajectory(
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
        ceil(std::max(sim_time_distance / sim_granularity_,
            sim_time_angle    / angular_sim_granularity_));
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

Eigen::Vector3f SimpleTrajectoryGeneratorKai::computeNewPositions(const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel, double dt)
{/*{{{*/
  Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
  new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
  new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
  new_pos[2] = pos[2] + vel[2] * dt;
  return new_pos;
}/*}}}*/

/**
 * cheange vel using acceleration limits to converge towards sample_target-vel
 */
Eigen::Vector3f SimpleTrajectoryGeneratorKai::computeNewVelocities(const Eigen::Vector3f& sample_target_vel,
    const Eigen::Vector3f& vel, Eigen::Vector3f acclimits, double dt)
{/*{{{*/
  Eigen::Vector3f new_vel = Eigen::Vector3f::Zero();
  for (int i = 0; i < 3; ++i) {
    if (vel[i] < sample_target_vel[i]) {
      new_vel[i] = std::min(double(sample_target_vel[i]), vel[i] + acclimits[i] * dt);
    } else {
      new_vel[i] = std::max(double(sample_target_vel[i]), vel[i] - acclimits[i] * dt);
    }
  }
  return new_vel;
}/*}}}*/

} /* namespace base_local_planner */

#ifndef SIMPLE_TRAJECTORY_GENERATOR_KAI_H_
#define SIMPLE_TRAJECTORY_GENERATOR_KAI_H_

#include <base_local_planner/trajectory_sample_generator.h>
#include <base_local_planner/local_planner_limits.h>
#include <Eigen/Core>

namespace base_local_planner {

/**
 * generates trajectories based on equi-distant discretisation of the degrees of freedom.
 * This is supposed to be a simple and robust implementation of the TrajectorySampleGenerator
 * interface, more efficient implementations are thinkable.
 *
 * This can be used for both dwa and trajectory rollout approaches.
 * As an example, assuming these values:
 * sim_time = 1s, sim_period=200ms, dt = 200ms,
 * vsamples_x=5,
 * acc_limit_x = 1m/s^2, vel_x=0 (robot at rest, values just for easy calculations)
 * dwa_planner will sample max-x-velocities from 0m/s to 0.2m/s.
 * trajectory rollout approach will sample max-x-velocities 0m/s up to 1m/s
 * trajectory rollout approach does so respecting the acceleration limit, so it gradually increases velocity
 */
class SimpleTrajectoryGeneratorKai: public base_local_planner::TrajectorySampleGenerator {
public:

  SimpleTrajectoryGeneratorKai() {
    limits_ = NULL;
  }

  ~SimpleTrajectoryGeneratorKai() {}

  /**
   * @param pos current robot position
   * @param vel current robot velocity
   * @param limits Current velocity limits
   * @param vsamples: in how many samples to divide the given dimension
   * @param use_acceleration_limits: if true use physical model, else idealized robot model
   * @param additional_samples (deprecated): Additional velocity samples to generate individual trajectories from.
   * @param discretize_by_time if true, the trajectory is split according in chunks of the same duration, else of same length
   */
  void initialise(
      const Eigen::Vector3f& pos,
      const Eigen::Vector3f& vel,
      const Eigen::Vector3f& goal,
      base_local_planner::LocalPlannerLimits* limits,
      const Eigen::Vector3f& vsamples,
      std::vector<Eigen::Vector3f> additional_samples,
      bool discretize_by_time = false);

  /**
   * @param pos current robot position
   * @param vel current robot velocity
   * @param limits Current velocity limits
   * @param vsamples: in how many samples to divide the given dimension
   * @param use_acceleration_limits: if true use physical model, else idealized robot model
   * @param discretize_by_time if true, the trajectory is split according in chunks of the same duration, else of same length
   */
  void initialise(
      const Eigen::Vector3f& pos,
      const Eigen::Vector3f& vel,
      const Eigen::Vector3f& goal,
      base_local_planner::LocalPlannerLimits* limits,
      const Eigen::Vector3f& vsamples,
      bool discretize_by_time = false);

  /**
   * This function is to be called only when parameters change
   *
   * @param sim_granularity granularity of collision detection
   * @param angular_sim_granularity angular granularity of collision detection
   * @param use_dwa whether to use DWA or trajectory rollout
   * @param sim_period distance between points in one trajectory
   */
  void setParameters(double sim_time,
      double sim_granularity,
      double angular_sim_granularity,
      double sim_period = 0.0);

  /**
   * Whether this generator can create more trajectories
   */
  bool hasMoreTrajectories();

  /**
   * Whether this generator can create more trajectories
   */
  bool nextTrajectory(Trajectory &traj);


  static Eigen::Vector3f computeNewPositions(const Eigen::Vector3f& pos,
      const Eigen::Vector3f& vel, double dt);

  static Eigen::Vector3f computeNewVelocities(const Eigen::Vector3f& sample_target_vel,
      const Eigen::Vector3f& vel, Eigen::Vector3f acclimits, double dt);

  bool generateTrajectory(
        Eigen::Vector3f pos,
        Eigen::Vector3f vel,
        Eigen::Vector3f sample_target_vel,
        base_local_planner::Trajectory& traj);

protected:

  unsigned int next_sample_index_;
  // to store sample params of each sample between init and generation
  std::vector<Eigen::Vector3f> sample_params_;
  base_local_planner::LocalPlannerLimits* limits_;
  Eigen::Vector3f pos_;
  Eigen::Vector3f vel_;

  // whether velocity of trajectory changes over time or not
  bool discretize_by_time_;

  double sim_time_, sim_granularity_, angular_sim_granularity_;
  double sim_period_; // only for dwa
};

} /* namespace base_local_planner */
#endif /* SIMPLE_TRAJECTORY_GENERATOR_H_ */

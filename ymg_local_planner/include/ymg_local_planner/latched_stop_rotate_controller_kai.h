#ifndef LATCHED_STOP_ROTATE_CONTROLLER_KAI_H_
#define LATCHED_STOP_ROTATE_CONTROLLER_KAI_H_

#include <string>

#include <Eigen/Core>

#include <tf/transform_datatypes.h>

#include <ymg_local_planner/local_planner_util_kai.h>
#include <base_local_planner/odometry_helper_ros.h>

namespace base_local_planner {

class LatchedStopRotateControllerKai {
public:
  LatchedStopRotateControllerKai(const std::string& name = "");
  virtual ~LatchedStopRotateControllerKai();

  bool isPositionReached(LocalPlannerUtilKai* planner_util,
      tf::Stamped<tf::Pose> global_pose);

  bool isGoalReached(LocalPlannerUtilKai* planner_util,
      OdometryHelperRos& odom_helper,
      tf::Stamped<tf::Pose> global_pose);

  void resetLatching() {
    xy_tolerance_latch_ = false;
  }

  /**
   * @brief Stop the robot taking into account acceleration limits
   * @param  global_pose The pose of the robot in the global frame
   * @param  robot_vel The velocity of the robot
   * @param  cmd_vel The velocity commands to be filled
   * @return  True if a valid trajectory was found, false otherwise
   */
  bool stopWithAccLimits(const tf::Stamped<tf::Pose>& global_pose,
      const tf::Stamped<tf::Pose>& robot_vel,
      geometry_msgs::Twist& cmd_vel,
      Eigen::Vector3f acc_lim,
      double sim_period,
      boost::function<bool (Eigen::Vector3f pos,
                            Eigen::Vector3f vel,
                            Eigen::Vector3f vel_samples)> obstacle_check);

  /**
   * @brief Once a goal position is reached... rotate to the goal orientation
   * @param  global_pose The pose of the robot in the global frame
   * @param  robot_vel The velocity of the robot
   * @param  goal_th The desired th value for the goal
   * @param  cmd_vel The velocity commands to be filled
   * @return  True if a valid trajectory was found, false otherwise
   */
  bool rotateToGoal(const tf::Stamped<tf::Pose>& global_pose,
      const tf::Stamped<tf::Pose>& robot_vel,
      double goal_th,
      geometry_msgs::Twist& cmd_vel,
      Eigen::Vector3f acc_lim,
      double sim_period,
      base_local_planner::LocalPlannerLimits& limits,
      boost::function<bool (Eigen::Vector3f pos,
                            Eigen::Vector3f vel,
                            Eigen::Vector3f vel_samples)> obstacle_check);

  bool computeVelocityCommandsStopRotate(geometry_msgs::Twist& cmd_vel,
      Eigen::Vector3f acc_lim,
      double sim_period,
      LocalPlannerUtilKai* planner_util,
      OdometryHelperRos& odom_helper,
      tf::Stamped<tf::Pose> global_pose,
      boost::function<bool (Eigen::Vector3f pos,
                            Eigen::Vector3f vel,
                            Eigen::Vector3f vel_samples)> obstacle_check);

private:
  inline double sign(double x){
    return x < 0.0 ? -1.0 : 1.0;
  }


  // whether to latch at all, and whether in this turn we have already been in goal area
  bool latch_xy_goal_tolerance_, xy_tolerance_latch_;
  bool rotating_to_goal_;
};

} /* namespace base_local_planner */
#endif /* LATCHED_STOP_ROTATE_CONTROLLER_H_ */

#ifndef LOACL_PLANNER_UTIL_KAI_H_
#define LOACL_PLANNER_UTIL_KAI_H_

#include <nav_core/base_local_planner.h>

#include <boost/thread.hpp>

#include <costmap_2d/costmap_2d.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <base_local_planner/local_planner_limits.h>


namespace base_local_planner {

/**
 * @class LocalPlannerUtilKai
 * @brief Helper class implementing infrastructure code many local planner implementations may need.
 */
class LocalPlannerUtilKai {

private:
  // things we get from move_base
  std::string name_;
  std::string global_frame_;

  costmap_2d::Costmap2D* costmap_;
  tf::TransformListener* tf_;


  std::vector<geometry_msgs::PoseStamped> global_plan_;


  boost::mutex limits_configuration_mutex_;
  bool setup_;
  LocalPlannerLimits default_limits_;
  LocalPlannerLimits limits_;
  bool initialized_;

public:

  /**
   * @brief  Callback to update the local planner's parameters
   */
  void reconfigureCB(LocalPlannerLimits &config, bool restore_defaults);

  LocalPlannerUtilKai() : initialized_(false) {}

  ~LocalPlannerUtilKai() {
  }

  void initialize(tf::TransformListener* tf,
      costmap_2d::Costmap2D* costmap,
      std::string global_frame);

  bool getGoal(tf::Stamped<tf::Pose>& goal_pose);

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  bool getLocalPlan(tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& transformed_plan);

  costmap_2d::Costmap2D* getCostmap();

  LocalPlannerLimits getCurrentLimits();

  std::string getGlobalFrame(){ return global_frame_; }
};




};

#endif /* ABSTRACT_LOCAL_PLANNER_ODOM_H_ */

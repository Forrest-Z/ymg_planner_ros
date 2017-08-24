/**
 * @file YmgLPROS.h
 * @brief YMG's local planner header for ros
 * @author YMG
 * @date 2017.07
 */


#ifndef YMGLPROS_H_
#define YMGLPROS_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/map_cell.h>
#include <base_local_planner/map_grid.h>
#include <base_local_planner/footprint_helper.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <dwa_local_planner/DWAPlannerConfig.h>

#include <angles/angles.h>

#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/latched_stop_rotate_controller.h>

#include <base_local_planner/odometry_helper_ros.h>

namespace ymglp
{
	class YmgLPROS: public nav_core::BaseLocalPlanner
	{
		public:
			YmgLPROS();
			void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
      bool isGoalReached();

		private:
      tf::TransformListener* tf_; ///< @brief Used for transforming point clouds

      // for visualisation, publishers of global and local plan
      ros::Publisher local_plan_pub_;

      base_local_planner::LocalPlannerUtil planner_util_;

      boost::shared_ptr<DWAPlanner> dp_; ///< @brief The trajectory controller

      costmap_2d::Costmap2DROS* costmap_ros_;

      dynamic_reconfigure::Server<DWAPlannerConfig> *dsrv_;
      dwa_local_planner::DWAPlannerConfig default_config_;
      bool setup_;
      tf::Stamped<tf::Pose> current_pose_;

      base_local_planner::LatchedStopRotateController latchedStopRotateController_;


      bool initialized_;


      base_local_planner::OdometryHelperRos odom_helper_;
      std::string odom_topic_;

	};   // class YmgLPROS
};   // namespace ymglp

#endif


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
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>


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
			bool initialized_, reached_goal_, xy_tolerance_latch_;
			double acc_lim_x_, acc_lim_theta_, max_vel_x_, max_vel_theta_;
			double sim_time_;
			int vtheta_samples_;
			geometry_msgs::Twist target_vel_;
      costmap_2d::Costmap2DROS* costmap_ros_; ///< @brief The ROS wrapper for the costmap the controller will use
      costmap_2d::Costmap2D* costmap_; ///< @brief The costmap the controller will use
      base_local_planner::OdometryHelperRos odom_helper_;
      std::vector<geometry_msgs::Point> footprint_spec_; ///< @brief The footprint specification of the robot
      base_local_planner::FootprintHelper footprint_helper_;
			base_local_planner::MapGrid path_map_; ///< @brief The local map grid where we propagate path distance
			base_local_planner::MapGrid goal_map_; ///< @brief The local map grid where we propagate goal distance
      ros::Publisher plan_pub_;
			std::string global_frame_, robot_base_frame_;
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      tf::TransformListener* tf_; ///< @brief Used for transforming point clouds
			double yaw_goal_tolerance_, xy_goal_tolerance_;

	};   // class YmgLPROS
};   // namespace ymglp

#endif


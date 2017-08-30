/**
 * @file ymggp_ros.h
 * @brief YMG's global planner plugin for ros
 * @author YMG
 * @date 2017.07
 */

#ifndef YMGGP_ROS_H_
#define YMGGP_ROS_H_

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <ymg_global_planner/ymggp.h>


namespace ymggp
{
	class YmgGPROS: public nav_core::BaseGlobalPlanner
	{
		public:
			YmgGPROS();
			void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
			bool makePlan(const geometry_msgs::PoseStamped& start, 
					const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
      void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
    
		
		private:
			bool initialized_;
      ros::Publisher plan_pub_;
			YmgGP ymg_global_planner_;

	};   // class YmgGPROS
};   // namespace ymggp


#endif

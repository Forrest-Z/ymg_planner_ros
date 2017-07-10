/**
 * @file YmgGPRos.h
 * @brief ymj global planner library for ros
 * @author YMG
 * @date 2017.05
 */

#ifndef YMGGPROS_H_
#define YMGGPROS_H_

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d.h>


namespace ymggp
{
	class YmgGPROS : public nav_core::BaseGlobalPlanner {

		public:
			YmgGPROS();
			void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
			bool makePlan(const geometry_msgs::PoseStamped& start, 
					const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
      void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
    
		
		protected:
			bool initialized_;
      ros::Publisher plan_pub_;


		private:
			std::string global_frame_, tf_prefix_;
			std::vector<geometry_msgs::PoseStamped> plan_;
			double path_resolution_;   //**< path resolution in point/m */
			double path_length_;       //**< path length in meters */
      inline double sq_distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

	};   // class YmgGPROS
};   // namespace ymggp


#endif

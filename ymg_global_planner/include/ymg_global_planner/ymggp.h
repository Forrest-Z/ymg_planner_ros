/**
 * @file ymggp.h
 * @brief YMG's global planner library
 * @author YMG
 * @date 2017.07
 */

#ifndef YMGGP_H_
#define YMGGP_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

namespace ymggp
{
	class YmgGP
	{
		public:
			YmgGP() : initialized_(false){};
			void initialize(std::string global_frame, double path_resolution);
			bool makePlan(const geometry_msgs::PoseStamped& start, 
					const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

		private:
			bool initialized_;
			std::vector<geometry_msgs::PoseStamped> plan_;
			std::string global_frame_;
			double path_resolution_;   //**< path resolution in point/m */
      inline double sqDistance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);
	};
};   // namespace ymggp

#endif

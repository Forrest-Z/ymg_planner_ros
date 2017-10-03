/**
 * @file ymggp_ros.cpp
 * @brief YMG's global planner plugin for ros
 * @author YMG
 * @date 2017.07
 */


#include <ymg_global_planner/ymggp_ros.h>
#include <pluginlib/class_list_macros.h>

using namespace ymggp;

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(ymggp::YmgGPROS, nav_core::BaseGlobalPlanner)


/**
 * @brief constructer
 */
YmgGPROS::YmgGPROS()
	: initialized_(false)
{}


/**
 * @brief  Initialization function for the NavFnROS object
 * @param  name The name of this planner
 * @param  costmap A pointer to the ROS wrapper of the costmap to use for planning
 */
void YmgGPROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{/*{{{*/
	if (!initialized_) {
		ros::NodeHandle private_nh("~/" + name);
		double path_granularity, max_path_length;
		private_nh.param("path_granularity", path_granularity, 10.0);
		plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

		ymg_global_planner_.initialize(costmap_ros->getGlobalFrameID(), path_granularity);

		initialized_ = true;
	}
	else
		ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
}/*}}}*/

/**
 * @brief Given a goal pose in the world, compute a plan
 * @param start The start pose 
 * @param goal The goal pose 
 * @param plan The plan... filled by the planner
 * @return True if a valid plan was found, false otherwise
 */
bool YmgGPROS::makePlan(const geometry_msgs::PoseStamped& start, 
		const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{/*{{{*/
	if (!initialized_) {
		ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
		return false;
	}

	plan.clear();
	ymg_global_planner_.makePlan(start, goal, plan);
	publishPlan(plan);

	return !plan.empty();
}/*}}}*/

/**
 * @brief convert vector<geometry_msgs::PoseStamped> into nav_msgs::Path and publish caliclated plan
 * @param path calclated path
 */
void YmgGPROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{/*{{{*/
	if (!initialized_) {
		ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
		return;
	}

	nav_msgs::Path gui_path;
	gui_path.poses.resize(path.size());

	if(!path.empty())
	{
		gui_path.header.frame_id = path[0].header.frame_id;
		gui_path.header.stamp = path[0].header.stamp;
	}

	// Extract the plan in world co-ordinates, we assume the path is all in the same frame
	for(unsigned int i=0; i<path.size(); ++i){
		gui_path.poses[i] = path[i];
	}

	plan_pub_.publish(gui_path);
}/*}}}*/




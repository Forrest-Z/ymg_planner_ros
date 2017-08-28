/**
 * @file ymggp_ros.cpp
 * @brief YMG's global planner library for ros
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
		private_nh.param("path_resolution", path_resolution_, 10.0);
		private_nh.param("max_path_length", max_path_length_, 20.0);
		plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

		// ROS_INFO("[YmgGPROS] global_frame = %s", global_frame_.c_str());
		// ROS_INFO("[YmgGPROS] path_resolution = %f", path_resolution_);
		// ROS_INFO("[YmgGPROS] max_path_length = %f", max_path_length_);

		max_path_size_ = max_path_length_ * path_resolution_;
		global_frame_ = costmap_ros->getGlobalFrameID();
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

	if (goal.header.frame_id != global_frame_) {
		ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
				global_frame_.c_str(), goal.header.frame_id.c_str());
		return false;
	}

	if (start.header.frame_id != global_frame_) {
		ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
				global_frame_.c_str(), start.header.frame_id.c_str());
		return false;
	}


	plan.clear();

	ros::Time plan_time = ros::Time::now();
	if (plan_.empty()) {
		geometry_msgs::PoseStamped pose;
		pose.header.stamp = plan_time;
		pose.header.frame_id = global_frame_;
		pose.pose.position = start.pose.position;
		pose.pose.orientation = start.pose.orientation;
		plan_.push_back(pose);
	}

	geometry_msgs::PoseStamped endpoint = plan_.back();

	int points = sq_distance(endpoint, goal) * path_resolution_;
	// ROS_INFO("global planner makePlan() function called and add %d points trajectory", points);

	if (1 <= points) {
		double step_x = (goal.pose.position.x - endpoint.pose.position.x) / points;
		double step_y = (goal.pose.position.y - endpoint.pose.position.y) / points;

		for (int i=1; i<=points; ++i)
		{
			geometry_msgs::PoseStamped pose;
			pose.header.stamp = plan_time;
			pose.header.frame_id = global_frame_;
			pose.pose.position.x = endpoint.pose.position.x + step_x * i;
			pose.pose.position.y = endpoint.pose.position.y + step_y * i;
			pose.pose.position.z = 0.0;
			pose.pose.orientation.x = 0.0;
			pose.pose.orientation.y = 0.0;
			pose.pose.orientation.z = 0.0;
			pose.pose.orientation.w = 1.0;
			plan_.push_back(pose);
		}

		if (max_path_size_ < plan_.size()) {
			plan_.erase(plan_.begin(), plan_.begin() + plan_.size()-max_path_size_); 
		}
	}
	
	// if (max_path_points_ < plan_.size()) {
	// 		std::vector<geometry_msgs::PoseStamped> cut_plan;
	// 		for (int i = plan_.size()-max_path_size_; i<plan_.size(); ++i) {
	// 			cut_plan.push_back(plan_[i]);
	// 		}
	// 		plan_ = cut_plan;
	// }

	plan = plan_;
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

/**
 * @brief calc distance between two poses
 * @param p1 pose1
 * @param p2 pose2
 */
inline double YmgGPROS::sq_distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{/*{{{*/
	double dx = p1.pose.position.x - p2.pose.position.x;
	double dy = p1.pose.position.y - p2.pose.position.y;
	return sqrt(dx*dx + dy*dy);
}/*}}}*/




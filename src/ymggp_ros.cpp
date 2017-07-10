/**
 * @file ymggp_ros.cpp
 * @brief ymj global planner library for ros
 * @author YMG
 * @date 2017.05
 */


#include <ymggp_ros/YmgGPROS.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>

using namespace ymggp;
using namespace std;

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_DECLARE_CLASS(ymggp, YmgGPROS, ymggp::YmgGPROS, nav_core::BaseGlobalPlanner)

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
	cout<<"initialized"<<endl;

	if (!initialized_) {
		ros::NodeHandle private_nh("~/" + name);
		private_nh.param("path_resolution", path_resolution_, 20.0);
		private_nh.param("global_frame", global_frame_, string("map"));
		plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

		// ros::NodeHandle prefix_nh;
		// tf_prefix_ = tf::getPrefixParam(prefix_nh);

		initialized_ = true;
	}
	else
		ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
}/*}}}*/
// class YmgGPROS

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

	// if(tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame_)){
	// 	ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
	// 			tf::resolve(tf_prefix_, global_frame_).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
	// 	return false;
	// }
	//
	// if(tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame_)){
	// 	ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
	// 			tf::resolve(tf_prefix_, global_frame_).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
	// 	return false;
	// }

	double dist = sq_distance(start, goal);
	int points = dist * path_resolution_;

	double step_x = (goal.pose.position.x - start.pose.position.x) / points;
	double step_y = (goal.pose.position.y - start.pose.position.y) / points;
	ros::Time plan_time = ros::Time::now();
	for (int i=0; i<points; ++i)
	{
		geometry_msgs::PoseStamped pose;
		pose.header.stamp = plan_time;
		pose.header.frame_id = global_frame_;
		pose.pose.position.x = start.pose.position.x + step_x * i;
		pose.pose.position.y = start.pose.position.y + step_y * i;
		pose.pose.position.z = 0.0;
		pose.pose.orientation.x = 0.0;
		pose.pose.orientation.y = 0.0;
		pose.pose.orientation.z = 0.0;
		pose.pose.orientation.w = 1.0;
		plan.push_back(pose);
	}

	publishPlan(plan);
	return !plan.empty();
}/*}}}*/

/**
 * @brief convert vector<geometry_msgs::PoseStamped> into nav_msgs::Path and publish caliclated plan
 * @param path calclated path
 */
void YmgGPROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{/*{{{*/
	nav_msgs::Path gui_path;
	gui_path.poses.resize(path.size());

	if(!path.empty())
	{
		gui_path.header.frame_id = path[0].header.frame_id;
		gui_path.header.stamp = path[0].header.stamp;
	}

	// Extract the plan in world co-ordinates, we assume the path is all in the same frame
	for(unsigned int i=0; i < path.size(); i++){
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
	return dx*dx +dy*dy;
}/*}}}*/




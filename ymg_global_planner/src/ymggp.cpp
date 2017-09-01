/**
 * @file ymggp.cpp
 * @brief YMG's global planner library
 * @author YMG
 * @date 2017.07
 */

#include <ymg_global_planner/ymggp.h>
#include <ymg_global_planner/util_function.h>

using namespace ymggp;

/**
 * @brief  Initialization function for the YmgGP object
 * @param  global_frame The Global frame of this planner
 * @param  path_resolution path resolution
 */
void YmgGP::initialize (std::string global_frame, double path_resolution)
{/*{{{*/
	global_frame_ = global_frame;
	path_resolution_ = path_resolution;
	initialized_ = true;
}/*}}}*/

/**
 * @brief clear the plan 
 */
void YmgGP::clearPlan ()
{/*{{{*/
	if (!initialized_) {
		std::cout<<"YmgGP has not initialized yet."<<std::endl;
		return;
	}

	plan_.clear();
}/*}}}*/

/**
 * @brief Given a goal pose in the world, compute a plan
 * @param start The start pose 
 * @param goal The goal pose 
 * @param plan The plan... filled by the planner
 * @return True if a valid plan was found, false otherwise
 */
bool YmgGP::makePlan(const geometry_msgs::PoseStamped& start, 
		const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{/*{{{*/
	if (!initialized_) {
		std::cout<<"YmgGP has not initialized yet."<<std::endl;
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

	ros::Time time_now = ros::Time::now();
	if (plan_.empty()) {
		geometry_msgs::PoseStamped pose;
		pose.header.stamp = time_now;
		pose.header.frame_id = global_frame_;
		pose.pose.position = start.pose.position;
		pose.pose.orientation = start.pose.orientation;
		plan_.push_back(pose);
	}
	
	// get nearest plan index and shorten trajectory
	double min_dist = 1000.0;
	int closest_index = getClosestIndexOfPath(start, plan_);
	if (closest_index < 0) closest_index = 0;

	std::vector<geometry_msgs::PoseStamped> new_plan;
	for (int i=closest_index; i<plan_.size(); ++i) {
		new_plan.push_back(plan_[i]);
	}


	geometry_msgs::PoseStamped endpoint = plan_.back();
	int points = calcDist(endpoint, goal) * path_resolution_;
	// ROS_INFO("global planner makePlan() function called and add %d points trajectory", points);
	if (1 <= points) {
		double step_x = (goal.pose.position.x - endpoint.pose.position.x) / points;
		double step_y = (goal.pose.position.y - endpoint.pose.position.y) / points;

		for (int i=1; i<=points; ++i)
		{
			geometry_msgs::PoseStamped pose;
			pose.header.stamp = time_now;
			pose.header.frame_id = global_frame_;
			pose.pose.position.x = endpoint.pose.position.x + step_x * i;
			pose.pose.position.y = endpoint.pose.position.y + step_y * i;
			pose.pose.position.z = 0.0;
			pose.pose.orientation.x = 0.0;
			pose.pose.orientation.y = 0.0;
			pose.pose.orientation.z = 0.0;
			pose.pose.orientation.w = 1.0;
			new_plan.push_back(pose);
		}
	}

	plan_ = new_plan;
	plan = new_plan;

	return !plan.empty();
}/*}}}*/


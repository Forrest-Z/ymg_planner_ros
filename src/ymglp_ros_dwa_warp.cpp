/**
 * @file ymglp_ros.cpp
 * @brief YMG's local planner library for ros
 * @author YMG
 * @date 2017.07
 */


#include <ymg_planner_ros/YmgLPROS.h>
#include <pluginlib/class_list_macros.h>

#include <base_local_planner/Position2DInt.h>
#include <base_local_planner/footprint_helper.h>
#include <base_local_planner/goal_functions.h>

using namespace ymglp;

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(ymglp::YmgLPROS, nav_core::BaseLocalPlanner)


	/**
	 * @brief constructer
	 */
YmgLPROS::YmgLPROS()
	: initialized_(false), reached_goal_(false), xy_goal_tolerance_(false), odom_helper_("odom")
{/*{{{*/
	geometry_msgs::Vector3 zero;
	zero.x = 0.0;
	zero.y = 0.0;
	zero.z = 0.0;
	target_vel_.linear = zero;
	target_vel_.angular = zero;
}/*}}}*/


/**
 * @brief  Initialization function for the NavFnROS object
 * @param  name The name of this planner
 * @param  costmap A pointer to the ROS wrapper of the costmap to use for planning
 */
void YmgLPROS::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
{/*{{{*/
	if (!initialized_) {

		ros::NodeHandle private_nh("~/" + name);
		local_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
		tf_ = tf;
		costmap_ros_ = costmap_ros;
		costmap_ros_->getRobotPose(current_pose_);

		// make sure to update the costmap we'll use for this cycle
		costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

		planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

		//create the actual planner that we'll use.. it'll configure itself from the parameter server
		dp_ = boost::shared_ptr<DWAPlanner>(new DWAPlanner(name, &planner_util_));

		if( private_nh.getParam( "odom_topic", odom_topic_ ))
		{
			odom_helper_.setOdomTopic( odom_topic_ );
		}

		initialized_ = true;
	}

	else{
		ROS_WARN("This planner has already been initialized, doing nothing.");
	}
}/*}}}*/



/**
 * @brief  Set the plan that the controller is following
 * @param orig_global_plan The plan to pass to the controller
 * @return True if the plan was updated successfully, false otherwise
 */
bool YmgLPROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{/*{{{*/
	if (!initialized_) {
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
	}

	global_plan_.clear();
	global_plan_ = orig_global_plan;

	return true;
}/*}}}*/


/**
 * @brief  Given the current position, orientation, and velocity of the robot,
 * compute velocity commands to send to the base
 * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
 * @return True if a valid trajectory was found, false otherwise
 */
bool YmgLPROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{/*{{{*/
	if (!initialized_) {
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
	}

	std::vector<geometry_msgs::PoseStamped> local_plan;
	tf::Stamped<tf::Pose> global_pose;
	if (!costmap_ros_->getRobotPose(global_pose)) {
		return false;
	}

	std::vector<geometry_msgs::PoseStamped> transformed_plan;
	// get the global plan in our frame
	if (!base_local_planner::transformGlobalPlan(*tf_, global_plan_, global_pose, *costmap_, global_frame_, transformed_plan)) {
		ROS_WARN("Could not transform the global plan to the frame of the controller");
		return false;
	}

	// if the global plan passed in is empty... we won't do anything
	if(transformed_plan.empty())
		return false;

	tf::Stamped<tf::Pose> drive_cmds;
	drive_cmds.frame_id_ = robot_base_frame_;

	tf::Stamped<tf::Pose> robot_vel;
	odom_helper_.getRobotVel(robot_vel);

	Eigen::Vector3f pos(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), tf::getYaw(global_pose.getRotation()));
	Eigen::Vector3f vel(robot_vel.getOrigin().getX(), robot_vel.getOrigin().getY(), tf::getYaw(robot_vel.getRotation()));

	//temporarily remove obstacles that are within the footprint of the robot
	std::vector<base_local_planner::Position2DInt> footprint_list =
		footprint_helper_.getFootprintCells(
				pos,
				footprint_spec_,
				*costmap_,
				true);

	//mark cells within the initial footprint of the robot
	for (unsigned int i = 0; i < footprint_list.size(); ++i) {
		path_map_(footprint_list[i].x, footprint_list[i].y).within_robot = true;
	}

	//make sure that we update our path based on the global plan and compute costs
	path_map_.setTargetCells(*costmap_, global_plan_);
	goal_map_.setLocalGoal(*costmap_, global_plan_);
	ROS_DEBUG("Path/Goal distance computed");

	// we assume the global goal is the last point in the global plan
	tf::Stamped<tf::Pose> goal_point;
	tf::poseStampedMsgToTF(transformed_plan.back(), goal_point);
	double goal_x = goal_point.getOrigin().getX();
	double goal_y = goal_point.getOrigin().getY();
	double goal_th = tf::getYaw(goal_point.getRotation());

	// check to see if we've reached the goal position
	if (xy_tolerance_latch_
			|| base_local_planner::getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance_) {

		// write near_by the goal motion

		return true;
	}


	return true;
}/*}}}*/


/**
 * @brief  Check if the goal pose has been achieved
 * @return True if achieved, false otherwise
 */
bool YmgLPROS::isGoalReached()
{/*{{{*/
	if (!initialized_) {
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
	}

	return reached_goal_;
}/*}}}*/



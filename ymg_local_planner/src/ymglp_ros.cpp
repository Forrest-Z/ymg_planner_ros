#include <ymg_local_planner/ymglp_ros.h>

#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(ymglp::YmgLPROS, nav_core::BaseLocalPlanner);

namespace ymglp {

void YmgLPROS::reconfigureCB(YmgLPConfig &config, uint32_t level)
{/*{{{*/
	if (setup_ && config.restore_defaults) {
		config = default_config_;
		config.restore_defaults = false;
	}
	if ( ! setup_) {
		default_config_ = config;
		setup_ = true;
	}

	// update generic local planner params
	base_local_planner::LocalPlannerLimits limits;
	limits.max_trans_vel = fabs(config.max_vel_x);
	limits.min_trans_vel = 0.0;
	if (0.0 < config.max_vel_x) {
		limits.max_vel_x = config.max_vel_x;
		limits.min_vel_x = 0.0;
	}
	else {
		limits.max_vel_x = 0.0;
		limits.min_vel_x = config.max_vel_x;
	}
	limits.max_vel_y = 0.0;
	limits.min_vel_y = 0.0;
	limits.max_rot_vel = config.max_vel_theta;
	limits.min_rot_vel = 0.0;
	limits.acc_lim_x = config.acc_lim_x;
	limits.acc_lim_y = 0.0;
	limits.acc_lim_theta = config.acc_lim_theta;
	limits.acc_limit_trans = config.acc_lim_x;
	limits.xy_goal_tolerance = config.xy_goal_tolerance;
	limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
	limits.prune_plan = false;
	limits.trans_stopped_vel = config.trans_stopped_vel;
	limits.rot_stopped_vel = config.rot_stopped_vel;
	planner_util_.reconfigureCB(limits, config.restore_defaults);

	// update dwa specific configuration
	ymglp_->reconfigure(config);
}/*}}}*/

YmgLPROS::YmgLPROS() :
	/*{{{*/
	initialized_(false), odom_helper_("odom"), setup_(false)
{}/*}}}*/

YmgLPROS::~YmgLPROS()
{/*{{{*/
	delete dsrv_;
}/*}}}*/

void YmgLPROS::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
{/*{{{*/
	if (! isInitialized()) {

		ros::NodeHandle private_nh("~/" + name);
		g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
		l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
		l_plan_array_pub_ = private_nh.advertise<geometry_msgs::PoseArray>("local_plan_array", 1);
		tf_ = tf;
		costmap_ros_ = costmap_ros;
		costmap_ros_->getRobotPose(current_pose_);

		// make sure to update the costmap we'll use for this cycle
		costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

		planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

		// create the actual planner that we'll use.. it'll configure itself from the parameter server
		ymglp_ = boost::shared_ptr<YmgLP>(new YmgLP(name, &planner_util_));

		if( private_nh.getParam( "odom_topic", odom_topic_ ))
		{
			odom_helper_.setOdomTopic( odom_topic_ );
		}

		initialized_ = true;

		dsrv_ = new dynamic_reconfigure::Server<YmgLPConfig>(private_nh);
		dynamic_reconfigure::Server<YmgLPConfig>::CallbackType cb = boost::bind(&YmgLPROS::reconfigureCB, this, _1, _2);
		dsrv_->setCallback(cb);
	}
	else{
		ROS_WARN("This planner has already been initialized, doing nothing.");
	}
}/*}}}*/

bool YmgLPROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{/*{{{*/
	if (! isInitialized()) {
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
	}
	//when we get a new plan, we also want to clear any latch we may have on goal tolerances
	latchedStopRotateController_.resetLatching();

	// ROS_INFO("Got new plan");
	return ymglp_->setPlan(orig_global_plan);
}/*}}}*/

bool YmgLPROS::isGoalReached()
{/*{{{*/
	if (! isInitialized()) {
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
	}
	if ( ! costmap_ros_->getRobotPose(current_pose_)) {
		ROS_ERROR("Could not get robot pose");
		return false;
	}

	if(latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
		ROS_INFO("Goal reached");
		return true;
	} else {
		return false;
	}
}/*}}}*/

void YmgLPROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path)
{/*{{{*/
	base_local_planner::publishPlan(path, l_plan_pub_);
}/*}}}*/

void YmgLPROS::publishLocalPlanArray(geometry_msgs::PoseArray& path)
{/*{{{*/
	l_plan_array_pub_.publish(path);
}/*}}}*/

void YmgLPROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path)
{/*{{{*/
	base_local_planner::publishPlan(path, g_plan_pub_);
}/*}}}*/

bool YmgLPROS::ymglpComputeVelocityCommands(tf::Stamped<tf::Pose> &global_pose, geometry_msgs::Twist& cmd_vel)
{/*{{{*/
	// dynamic window sampling approach to get useful velocity commands
	if(! isInitialized()){
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
	}

	tf::Stamped<tf::Pose> robot_vel;
	odom_helper_.getRobotVel(robot_vel);

	/* For timing uncomment
		 struct timeval start, end;
		 double start_t, end_t, t_diff;
		 gettimeofday(&start, NULL);
		 */

	//compute what trajectory to drive along
	tf::Stamped<tf::Pose> drive_cmds;
	drive_cmds.frame_id_ = costmap_ros_->getBaseFrameID();

	// call with updated footprint
	base_local_planner::Trajectory path =
		ymglp_->findBestPath(global_pose, robot_vel, drive_cmds, costmap_ros_->getRobotFootprint());
	//ROS_ERROR("Best: %.2f, %.2f, %.2f, %.2f", path.xv_, path.yv_, path.thetav_, path.cost_);

	/* For timing uncomment
		 gettimeofday(&end, NULL);
		 start_t = start.tv_sec + double(start.tv_usec) / 1e6;
		 end_t = end.tv_sec + double(end.tv_usec) / 1e6;
		 t_diff = end_t - start_t;
		 ROS_INFO("Cycle time: %.9f", t_diff);
		 */

	//pass along drive commands
	cmd_vel.linear.x = drive_cmds.getOrigin().getX();
	cmd_vel.linear.y = drive_cmds.getOrigin().getY();
	cmd_vel.angular.z = tf::getYaw(drive_cmds.getRotation());

	//if we cannot move... tell someone
	std::vector<geometry_msgs::PoseStamped> local_plan;
	geometry_msgs::PoseArray local_plan_array;
	if(path.cost_ < 0) {
		ROS_DEBUG_NAMED("ymg_local_planner",
				"The dwa local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
		local_plan.clear();
		publishLocalPlan(local_plan);
		publishLocalPlanArray(local_plan_array);
		return false;
	}

	ROS_DEBUG_NAMED("ymg_local_planner", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.", 
			cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

	// Fill out the local plan
	geometry_msgs::PoseStamped pose;
	for(unsigned int i = 0; i < path.getPointsSize(); ++i) {
		double p_x, p_y, p_th;
		path.getPoint(i, p_x, p_y, p_th);

		tf::Stamped<tf::Pose> p =
			tf::Stamped<tf::Pose>(tf::Pose(
						tf::createQuaternionFromYaw(p_th),
						tf::Point(p_x, p_y, 0.0)),
					ros::Time::now(),
					costmap_ros_->getGlobalFrameID());
		tf::poseStampedTFToMsg(p, pose);
		local_plan.push_back(pose);
		local_plan_array.poses.push_back(pose.pose);
	}

	//publish information to the visualizer

	publishLocalPlan(local_plan);
	local_plan_array.header = pose.header;
	publishLocalPlanArray(local_plan_array);
	return true;
}/*}}}*/

bool YmgLPROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{/*{{{*/
	// dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close enough to goal
	if ( ! costmap_ros_->getRobotPose(current_pose_)) {
		ROS_ERROR("Could not get robot pose");
		return false;
	}
	std::vector<geometry_msgs::PoseStamped> transformed_plan;
	if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
		ROS_ERROR("Could not get local plan");
		return false;
	}

	//if the global plan passed in is empty... we won't do anything
	if(transformed_plan.empty()) {
		ROS_WARN_NAMED("ymg_local_planner", "Received an empty transformed plan.");
		return false;
	}
	ROS_DEBUG_NAMED("ymg_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

	// update plan in dwa_planner even if we just stop and rotate, to allow checkTrajectory
	ymglp_->updatePlanAndLocalCosts(current_pose_, transformed_plan);

	if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)) {
		//publish an empty plan because we've reached our goal position
		std::vector<geometry_msgs::PoseStamped> local_plan;
		std::vector<geometry_msgs::PoseStamped> transformed_plan;
		publishGlobalPlan(transformed_plan);
		publishLocalPlan(local_plan);
		base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
		return latchedStopRotateController_.computeVelocityCommandsStopRotate(
				cmd_vel,
				limits.getAccLimits(),
				ymglp_->getSimPeriod(),
				&planner_util_,
				odom_helper_,
				current_pose_,
				boost::bind(&YmgLP::checkTrajectory, ymglp_, _1, _2, _3));
	}
	else {
		bool isOk = ymglpComputeVelocityCommands(current_pose_, cmd_vel);
		if (isOk) {
			publishGlobalPlan(transformed_plan);
		}
		else {
			ROS_WARN_NAMED("ymg_local_planner", "failed to produce path.");
			std::vector<geometry_msgs::PoseStamped> empty_plan;
			publishGlobalPlan(empty_plan);
		}
		return isOk;
	}
}/*}}}*/

};   // namespace ymglp

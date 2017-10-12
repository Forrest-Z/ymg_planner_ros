#include <ymg_global_planner_hybrid/ymggp_hybrid_ros.h>
#include <ymg_local_planner/util_functions.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>

#include <pcl_conversions/pcl_conversions.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(ymggp::YmgGPHybROS, nav_core::BaseGlobalPlanner);

namespace ymggp {

YmgGPHybROS::YmgGPHybROS() 
/*{{{*/
	: costmap_(NULL),  planner_(), initialized_(false), allow_unknown_(true) {}
/*}}}*/

YmgGPHybROS::YmgGPHybROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
	/*{{{*/
	: costmap_(NULL),  planner_(), initialized_(false), allow_unknown_(true)
{
	initialize(name, costmap_ros);
}/*}}}*/

YmgGPHybROS::YmgGPHybROS(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame)
	/*{{{*/
	: costmap_(NULL),  planner_(), initialized_(false), allow_unknown_(true)
{
	initialize(name, costmap, global_frame);
}/*}}}*/

void YmgGPHybROS::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame)
{/*{{{*/
	if(!initialized_){
		costmap_ = costmap;
		global_frame_ = global_frame;
		planner_ = boost::shared_ptr<navfn::NavFn>(new navfn::NavFn(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY()));

		ros::NodeHandle private_nh("~/" + name);

		navfn_plan_pub_ = private_nh.advertise<nav_msgs::Path>("navfn_plan", 1);
		ymggp_plan_pub_ = private_nh.advertise<nav_msgs::Path>("ymggp_plan", 1);
		navfn_goal_pub_ = private_nh.advertise<geometry_msgs::PointStamped>("navfn_goal", 1);

		private_nh.param("visualize_potential", visualize_potential_, false);

		//if we're going to visualize the potential array we need to advertise
		if(visualize_potential_)
			potarr_pub_.advertise(private_nh, "potential", 1);

		private_nh.param("allow_unknown", allow_unknown_, true);
		private_nh.param("planner_window_x", planner_window_x_, 0.0);
		private_nh.param("planner_window_y", planner_window_y_, 0.0);
		private_nh.param("default_tolerance", default_tolerance_, 0.0);

		private_nh.param("path_granularity", path_granularity_, 0.05);
		private_nh.param("stuck_timeout", stuck_timeout_, 10.0);
		private_nh.param("navfn_goal_dist", navfn_goal_dist_, 5.0);
		private_nh.param("recovery_dist", recovery_dist_, 2.0);

		private_nh.param("stuck_vel", stuck_vel_, 0.05);
		private_nh.param("stuck_rot_vel", stuck_rot_vel_, -1.0);
		private_nh.param("goal_tolerance", goal_tolerance_, 0.3);

		private_nh.param("clear_plan_when_goal_reached", clear_plan_when_goal_reached_, true);

		//get the tf prefix
		ros::NodeHandle prefix_nh;
		tf_prefix_ = tf::getPrefixParam(prefix_nh);

		make_plan_srv_ =  private_nh.advertiseService("make_plan", &YmgGPHybROS::makePlanService, this);

		setNavfnFlag(false);
		ymg_global_planner_.initialize(global_frame, path_granularity_);
		odom_helper_.setOdomTopic("odom");
		robot_status_ = goal_reached;
		stop_time_ = ros::Time::now();

		reset_flag_sub_ = private_nh.subscribe("reset_flag", 100, &YmgGPHybROS::resetFlagCallback, this);
		use_ymggp_force_sub_ = private_nh.subscribe("use_ymggp_force", 100, &YmgGPHybROS::useYmggpForceCallback, this);
		movebase_status_sub_ = private_nh.subscribe("/move_base/status", 100, &YmgGPHybROS::movebaseStatusCallback, this);

		initialized_ = true;
	}
	else
		ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
}/*}}}*/

void YmgGPHybROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{/*{{{*/
	initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}/*}}}*/

void YmgGPHybROS::resetFlagCallback (const std_msgs::Empty& msg)
{/*{{{*/
	ymg_global_planner_.clearPlan();
	setNavfnFlag(false);
	robot_status_ = stopped;
}/*}}}*/

void YmgGPHybROS::useYmggpForceCallback (const std_msgs::Int32& msg)
{/*{{{*/
	switch (msg.data) {
		case 0:
			use_ymggp_force_ = false;
			break;
		case 1:
			use_ymggp_force_ = true;
			break;
		case 2:
			use_ymggp_force_ = true;
			use_navfn_ = false;
			break;
	}
	ROS_INFO("[YmgGPHybROS] subscribed /use_ymggp_force (data : %d)", msg.data);
}/*}}}*/
		
void YmgGPHybROS::movebaseStatusCallback (const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{/*{{{*/
	if (msg->status_list.empty())
		return;

	actionlib_msgs::GoalStatus status = msg->status_list[0];

	if (clear_plan_when_goal_reached_
			&& status.status == actionlib_msgs::GoalStatus::SUCCEEDED) {
		ymg_global_planner_.clearPlan();
		use_navfn_ = false;
		// ROS_INFO_NAMED("YmgGPHyb", "Goal reached. Cleared plan.");
	}
}/*}}}*/

bool YmgGPHybROS::validPointPotential(const geometry_msgs::Point& world_point)
{/*{{{*/
	return validPointPotential(world_point, default_tolerance_);
}/*}}}*/

bool YmgGPHybROS::validPointPotential(const geometry_msgs::Point& world_point, double tolerance)
{/*{{{*/
	if(!initialized_){
		ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
		return false;
	}

	double resolution = costmap_->getResolution();
	geometry_msgs::Point p;
	p = world_point;

	p.y = world_point.y - tolerance;

	while(p.y <= world_point.y + tolerance){
		p.x = world_point.x - tolerance;
		while(p.x <= world_point.x + tolerance){
			double potential = getPointPotential(p);
			if(potential < POT_HIGH){
				return true;
			}
			p.x += resolution;
		}
		p.y += resolution;
	}

	return false;
}/*}}}*/

double YmgGPHybROS::getPointPotential(const geometry_msgs::Point& world_point)
{/*{{{*/
	if(!initialized_){
		ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
		return -1.0;
	}

	unsigned int mx, my;
	if(!costmap_->worldToMap(world_point.x, world_point.y, mx, my))
		return DBL_MAX;

	unsigned int index = my * planner_->nx + mx;
	return planner_->potarr[index];
}/*}}}*/

bool YmgGPHybROS::computePotential(const geometry_msgs::Point& world_point)
{/*{{{*/
	if(!initialized_){
		ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
		return false;
	}

	//make sure to resize the underlying array that Navfn uses
	planner_->setNavArr(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
	planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

	unsigned int mx, my;
	if(!costmap_->worldToMap(world_point.x, world_point.y, mx, my))
		return false;

	int map_start[2];
	map_start[0] = 0;
	map_start[1] = 0;

	int map_goal[2];
	map_goal[0] = mx;
	map_goal[1] = my;

	planner_->setStart(map_start);
	planner_->setGoal(map_goal);

	return planner_->calcNavFnDijkstra();
}/*}}}*/

void YmgGPHybROS::clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my)
{/*{{{*/
	if(!initialized_){
		ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
		return;
	}

	//set the associated costs in the cost map to be free
	costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}/*}}}*/

bool YmgGPHybROS::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp)
{/*{{{*/
	makePlan(req.start, req.goal, resp.plan.poses);

	resp.plan.header.stamp = ros::Time::now();
	resp.plan.header.frame_id = global_frame_;

	return true;
} /*}}}*/

void YmgGPHybROS::mapToWorld(double mx, double my, double& wx, double& wy)
{/*{{{*/
	wx = costmap_->getOriginX() + mx * costmap_->getResolution();
	wy = costmap_->getOriginY() + my * costmap_->getResolution();
}/*}}}*/

bool YmgGPHybROS::makeNavfnPlan (const geometry_msgs::PoseStamped& start,
		const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan)
{/*{{{*/
	boost::mutex::scoped_lock lock(mutex_);

	//clear the plan, just in case
	plan.clear();

	ros::NodeHandle n;

	//until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
	if(tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame_)){
		ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
				tf::resolve(tf_prefix_, global_frame_).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
		return false;
	}

	if(tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame_)){
		ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
				tf::resolve(tf_prefix_, global_frame_).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
		return false;
	}

	double wx = start.pose.position.x;
	double wy = start.pose.position.y;

	unsigned int mx, my;
	if(!costmap_->worldToMap(wx, wy, mx, my)){
		ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
		return false;
	}

	//clear the starting cell within the costmap because we know it can't be an obstacle
	tf::Stamped<tf::Pose> start_pose;
	tf::poseStampedMsgToTF(start, start_pose);
	clearRobotCell(start_pose, mx, my);

#if 0
	{
		static int n = 0;
		static char filename[1000];
		snprintf( filename, 1000, "navfnros-makeplan-costmapB-%04d.pgm", n++ );
		costmap->saveRawMap( std::string( filename ));
	}
#endif

	//make sure to resize the underlying array that Navfn uses
	planner_->setNavArr(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
	planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

#if 0
	{
		static int n = 0;
		static char filename[1000];
		snprintf( filename, 1000, "navfnros-makepwlan-costmapC-%04d", n++ );
		planner_->savemap( filename );
	}
#endif

	int map_start[2];
	map_start[0] = mx;
	map_start[1] = my;

	wx = goal.pose.position.x;
	wy = goal.pose.position.y;

	if(!costmap_->worldToMap(wx, wy, mx, my)){
		if(tolerance <= 0.0){
			ROS_WARN_THROTTLE(1.0, "The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
			return false;
		}
		mx = 0;
		my = 0;
	}

	int map_goal[2];
	map_goal[0] = mx;
	map_goal[1] = my;

	planner_->setStart(map_goal);
	planner_->setGoal(map_start);

	//bool success = planner_->calcNavFnAstar();
	planner_->calcNavFnDijkstra(true);

	double resolution = costmap_->getResolution();
	geometry_msgs::PoseStamped p, best_pose;
	p = goal;

	bool found_legal = false;
	double best_sdist = DBL_MAX;

	p.pose.position.y = goal.pose.position.y - tolerance;

	while(p.pose.position.y <= goal.pose.position.y + tolerance){
		p.pose.position.x = goal.pose.position.x - tolerance;
		while(p.pose.position.x <= goal.pose.position.x + tolerance){
			double potential = getPointPotential(p.pose.position);
			double sdist = ymglp::UtilFcn::calcSqDist(p, goal);
			if(potential < POT_HIGH && sdist < best_sdist){
				best_sdist = sdist;
				best_pose = p;
				found_legal = true;
			}
			p.pose.position.x += resolution;
		}
		p.pose.position.y += resolution;
	}

	if(found_legal){
		//extract the plan
		if(getPlanFromPotential(best_pose, plan)){
			//make sure the goal we push on has the same timestamp as the rest of the plan
			geometry_msgs::PoseStamped goal_copy = best_pose;
			goal_copy.header.stamp = ros::Time::now();
			plan.push_back(goal_copy);
		}
		else{
			ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
		}
	}

	if (visualize_potential_){
		//publish potential array
		pcl::PointCloud<navfn::PotarrPoint> pot_area;
		pot_area.header.frame_id = global_frame_;
		pot_area.points.clear();
		std_msgs::Header header;
		pcl_conversions::fromPCL(pot_area.header, header);
		header.stamp = ros::Time::now();
		pot_area.header = pcl_conversions::toPCL(header);

		navfn::PotarrPoint pt;
		float *pp = planner_->potarr;
		double pot_x, pot_y;
		for (unsigned int i = 0; i < (unsigned int)planner_->ny*planner_->nx ; i++)
		{
			if (pp[i] < 10e7)
			{
				mapToWorld(i%planner_->nx, i/planner_->nx, pot_x, pot_y);
				pt.x = pot_x;
				pt.y = pot_y;
				pt.z = pp[i]/pp[planner_->start[1]*planner_->nx + planner_->start[0]]*20;
				pt.pot_value = pp[i];
				pot_area.push_back(pt);
			}
		}
		potarr_pub_.publish(pot_area);
	}

	return !plan.empty();
}/*}}}*/

bool YmgGPHybROS::makeYmggpPlan (const geometry_msgs::PoseStamped& start,
		const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{/*{{{*/
	plan.clear();
	ymg_global_planner_.makePlan(start, goal, plan);

	return !plan.empty();
}/*}}}*/

bool YmgGPHybROS::makePlan(const geometry_msgs::PoseStamped& start, 
		const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{/*{{{*/
	return makePlan(start, goal, default_tolerance_, plan);
}/*}}}*/

bool YmgGPHybROS::makePlan(const geometry_msgs::PoseStamped& start, 
		const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan)
{/*{{{*/
	if(!initialized_){
		ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
		return false;
	}

	makeYmggpPlan(start, goal, plan);
	publishYmggpPlan(plan);

	updateRobotStatus(start, goal, plan);

	// if the robot is near the navfn goal. changes algorithm to navfn.
	if (use_navfn_ && ymglp::UtilFcn::calcDist(start, navfn_goal_) < recovery_dist_) {
		ROS_INFO("[YmgGPHybROS] Changes planner to ymggp.");
		setNavfnFlag(false);
	}

	if (use_navfn_) {
		// ROS_INFO("path size: %d", (int)plan.size());
		updateNavfnGoal(start, plan);
		makeNavfnPlan(start, navfn_goal_, tolerance, plan);
		// ROS_INFO("dijkstra path size: %d", (int)plan.size());
		publishNavfnPlan(plan);
	}
	else if (robot_status_ == stopped
			&& ros::Duration(stuck_timeout_) < ros::Time::now() - stop_time_
			&& setNavfnFlag(true)) {
		ROS_INFO("[YmgGPHybROS] Changes planner to navfn.");
		// ROS_INFO("path size: %d", (int)plan.size());
		setNavfnGoal(plan);
		makeNavfnPlan(start, navfn_goal_, tolerance, plan);
		publishNavfnPlan(plan);
	}
	else {
		std::vector<geometry_msgs::PoseStamped> empty_plan;
		publishNavfnPlan(empty_plan);
	}

	if (use_navfn_) {
		geometry_msgs::PointStamped msg;
		msg.header = navfn_goal_.header;
		msg.point = navfn_goal_.pose.position;
		navfn_goal_pub_.publish(msg);
		if (plan.empty()) {
			ROS_INFO("[YmgGPHybROS] Navfn faild to produce path.");
		}
	}

	// for debug
	// double dist_min = DBL_MAX, dist_max = -1.0, dist;
	// for (int i=1; i<plan.size(); ++i) {
	// 	dist = ymglp::UtilFcn::calcDist(plan[i-1], plan[i]);
	// 	if (dist < dist_min) {
	// 		dist_min = dist;
	// 	}
	// 	if (dist_max < dist) {
	// 		dist_max = dist;
	// 	}
	// }
	// ROS_INFO("dist between traj point m-M : %f : %f", dist_min, dist_max);

	return !plan.empty();
}/*}}}*/

bool YmgGPHybROS::setNavfnFlag(bool flag)
{/*{{{*/
	use_navfn_ = flag;

	if (use_ymggp_force_) {
		use_navfn_ = false;
	}

	return use_navfn_;
}/*}}}*/

bool YmgGPHybROS::setNavfnGoal (const std::vector<geometry_msgs::PoseStamped>& plan)
{/*{{{*/
	if (plan.empty())
		return false;

	int forward_index = navfn_goal_dist_ / path_granularity_;
	setValidGoal(plan, forward_index);
	// ROS_INFO("start_index = %d", forward_index);

	return true;
}/*}}}*/

bool YmgGPHybROS::updateNavfnGoal (const geometry_msgs::PoseStamped robot_pos,
		const std::vector<geometry_msgs::PoseStamped>& plan)
{/*{{{*/
	unsigned int cell_x, cell_y;
	double px = navfn_goal_.pose.position.x;
	double py = navfn_goal_.pose.position.y;

	// in the costmap and not free space
	if (costmap_->worldToMap(px, py, cell_x, cell_y)
			&& costmap_->getCost(cell_x, cell_y) != costmap_2d::FREE_SPACE)
	{
		int goal_closest_index = ymglp::UtilFcn::getClosestIndexOfPath(navfn_goal_, plan);
		setValidGoal(plan, goal_closest_index);
		return true;
	}

	return false;
}/*}}}*/

bool YmgGPHybROS::setValidGoal(const std::vector<geometry_msgs::PoseStamped>& plan, int start_index)
{/*{{{*/
	double px, py;
	unsigned int cell_x, cell_y;

	for (int i=start_index; i<plan.size(); ++i) {
		px = plan[i].pose.position.x;
		py = plan[i].pose.position.y;
		// out of the costmap or free space
		if (!costmap_->worldToMap(px, py, cell_x, cell_y)
				|| costmap_->getCost(cell_x, cell_y) == costmap_2d::FREE_SPACE) {
			navfn_goal_ = plan[i];
			return true;
		}
	}

	ROS_INFO("setValidGoal function cannot find valid goal. Set navfn goal to the end point of ymggp path.");
	navfn_goal_ = plan.back();

	return false;
}/*}}}*/

void YmgGPHybROS::updateRobotStatus(const geometry_msgs::PoseStamped& start,
		const geometry_msgs::PoseStamped& goal,
		const std::vector<geometry_msgs::PoseStamped>& plan)
{/*{{{*/
	if (ymglp::UtilFcn::calcDist(start, goal) < goal_tolerance_) {
		// ROS_INFO("[YmgGPHybROS] robot status : goal_reached");
		robot_status_ = goal_reached;
		return;
	}

	tf::Stamped<tf::Pose> robot_vel;
	odom_helper_.getRobotVel(robot_vel);
	double robot_v = robot_vel.getOrigin().getX();
	double robot_w = tf::getYaw(robot_vel.getRotation());
	bool v_is_zero = false, omega_is_zero = false;

	if (fabs(robot_v) < stuck_vel_) {
		v_is_zero = true;
	}
	if (fabs(robot_w) < stuck_rot_vel_ || stuck_rot_vel_ < 0.0) {
		omega_is_zero = true;
	}

	// ROS_INFO("stack_rot_vel - robot_w = %f - %f", stuck_rot_vel_, fabs(robot_w));
	// ROS_INFO("v_zero - omega_zero = %d - %d", v_is_zero, omega_is_zero);

	if (v_is_zero && omega_is_zero) {
		// ROS_INFO("[YmgGPHybROS] robot status : stopped");
		if (robot_status_ != stopped) {
			stop_time_ = ros::Time::now();
		}
		robot_status_ = stopped;
	}
	else {
		// ROS_INFO("[YmgGPHybROS] robot status : moving");
		robot_status_ = moving;
	}

	// ROS_INFO("[YmgGPHybROS] robot status : %d", robot_status_);
	if (robot_status_ == stopped) {
		ROS_INFO("[YmgGPHybROS] robot stopped.");
	}

}/*}}}*/

void YmgGPHybROS::publishNavfnPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{/*{{{*/
	if(!initialized_){
		ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
		return;
	}

	//create a message for the plan 
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

	navfn_plan_pub_.publish(gui_path);
}/*}}}*/

void YmgGPHybROS::publishYmggpPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{/*{{{*/
	if(!initialized_){
		ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
		return;
	}

	//create a message for the plan 
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

	ymggp_plan_pub_.publish(gui_path);
}/*}}}*/

bool YmgGPHybROS::getPlanFromPotential(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{/*{{{*/
	if(!initialized_){
		ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
		return false;
	}

	//clear the plan, just in case
	plan.clear();

	//until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
	if(tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame_)){
		ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
				tf::resolve(tf_prefix_, global_frame_).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
		return false;
	}

	double wx = goal.pose.position.x;
	double wy = goal.pose.position.y;

	//the potential has already been computed, so we won't update our copy of the costmap
	unsigned int mx, my;
	if(!costmap_->worldToMap(wx, wy, mx, my)){
		ROS_WARN_THROTTLE(1.0, "The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
		return false;
	}

	int map_goal[2];
	map_goal[0] = mx;
	map_goal[1] = my;

	planner_->setStart(map_goal);

	planner_->calcPath(costmap_->getSizeInCellsX() * 4);

	//extract the plan
	float *x = planner_->getPathX();
	float *y = planner_->getPathY();
	int len = planner_->getPathLen();
	ros::Time plan_time = ros::Time::now();

	for(int i = len - 1; i >= 0; --i){
		//convert the plan to world coordinates
		double world_x, world_y;
		mapToWorld(x[i], y[i], world_x, world_y);

		geometry_msgs::PoseStamped pose;
		pose.header.stamp = plan_time;
		pose.header.frame_id = global_frame_;
		pose.pose.position.x = world_x;
		pose.pose.position.y = world_y;
		pose.pose.position.z = 0.0;
		pose.pose.orientation.x = 0.0;
		pose.pose.orientation.y = 0.0;
		pose.pose.orientation.z = 0.0;
		pose.pose.orientation.w = 1.0;
		plan.push_back(pose);
	}

	//publish the plan for visualization purposes
	publishNavfnPlan(plan);

	return !plan.empty();
}/*}}}*/

};   // namespace ymggp

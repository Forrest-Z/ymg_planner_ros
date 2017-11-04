#include <ymggp_bgp/ymggp_bgp.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <global_planner/dijkstra.h>
#include <global_planner/astar.h>
#include <global_planner/grid_path.h>
#include <global_planner/gradient_path.h>
#include <global_planner/quadratic_calculator.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(ymggp_bgp::YmgGPBGP, nav_core::BaseGlobalPlanner);

namespace ymggp_bgp {

void YmgGPBGP::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value)
{/*{{{*/
	unsigned char* pc = costarr;
	for (int i = 0; i < nx; i++)
		*pc++ = value;
	pc = costarr + (ny - 1) * nx;
	for (int i = 0; i < nx; i++)
		*pc++ = value;
	pc = costarr;
	for (int i = 0; i < ny; i++, pc += nx)
		*pc = value;
	pc = costarr + nx - 1;
	for (int i = 0; i < ny; i++, pc += nx)
		*pc = value;
}/*}}}*/

YmgGPBGP::YmgGPBGP()
	:/*{{{*/
		costmap_(NULL), initialized_(false), allow_unknown_(true) {
		}/*}}}*/

YmgGPBGP::YmgGPBGP(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id)
	:/*{{{*/
		costmap_(NULL), initialized_(false), allow_unknown_(true) {
			//initialize the planner
			initialize(name, costmap, frame_id);
		}/*}}}*/

YmgGPBGP::~YmgGPBGP()
{/*{{{*/
	if (p_calc_)
		delete p_calc_;
	if (planner_)
		delete planner_;
	if (path_maker_)
		delete path_maker_;
	if (dsrv_)
		delete dsrv_;
}/*}}}*/

void YmgGPBGP::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{/*{{{*/
	initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}/*}}}*/

void YmgGPBGP::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id)
{/*{{{*/
	if (!initialized_) {
		ros::NodeHandle private_nh("~/" + name);
		costmap_ = costmap;
		frame_id_ = frame_id;

		unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();

		private_nh.param("old_navfn_behavior", old_navfn_behavior_, false);
		if(!old_navfn_behavior_)
			convert_offset_ = 0.5;
		else
			convert_offset_ = 0.0;

		bool use_quadratic;
		private_nh.param("use_quadratic", use_quadratic, true);
		if (use_quadratic)
			p_calc_ = new global_planner::QuadraticCalculator(cx, cy);
		else
			p_calc_ = new global_planner::PotentialCalculator(cx, cy);

		bool use_dijkstra;
		private_nh.param("use_dijkstra", use_dijkstra, true);
		if (use_dijkstra)
		{
			global_planner::DijkstraExpansion* de = new global_planner::DijkstraExpansion(p_calc_, cx, cy);
			if(!old_navfn_behavior_)
				de->setPreciseStart(true);
			planner_ = de;
		}
		else
			planner_ = new global_planner::AStarExpansion(p_calc_, cx, cy);

		bool use_grid_path;
		private_nh.param("use_grid_path", use_grid_path, false);
		if (use_grid_path)
			path_maker_ = new global_planner::GridPath(p_calc_);
		else
			path_maker_ = new global_planner::GradientPath(p_calc_);

		orientation_filter_ = new global_planner::OrientationFilter();

		// bgp_plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
		potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);

		private_nh.param("allow_unknown", allow_unknown_, true);
		planner_->setHasUnknown(allow_unknown_);
		private_nh.param("planner_window_x", planner_window_x_, 0.0);
		private_nh.param("planner_window_y", planner_window_y_, 0.0);
		private_nh.param("default_tolerance", default_tolerance_, 0.0);
		private_nh.param("publish_scale", publish_scale_, 100);

		double costmap_pub_freq;
		private_nh.param("planner_costmap_publish_frequency", costmap_pub_freq, 0.0);

		//get the tf prefix
		ros::NodeHandle prefix_nh;
		tf_prefix_ = tf::getPrefixParam(prefix_nh);

		make_plan_srv_ = private_nh.advertiseService("make_plan", &YmgGPBGP::makePlanService, this);

		dsrv_ = new dynamic_reconfigure::Server<ymggp_bgp::YmgGPBGPConfig>(ros::NodeHandle("~/" + name));
		dynamic_reconfigure::Server<ymggp_bgp::YmgGPBGPConfig>::CallbackType cb = boost::bind(
				&YmgGPBGP::reconfigureCB, this, _1, _2);
		dsrv_->setCallback(cb);

		initialized_ = true;


		setBGPFlag(false);
		ymg_global_planner_.initialize(frame_id);

		bgp_plan_pub_ = private_nh.advertise<nav_msgs::Path>("bgp_plan", 1);
		ymggp_plan_pub_ = private_nh.advertise<nav_msgs::Path>("ymggp_plan", 1);
		bgp_goal_pub_ = private_nh.advertise<geometry_msgs::PointStamped>("bgp_goal", 1);

		reset_flag_sub_ = private_nh.subscribe("reset_flag", 100, &YmgGPBGP::resetFlagCallback, this);
		use_ymggp_force_sub_ = private_nh.subscribe("use_ymggp_force", 100, &YmgGPBGP::useYmggpForceCallback, this);
		movebase_status_sub_ = private_nh.subscribe("/move_base/status", 100, &YmgGPBGP::movebaseStatusCallback, this);

	}
	else {
		ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
	}


}/*}}}*/

void YmgGPBGP::reconfigureCB(ymggp_bgp::YmgGPBGPConfig& config, uint32_t level)
{/*{{{*/
	planner_->setLethalCost(config.lethal_cost);
	path_maker_->setLethalCost(config.lethal_cost);
	planner_->setNeutralCost(config.neutral_cost);
	planner_->setFactor(config.cost_factor);
	publish_potential_ = config.publish_potential;
	orientation_filter_->setMode(config.orientation_mode);

	path_granularity_ = config.path_granularity;
	ymg_global_planner_.setPathGranularity(path_granularity_);
	// stuck_timeout_ = config.stuck_timeout;
	// robot_status_manager_.setStoppedVel(config.trans_stopped_vel, config.rot_stopped_vel);
	status_manager_.setParameters(config.trans_stopped_vel, config.rot_stopped_vel, config.stuck_timeout);
	bgp_goal_dist_ = config.bgp_goal_dist;
	bgp_goal_max_cost_ = config.bgp_goal_max_cost;
	bgp_goal_pull_back_ = config.bgp_goal_pull_back;
	recovery_dist_ = config.recovery_dist;
	clear_plan_when_goal_reached_ = config.clear_plan_when_goal_reached;
}/*}}}*/

void YmgGPBGP::clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my)
{/*{{{*/
	if (!initialized_) {
		ROS_ERROR(
				"This planner has not been initialized yet, but it is being used, please call initialize() before use");
		return;
	}

	//set the associated costs in the cost map to be free
	costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}/*}}}*/

bool YmgGPBGP::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp)
{/*{{{*/
	makePlan(req.start, req.goal, resp.plan.poses);

	resp.plan.header.stamp = ros::Time::now();
	resp.plan.header.frame_id = frame_id_;

	return true;
}/*}}}*/

void YmgGPBGP::mapToWorld(double mx, double my, double& wx, double& wy)
{/*{{{*/
	wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
	wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
}/*}}}*/

bool YmgGPBGP::worldToMap(double wx, double wy, double& mx, double& my)
{/*{{{*/
	double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
	double resolution = costmap_->getResolution();

	if (wx < origin_x || wy < origin_y)
		return false;

	mx = (wx - origin_x) / resolution - convert_offset_;
	my = (wy - origin_y) / resolution - convert_offset_;

	if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()) {
		return true;
	}

	return false;
}/*}}}*/

bool YmgGPBGP::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
		std::vector<geometry_msgs::PoseStamped>& plan)
{/*{{{*/
	return makePlan(start, goal, default_tolerance_, plan);
}/*}}}*/

bool YmgGPBGP::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
		double tolerance, std::vector<geometry_msgs::PoseStamped>& plan)
{/*{{{*/
	if(!initialized_){
		ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
		return false;
	}

	makeYmggpPlan(start, goal, plan);
	publishYmggpPlan(plan);

	// if the robot is near the BGP goal. changes algorithm to BGP.
	if (use_bgp_ && ymglp::UtilFcn::calcDist(start, bgp_goal_) < recovery_dist_) {
		ROS_INFO("[YmgGPBGP] Changes planner to ymggp.");
		setBGPFlag(false);
	}

	if (use_bgp_) {
		// ROS_INFO("path size: %d", (int)plan.size());
		updateBGPGoal(start, plan);
		makeBGPPlan(start, bgp_goal_, tolerance, plan);
		// ROS_INFO("dijkstra path size: %d", (int)plan.size());
		publishBGPPlan(plan);
	}
	// else if (ros::Duration(stuck_timeout_) < robot_status_manager_.getTimeWhileStopped() && setBGPFlag(true)) {
	else if (status_manager_.isStack() && setBGPFlag(true)) {
		ROS_INFO("[YmgGPBGP] Changes planner to BGP.");
		// ROS_INFO("path size: %d", (int)plan.size());
		setBGPGoal(start, plan);
		makeBGPPlan(start, bgp_goal_, tolerance, plan);
		publishBGPPlan(plan);
	}
	else {
		std::vector<geometry_msgs::PoseStamped> empty_plan;
		publishBGPPlan(empty_plan);
	}

	if (use_bgp_) {
		geometry_msgs::PointStamped msg;
		msg.header = bgp_goal_.header;
		msg.point = bgp_goal_.pose.position;
		bgp_goal_pub_.publish(msg);
		if (plan.empty()) {
			ROS_INFO("[YmgGPBGP] BGP faild to produce path.");
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

// renamed makePlan to makeBGPPlan
bool YmgGPBGP::makeBGPPlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
		double tolerance, std::vector<geometry_msgs::PoseStamped>& plan)
{/*{{{*/
	boost::mutex::scoped_lock lock(mutex_);
	if (!initialized_) {
		ROS_ERROR(
				"This planner has not been initialized yet, but it is being used, please call initialize() before use");
		return false;
	}

	//clear the plan, just in case
	plan.clear();

	ros::NodeHandle n;
	std::string global_frame = frame_id_;

	//until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
	if (tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
		ROS_ERROR(
				"The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
		return false;
	}

	if (tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
		ROS_ERROR(
				"The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
		return false;
	}

	double wx = start.pose.position.x;
	double wy = start.pose.position.y;

	unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
	double start_x, start_y, goal_x, goal_y;

	if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
		ROS_WARN(
				"The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
		return false;
	}
	if(old_navfn_behavior_){
		start_x = start_x_i;
		start_y = start_y_i;
	}else{
		worldToMap(wx, wy, start_x, start_y);
	}

	wx = goal.pose.position.x;
	wy = goal.pose.position.y;

	if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
		ROS_WARN_THROTTLE(1.0,
				"The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
		return false;
	}
	if(old_navfn_behavior_){
		goal_x = goal_x_i;
		goal_y = goal_y_i;
	}else{
		worldToMap(wx, wy, goal_x, goal_y);
	}

	//clear the starting cell within the costmap because we know it can't be an obstacle
	tf::Stamped<tf::Pose> start_pose;
	tf::poseStampedMsgToTF(start, start_pose);
	clearRobotCell(start_pose, start_x_i, start_y_i);

	int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();

	//make sure to resize the underlying array that Navfn uses
	p_calc_->setSize(nx, ny);
	planner_->setSize(nx, ny);
	path_maker_->setSize(nx, ny);
	potential_array_ = new float[nx * ny];

	outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);

	bool found_legal = planner_->calculatePotentials(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y,
			nx * ny * 2, potential_array_);

	if(!old_navfn_behavior_)
		planner_->clearEndpoint(costmap_->getCharMap(), potential_array_, goal_x_i, goal_y_i, 2);
	if(publish_potential_)
		publishPotential(potential_array_);

	if (found_legal) {
		//extract the plan
		if (getPlanFromPotential(start_x, start_y, goal_x, goal_y, goal, plan)) {
			//make sure the goal we push on has the same timestamp as the rest of the plan
			geometry_msgs::PoseStamped goal_copy = goal;
			goal_copy.header.stamp = ros::Time::now();
			plan.push_back(goal_copy);
		} else {
			ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
		}
	}else{
		ROS_ERROR("Failed to get a plan.");
	}

	// add orientations if needed
	orientation_filter_->processPath(start, plan);

	//publish the plan for visualization purposes
	// publishPlan(plan);
	addYmggpPlan(plan);   // XXX added
	delete potential_array_;
	return !plan.empty();
}/*}}}*/

bool YmgGPBGP::makeYmggpPlan (const geometry_msgs::PoseStamped& start,
		const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{/*{{{*/
	plan.clear();
	ymg_global_planner_.makePlan(start, goal, plan);

	return !plan.empty();
}/*}}}*/

void YmgGPBGP::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{/*{{{*/
	if (!initialized_) {
		ROS_ERROR(
				"This planner has not been initialized yet, but it is being used, please call initialize() before use");
		return;
	}

	//create a message for the plan
	nav_msgs::Path gui_path;
	gui_path.poses.resize(path.size());

	gui_path.header.frame_id = frame_id_;
	gui_path.header.stamp = ros::Time::now();

	// Extract the plan in world co-ordinates, we assume the path is all in the same frame
	for (unsigned int i = 0; i < path.size(); i++) {
		gui_path.poses[i] = path[i];
	}

	bgp_plan_pub_.publish(gui_path);
}/*}}}*/

bool YmgGPBGP::getPlanFromPotential(double start_x, double start_y, double goal_x, double goal_y,
		const geometry_msgs::PoseStamped& goal,
		std::vector<geometry_msgs::PoseStamped>& plan)
{/*{{{*/
	if (!initialized_) {
		ROS_ERROR(
				"This planner has not been initialized yet, but it is being used, please call initialize() before use");
		return false;
	}

	std::string global_frame = frame_id_;

	//clear the plan, just in case
	plan.clear();

	std::vector<std::pair<float, float> > path;

	if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path)) {
		ROS_ERROR("NO PATH!");
		return false;
	}

	ros::Time plan_time = ros::Time::now();
	for (int i = path.size() -1; i>=0; i--) {
		std::pair<float, float> point = path[i];
		//convert the plan to world coordinates
		double world_x, world_y;
		mapToWorld(point.first, point.second, world_x, world_y);

		geometry_msgs::PoseStamped pose;
		pose.header.stamp = plan_time;
		pose.header.frame_id = global_frame;
		pose.pose.position.x = world_x;
		pose.pose.position.y = world_y;
		pose.pose.position.z = 0.0;
		pose.pose.orientation.x = 0.0;
		pose.pose.orientation.y = 0.0;
		pose.pose.orientation.z = 0.0;
		pose.pose.orientation.w = 1.0;
		plan.push_back(pose);
	}
	if(old_navfn_behavior_){
		plan.push_back(goal);
	}
	return !plan.empty();
}/*}}}*/

void YmgGPBGP::publishPotential(float* potential)
{/*{{{*/
	int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
	double resolution = costmap_->getResolution();
	nav_msgs::OccupancyGrid grid;
	// Publish Whole Grid
	grid.header.frame_id = frame_id_;
	grid.header.stamp = ros::Time::now();
	grid.info.resolution = resolution;

	grid.info.width = nx;
	grid.info.height = ny;

	double wx, wy;
	costmap_->mapToWorld(0, 0, wx, wy);
	grid.info.origin.position.x = wx - resolution / 2;
	grid.info.origin.position.y = wy - resolution / 2;
	grid.info.origin.position.z = 0.0;
	grid.info.origin.orientation.w = 1.0;

	grid.data.resize(nx * ny);

	float max = 0.0;
	for (unsigned int i = 0; i < grid.data.size(); i++) {
		float potential = potential_array_[i];
		if (potential < POT_HIGH) {
			if (potential > max) {
				max = potential;
			}
		}
	}

	for (unsigned int i = 0; i < grid.data.size(); i++) {
		if (potential_array_[i] >= POT_HIGH) {
			grid.data[i] = -1;
		} else
			grid.data[i] = potential_array_[i] * publish_scale_ / max;
	}
	potential_pub_.publish(grid);
}/*}}}*/

// ########## added
bool YmgGPBGP::setBGPFlag(bool flag)
{/*{{{*/
	use_bgp_ = flag;

	if (use_ymggp_force_) {
		use_bgp_ = false;
	}

	return use_bgp_;
}/*}}}*/

bool YmgGPBGP::setBGPGoal (const geometry_msgs::PoseStamped robot_pos,
		const std::vector<geometry_msgs::PoseStamped>& ymggp_plan)
{/*{{{*/
	if (ymggp_plan.empty())
		return false;

	// int forward_index = bgp_goal_dist_ / path_granularity_;

	// XXX added but has not tested yet
	int forward_index = ymggp_plan.size()-1;
	double dist = ymglp::UtilFcn::calcDist(robot_pos, ymggp_plan[0]);
	for (int i=1; i<ymggp_plan.size(); ++i) {
		dist += ymglp::UtilFcn::calcDist(ymggp_plan[i-1], ymggp_plan[i]);
		if (bgp_goal_dist_ < dist) {
			forward_index = i;
			break;
		}
	}

	setValidGoal(ymggp_plan, forward_index);
	// ROS_INFO("start_index = %d", forward_index);

	return true;
}/*}}}*/

bool YmgGPBGP::updateBGPGoal (const geometry_msgs::PoseStamped robot_pos,
		const std::vector<geometry_msgs::PoseStamped>& ymggp_plan)
{/*{{{*/
	unsigned int cell_x, cell_y;
	double px = bgp_goal_.pose.position.x;
	double py = bgp_goal_.pose.position.y;

	// in the costmap and not free space
	if (costmap_->worldToMap(px, py, cell_x, cell_y)
			&& bgp_goal_max_cost_ < costmap_->getCost(cell_x, cell_y))
			// && costmap_->getCost(cell_x, cell_y) != costmap_2d::FREE_SPACE)
	{
		int goal_closest_index = ymglp::UtilFcn::getClosestIndexOfPath(bgp_goal_, ymggp_plan);
		setValidGoal(ymggp_plan, goal_closest_index);
		return true;
	}

	return false;
}/*}}}*/

bool YmgGPBGP::setValidGoal(const std::vector<geometry_msgs::PoseStamped>& plan, int start_index)
{/*{{{*/
	double px, py;
	unsigned int cell_x, cell_y;

	for (int i=start_index; i<plan.size(); ++i) {
		px = plan[i].pose.position.x;
		py = plan[i].pose.position.y;
		// out of the costmap or free space
		if (!costmap_->worldToMap(px, py, cell_x, cell_y)
				|| costmap_->getCost(cell_x, cell_y) <= bgp_goal_max_cost_) {
				// || costmap_->getCost(cell_x, cell_y) == costmap_2d::FREE_SPACE) {
			bgp_goal_ = plan[i];
			return true;
		}
	}
	
	// XXX added 
	if (bgp_goal_pull_back_) {
		for (int i=start_index-1; 0<=i; --i) {
			px = plan[i].pose.position.x;
			py = plan[i].pose.position.y;
			// out of the costmap or free space
			if (!costmap_->worldToMap(px, py, cell_x, cell_y)
					|| costmap_->getCost(cell_x, cell_y) <= bgp_goal_max_cost_) {
				bgp_goal_ = plan[i];
				return true;
			}
		}
	}

	ROS_INFO("setValidGoal function cannot find valid goal. Set BGP goal to the end point of ymggp path.");
	bgp_goal_ = plan.back();

	return false;
}/*}}}*/

void YmgGPBGP::addYmggpPlan(std::vector<geometry_msgs::PoseStamped>& plan)
{/*{{{*/
	int closest_index;
	if (plan.empty()) {
		closest_index = 0;
	}
	else {
		closest_index = ymglp::UtilFcn::getClosestIndexOfPath(plan.back(), ymg_global_planner_.plan_);
	}

	for (int i=closest_index; i<ymg_global_planner_.plan_.size(); ++i) {
		plan.push_back(ymg_global_planner_.plan_[i]);
	}
}/*}}}*/

void YmgGPBGP::publishBGPPlan(const std::vector<geometry_msgs::PoseStamped>& path)
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

	bgp_plan_pub_.publish(gui_path);
}/*}}}*/

void YmgGPBGP::publishYmggpPlan(const std::vector<geometry_msgs::PoseStamped>& path)
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

void YmgGPBGP::resetFlagCallback (const std_msgs::Empty& msg)
{/*{{{*/
	ymg_global_planner_.clearPlan();
	setBGPFlag(false);
	// robot_status_manager_.clearStoppedTime();
	status_manager_.clearStatus();
	ROS_INFO("Reset flag received. Cleared plan.");
}/*}}}*/

void YmgGPBGP::useYmggpForceCallback (const std_msgs::Int32& msg)
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
			use_bgp_ = false;
			break;
	}
	ROS_INFO("[YmgGPBGP] subscribed /use_ymggp_force (data : %d)", msg.data);
}/*}}}*/
		
void YmgGPBGP::movebaseStatusCallback (const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{/*{{{*/
	static bool cleared = 0;

	if (msg->status_list.empty())
		return;

	actionlib_msgs::GoalStatus status = msg->status_list[0];

	if (clear_plan_when_goal_reached_
			&& status.status == actionlib_msgs::GoalStatus::SUCCEEDED) {
		if (!cleared) {
			cleared = true;
			use_bgp_ = false;
			ymg_global_planner_.clearPlan();
			ROS_INFO("[YmgGPBGP] Goal reached. Cleared plan.");
		}
	}
	else {
		cleared = false;
	}
}/*}}}*/

} //end namespace ymggp


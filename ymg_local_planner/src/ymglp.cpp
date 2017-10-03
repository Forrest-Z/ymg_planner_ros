#include <ymg_local_planner/ymglp.h>

#include <base_local_planner/goal_functions.h>
#include <base_local_planner/map_grid_cost_point.h>
#include <ymg_local_planner/map_grid_cost_function_kai.h>
#include <ymg_local_planner/util_function.h>
#include <cmath>

//for computing path distance
#include <queue>
#include <angles/angles.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

namespace ymglp {

void YmgLP::reconfigure (YmgLPConfig &config)
{/*{{{*/

	boost::mutex::scoped_lock l(configuration_mutex_);

	use_dwa_ = config.use_dwa;

	generator_.setParameters(
			config.sim_time, config.sim_granularity, config.angular_sim_granularity, sim_period_);

	ymg_sampling_planner_.setParameters(
			config.sim_time, config.sim_granularity, config.angular_sim_granularity, sim_period_);

	ymg_sampling_planner_.setTolerance(config.path_tolerance, config.obstacle_tolerance);

	double resolution = planner_util_->getCostmap()->getResolution();
	pdist_scale_ = config.path_distance_bias;
	if (!use_dwa_)
		path_costs_.setScale(resolution);
	else
		path_costs_.setScale(resolution * pdist_scale_);
	path_costs_.setForwardPointDist(config.forward_point_dist);

	gdist_scale_ = config.goal_distance_bias;
	goal_costs_.setScale(resolution * gdist_scale_);

	occdist_scale_ = config.occdist_scale;
	if (!use_dwa_)
		obstacle_costs_.setScale(1.0);
	else
		obstacle_costs_.setScale(resolution * occdist_scale_);
	obstacle_costs_.setSimGranularity(config.sim_granularity);
	obstacle_costs_.setForwardPointDist(config.forward_point_dist_obstacle);

	// obstacle costs can vary due to scaling footprint feature
	double max_vel_abs = std::max( fabs(config.min_vel_x), fabs(config.max_vel_x) );
	obstacle_costs_.setParams(max_vel_abs, config.max_scaling_factor, config.scaling_speed);

	local_goal_distance_ = config.local_goal_distance;

	int vx_samp, vy_samp, vth_samp;
	vx_samp = config.vx_samples;
	vy_samp = config.vy_samples;
	vth_samp = config.vth_samples;

	if (vx_samp <= 0) {
		ROS_WARN("You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
		vx_samp = 1;
		config.vx_samples = vx_samp;
	}

	if (vy_samp <= 0) {
		ROS_WARN("You've specified that you don't want any samples in the y dimension. We'll at least assume that you want to sample one value... so we're going to set vy_samples to 1 instead");
		vy_samp = 1;
		config.vy_samples = vy_samp;
	}

	if (vth_samp <= 0) {
		ROS_WARN("You've specified that you don't want any samples in the th dimension. We'll at least assume that you want to sample one value... so we're going to set vth_samples to 1 instead");
		vth_samp = 1;
		config.vth_samples = vth_samp;
	}

	vsamples_[0] = vx_samp;
	vsamples_[1] = vy_samp;
	vsamples_[2] = vth_samp;


}/*}}}*/

YmgLP::YmgLP (std::string name, base_local_planner::LocalPlannerUtil *planner_util)
	/*{{{*/
	: planner_util_(planner_util),
	obstacle_costs_(planner_util->getCostmap()),
	path_costs_(planner_util->getCostmap(), false),
	goal_costs_(planner_util->getCostmap(), true)
{
	ros::NodeHandle private_nh("~/" + name);

	//Assuming this planner is being run within the navigation stack, we can
	//just do an upward search for the frequency at which its being run. This
	//also allows the frequency to be overwritten locally.
	std::string controller_frequency_param_name;
	if(!private_nh.searchParam("controller_frequency", controller_frequency_param_name)) {
		sim_period_ = 0.05;
	} else {
		double controller_frequency = 0;
		private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
		if(controller_frequency > 0) {
			sim_period_ = 1.0 / controller_frequency;
		} else {
			ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
			sim_period_ = 0.05;
		}
	}
	ROS_INFO("Sim period is set to %.2f", sim_period_);

	private_nh.param("publish_cost_grid_pc", publish_cost_grid_pc_, false);
	map_viz_.initialize(name, planner_util->getGlobalFrame(), boost::bind(&YmgLP::getCellCosts, this, _1, _2, _3, _4, _5, _6));

	std::string frame_id;
	private_nh.param("global_frame_id", frame_id, std::string("odom"));

	traj_cloud_ = new pcl::PointCloud<base_local_planner::MapGridCostPoint>;
	traj_cloud_->header.frame_id = frame_id;
	traj_cloud_pub_.advertise(private_nh, "trajectory_cloud", 1);
	private_nh.param("publish_traj_pc", publish_traj_pc_, false);

	// set up all the cost functions that will be applied in order
	// (any function returning negative values will abort scoring, so the order can improve performance)
	std::vector<base_local_planner::TrajectoryCostFunction*> critics;
	critics.push_back(&path_costs_); // prefers trajectories on global path
	critics.push_back(&goal_costs_); // prefers trajectories that go towards (local) goal, based on wave propagation
	critics.push_back(&obstacle_costs_); // discards trajectories that move into obstacles

	// trajectory generators
	std::vector<base_local_planner::TrajectorySampleGenerator*> generator_list;
	generator_list.push_back(&generator_);

	use_dwa_ = false;
	scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlannerKai(generator_list, critics);
	ymg_sampling_planner_ = YmgSamplingPlanner(&path_costs_, &obstacle_costs_);
	local_goal_pub_ = private_nh.advertise<geometry_msgs::PointStamped>("local_goal", 1);

	private_nh.param("cheat_factor", cheat_factor_, 1.0);
}/*}}}*/

// used for visualization only, total_costs are not really total costs
bool YmgLP::getCellCosts (int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost)
{/*{{{*/
	path_cost = path_costs_.getCellCosts(cx, cy);
	goal_cost = goal_costs_.getCellCosts(cx, cy);
	occ_cost = planner_util_->getCostmap()->getCost(cx, cy);
	if (path_cost == path_costs_.obstacleCosts() ||
			path_cost == path_costs_.unreachableCellCosts() ||
			occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
		return false;
	}

	double resolution = planner_util_->getCostmap()->getResolution();
	total_cost =
		pdist_scale_ * resolution * path_cost +
		gdist_scale_ * resolution * goal_cost +
		occdist_scale_ * occ_cost;
	return true;
}/*}}}*/

bool YmgLP::setPlan (const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{/*{{{*/
	return planner_util_->setPlan(orig_global_plan);
}/*}}}*/

/**
 * This function is used when other strategies are to be applied,
 * but the cost functions for obstacles are to be reused.
 */
bool YmgLP::checkTrajectory (Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f vel_samples)
{/*{{{*/
	base_local_planner::Trajectory traj;
	geometry_msgs::PoseStamped goal_pose = global_plan_.back();
	Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf::getYaw(goal_pose.pose.orientation));
	base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
	generator_.initialise(pos,
			vel,
			goal,
			&limits,
			vsamples_);
	generator_.generateTrajectory(pos, vel, vel_samples, traj);
	double cost = scored_sampling_planner_.scoreTrajectory(traj, -1);
	//if the trajectory is a legal one... the check passes
	if(cost >= 0) {
		return true;
	}
	ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vel_samples[0], vel_samples[1], vel_samples[2], cost);

	//otherwise the check fails
	return false;
}/*}}}*/

void YmgLP::updatePlanAndLocalCosts (tf::Stamped<tf::Pose> global_pose,
		const std::vector<geometry_msgs::PoseStamped>& new_plan)
{/*{{{*/
	global_plan_.resize(new_plan.size());
	for (unsigned int i = 0; i < new_plan.size(); ++i) {
		global_plan_[i] = new_plan[i];
	}

	if (!use_dwa_) {
		path_costs_.setTargetPoses(new_plan);
		goal_costs_.setTargetPoses(new_plan);
	}
	else {
		std::vector<geometry_msgs::PoseStamped> shortened_plan;
		shortenPath(global_pose, new_plan, shortened_plan);

		path_costs_.setTargetPoses(shortened_plan);
		goal_costs_.setTargetPoses(shortened_plan);

		geometry_msgs::PointStamped local_goal_msg;
		local_goal_msg.header = new_plan.front().header;
		local_goal_msg.point.x = shortened_plan.back().pose.position.x;
		local_goal_msg.point.y = shortened_plan.back().pose.position.y;
		local_goal_msg.point.z = 0.0;
		local_goal_pub_.publish(local_goal_msg);
	}

}/*}}}*/

void YmgLP::shortenPath(const tf::Stamped<tf::Pose>& global_pose,
		const std::vector<geometry_msgs::PoseStamped>& orig_plan,
		std::vector<geometry_msgs::PoseStamped>& shortened_plan)
{/*{{{*/
	geometry_msgs::PoseStamped current_pose;
	current_pose.pose.position.x = global_pose.getOrigin().getX();
	current_pose.pose.position.y = global_pose.getOrigin().getY();

	int closest_index = getClosestIndexOfPath(current_pose, orig_plan);
	if (closest_index < 0) return;

	// calc local goal and shorten the global plan
	double now_distance = 0.0;
	int local_goal_index = -1;
	for (int i=closest_index+1; i<orig_plan.size(); ++i) {
		now_distance += calcDist(orig_plan[i-1], orig_plan[i]);
		if (local_goal_distance_ < now_distance) {
			local_goal_index = i;
			break;
		}
	}

	// resize orig_plan
	shortened_plan.clear();
	if (local_goal_index == -1) {
		shortened_plan = orig_plan;
	} else {
		for (int i=closest_index; i<local_goal_index; ++i) {
			shortened_plan.push_back(orig_plan[i]);
		}
	}
}/*}}}*/

/*
 * given the current state of the robot, find a good trajectory
 */
base_local_planner::Trajectory YmgLP::findBestPath (
		tf::Stamped<tf::Pose> global_pose,
		tf::Stamped<tf::Pose> global_vel,
		tf::Stamped<tf::Pose>& drive_velocities,
		std::vector<geometry_msgs::Point> footprint_spec)
{/*{{{*/
	obstacle_costs_.setFootprint(footprint_spec);

	//make sure that our configuration doesn't change mid-run
	boost::mutex::scoped_lock l(configuration_mutex_);

	Eigen::Vector3f pos(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), tf::getYaw(global_pose.getRotation()));
	Eigen::Vector3f vel(global_vel.getOrigin().getX(), global_vel.getOrigin().getY(), tf::getYaw(global_vel.getRotation()));
	geometry_msgs::PoseStamped goal_pose = global_plan_.back();
	Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf::getYaw(goal_pose.pose.orientation));
	base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();

	// prepare cost functions and generators for this run
	generator_.initialise(pos, vel, goal, &limits, vsamples_);
	ymg_sampling_planner_.initialize(&limits, pos, vel, vsamples_);

	std::vector<base_local_planner::Trajectory> all_explored;

	result_traj_.cost_ = -7;

	if (!use_dwa_)
		ymg_sampling_planner_.findBestTrajectory(result_traj_, &all_explored);
	else
		scored_sampling_planner_.findBestTrajectory(result_traj_, &all_explored);

	if(publish_traj_pc_)
	{
		base_local_planner::MapGridCostPoint pt;
		traj_cloud_->points.clear();
		traj_cloud_->width = 0;
		traj_cloud_->height = 0;
		std_msgs::Header header;
		pcl_conversions::fromPCL(traj_cloud_->header, header);
		header.stamp = ros::Time::now();
		traj_cloud_->header = pcl_conversions::toPCL(header);
		for(std::vector<base_local_planner::Trajectory>::iterator t=all_explored.begin(); t != all_explored.end(); ++t)
		{
			if(t->cost_<0)
				continue;
			// Fill out the plan
			for(unsigned int i = 0; i < t->getPointsSize(); ++i) {
				double p_x, p_y, p_th;
				t->getPoint(i, p_x, p_y, p_th);
				pt.x=p_x;
				pt.y=p_y;
				pt.z=0;
				pt.path_cost=p_th;
				pt.total_cost=t->cost_;
				traj_cloud_->push_back(pt);
			}
		}
		traj_cloud_pub_.publish(*traj_cloud_);
	}

	// verbose publishing of point clouds
	if (publish_cost_grid_pc_) {
		//we'll publish the visualization of the costs to rviz before returning our best trajectory
		map_viz_.publishCostCloud(planner_util_->getCostmap());
	}

	// debrief stateful scoring functions

	//if we don't have a legal trajectory, we'll just command zero
	if (result_traj_.cost_ < 0) {
		drive_velocities.setIdentity();
	} else {
		tf::Vector3 start(result_traj_.xv_, result_traj_.yv_, 0);
		drive_velocities.setOrigin(start);
		tf::Matrix3x3 matrix;
		matrix.setRotation(tf::createQuaternionFromYaw(result_traj_.thetav_));
		drive_velocities.setBasis(matrix);
	}

	return result_traj_;
}/*}}}*/

};

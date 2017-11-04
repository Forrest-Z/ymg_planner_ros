#include <ymg_local_planner/ymglp.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/map_grid_cost_point.h>
#include <ymg_local_planner/util_functions.h>
#include <cmath>

// for computing path distance
#include <queue>
#include <angles/angles.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

namespace ymglp {

void YmgLP::reconfigure (YmgLPConfig &config)
{/*{{{*/

	boost::mutex::scoped_lock l(configuration_mutex_);

	use_dwa_ = config.use_dwa;

	if (0.0 < config.max_vel_x)
		reverse_mode_ = false;
	else
		reverse_mode_ = true;

	generator_.setParameters(
			config.sim_time, config.sim_granularity, config.angular_sim_granularity, sim_period_);
	ymg_sampling_planner_.setParameters(
			config.sim_time, config.sim_granularity, config.angular_sim_granularity, sim_period_,
			config.path_tolerance, config.obstacle_tolerance);
	simple_backup_planner_.setParameters(
			config.sim_time, config.sim_granularity, config.angular_sim_granularity, sim_period_,
			config.obstacle_tolerance, config.backup_vel);
	// stuck_timeout_ = config.stuck_timeout;
	backup_time_ = config.backup_time;

	// robot_status_manager_.setStoppedVel(config.trans_stopped_vel, config.rot_stopped_vel);
	status_manager_.setParameters(config.trans_stopped_vel, config.rot_stopped_vel, config.stuck_timeout);


	utilfcn_.setScoringPointOffsetX(config.scoring_point_offset_x);

	double resolution = planner_util_->getCostmap()->getResolution();
	pdist_scale_ = config.path_distance_bias;
	if (!use_dwa_)
		path_costs_.setScale(resolution);
	else
		path_costs_.setScale(resolution * pdist_scale_);
	path_costs_.setForwardPointDist(config.scoring_point_offset_x);

	gdist_scale_ = config.goal_distance_bias;
	goal_costs_.setScale(resolution * gdist_scale_);

	occdist_scale_ = config.occdist_scale;
	if (!use_dwa_)
		obstacle_costs_.setScale(1.0);
	else
		obstacle_costs_.setScale(resolution * occdist_scale_);
	obstacle_costs_.setSimGranularity(config.sim_granularity);
	obstacle_costs_.setForwardPointDist(config.obstacle_stop_margin);

	// obstacle costs can vary due to scaling footprint feature
	double max_vel_abs = fabs(config.max_vel_x);
	obstacle_costs_.setParams(max_vel_abs, config.max_scaling_factor, config.scaling_speed);

	local_goal_distance_ = config.local_goal_distance;

	int vx_samp = config.vx_samples;
	int vth_samp = config.vth_samples;

	if (vx_samp <= 0) {
		ROS_WARN("You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
		vx_samp = 1;
		config.vx_samples = vx_samp;
	}

	if (vth_samp <= 0) {
		ROS_WARN("You've specified that you don't want any samples in the th dimension. We'll at least assume that you want to sample one value... so we're going to set vth_samples to 1 instead");
		vth_samp = 1;
		config.vth_samples = vth_samp;
	}

	vsamples_[0] = vx_samp;
	vsamples_[1] = 1;
	vsamples_[2] = vth_samp;

}/*}}}*/

YmgLP::YmgLP (std::string name, base_local_planner::LocalPlannerUtil *planner_util)
	/*{{{*/
	: planner_util_(planner_util),
	obstacle_costs_(planner_util->getCostmap()),
	path_costs_(planner_util->getCostmap(), false),
	goal_costs_(planner_util->getCostmap(), true),
	ymg_sampling_planner_(&obstacle_costs_, &utilfcn_),
	simple_backup_planner_(&obstacle_costs_)
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
	backup_latch_ = false;
	stuck_timeout_ = -1.0;
	scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlannerKai(generator_list, critics);
	local_goal_pub_ = private_nh.advertise<geometry_msgs::PointStamped>("local_goal", 1);
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

	utilfcn_.setPose(global_pose);
	utilfcn_.setPlan(global_plan_);

	if (!use_dwa_) {
		path_costs_.setTargetPoses(global_plan_);
		goal_costs_.setTargetPoses(global_plan_);
	}
	else {
		std::vector<geometry_msgs::PoseStamped> shortened_plan;
		utilfcn_.getShortenedPlan(local_goal_distance_, shortened_plan);
		// shortenPath(global_plan_, shortened_plan, nearest_index_, local_goal_distance_);

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

void YmgLP::publishTrajPC(std::vector<base_local_planner::Trajectory>& all_explored)
{/*{{{*/
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

	Eigen::Vector3f pos(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(),
			tf::getYaw(global_pose.getRotation()));
	Eigen::Vector3f vel(global_vel.getOrigin().getX(), global_vel.getOrigin().getY(),
			tf::getYaw(global_vel.getRotation()));
	geometry_msgs::PoseStamped goal_pose = global_plan_.back();
	Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y,
			tf::getYaw(goal_pose.pose.orientation));
	base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();

	utilfcn_.setPose(global_pose);

	std::vector<base_local_planner::Trajectory> all_explored;
	result_traj_.cost_ = -7;

	if (!backup_latch_ && 0.0 < stuck_timeout_ && status_manager_.isStack()) {
			// && ros::Duration(stuck_timeout_) < robot_status_manager_.getTimeWhileStopped()) {
		backup_latch_ = true;
		backup_start_time_ = ros::Time::now();
		ROS_INFO("[ymglp] Robot stopped while %f sec. Try backup.", stuck_timeout_);
	}

	if (!use_dwa_) {
		if (backup_latch_) {
			simple_backup_planner_.initialize(&limits, pos, vel);
			simple_backup_planner_.findBestTrajectory(result_traj_, &all_explored);

			if (ros::Duration(backup_time_) < ros::Time::now() - backup_start_time_) {
				backup_latch_ = false;
				status_manager_.clearStatus();
				ROS_INFO("[ymglp] Backup end.");
			}
		}
		else {
			ymg_sampling_planner_.initialize(&limits, pos, vel, vsamples_);
			ymg_sampling_planner_.findBestTrajectory(result_traj_, &all_explored);
		}
	}
	else {
		generator_.initialise(pos, vel, goal, &limits, vsamples_);
		scored_sampling_planner_.findBestTrajectory(result_traj_, &all_explored);
	}

	if(publish_traj_pc_) {
		publishTrajPC(all_explored);
	}

	//if we don't have a legal trajectory, we'll just command zero
	if (result_traj_.cost_ < 0) {
		drive_velocities.setIdentity();
	}
	else {
		// ROS_INFO("v: %f, %f, %f", result_traj_.xv_, result_traj_.yv_, result_traj_.thetav_);
		tf::Vector3 start(result_traj_.xv_, result_traj_.yv_, 0);
		drive_velocities.setOrigin(start);
		tf::Matrix3x3 matrix;
		matrix.setRotation(tf::createQuaternionFromYaw(result_traj_.thetav_));
		drive_velocities.setBasis(matrix);
	}

	return result_traj_;
}/*}}}*/

}   // namespace ymglp

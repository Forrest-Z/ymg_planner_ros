#include <ymg_local_planner/robot_status_manager.h>

namespace ymglp {

RobotStatusManager::RobotStatusManager()
: /*{{{*/
	stuck_vel_(-1.0), stuck_rot_vel_(-1.0)
{
	odom_helper_.setOdomTopic("odom");
	robot_status_ = GOAL_REACHED;

	ros::NodeHandle nh;
	movebase_status_sub_ = nh.subscribe("/move_base/status", 100, &RobotStatusManager::movebaseStatusCallback, this);
}/*}}}*/

void RobotStatusManager::updateRobotStatus()
{/*{{{*/
	static RobotStatus robot_status_past = robot_status_;

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
		if (robot_status_ != STOPPED) {
			stop_time_ = ros::Time::now();
		}
		robot_status_ = STOPPED;
	}
	else {
		robot_status_ = MOVING;
	}

	if (robot_status_ != robot_status_past) {
		// ROS_INFO("[RSM] robot status %s.", robotStatusToString(robot_status_).c_str());
	}
		ROS_INFO("[RSM] robot status %s.", robotStatusToString(robot_status_).c_str());
	robot_status_past = robot_status_;

}/*}}}*/

void RobotStatusManager::setRobotStatus(RobotStatus status)
{/*{{{*/
	robot_status_ = status;
}/*}}}*/

ros::Duration RobotStatusManager::getTimeWhileStopped()
{/*{{{*/
	if (robot_status_ != STOPPED) {
		return ros::Duration(0.0);
	}

	return ros::Time::now() - stop_time_;
}/*}}}*/

std::string RobotStatusManager::robotStatusToString(RobotStatus status)
{/*{{{*/
	switch (status) {
		case MOVING:
			return "MOVING";
		case STOPPED:
			return "STOPPED";
		case GOAL_REACHED:
			return "GOAL_REACHED";
	}

	return "Cannot convert to string.";
}/*}}}*/

void RobotStatusManager::movebaseStatusCallback (const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{/*{{{*/
	static bool cleared = 0;

	if (msg->status_list.empty())
		return;

	actionlib_msgs::GoalStatus status = msg->status_list[0];

	if (status.status == actionlib_msgs::GoalStatus::SUCCEEDED) {
		robot_status_ = GOAL_REACHED;
	}
}/*}}}*/

}// namespace ymglp

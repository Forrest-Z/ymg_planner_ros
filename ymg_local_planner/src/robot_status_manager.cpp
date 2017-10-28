#include <ymg_local_planner/robot_status_manager.h>

namespace ymglp {

RobotStatusManager::RobotStatusManager()
: /*{{{*/
	trans_stopped_vel_(-1.0), rot_stopped_vel_(-1.0)
{
	is_goal_reached_ = true;
	is_robot_moving_ = false;

	ros::NodeHandle nh;
	movebase_status_sub_ = nh.subscribe("/move_base/status", 100, &RobotStatusManager::movebaseStatusCallback, this);
	cmd_vel_sub_ = nh.subscribe("/cmd_vel", 100, &RobotStatusManager::cmdVelCallback, this);
}/*}}}*/

void RobotStatusManager::updateRobotStatus(double robot_v, double robot_w)
{/*{{{*/
	static std::string robot_status_past = getRobotStatusString();

	bool v_is_zero = false, omega_is_zero = false;
	if (fabs(robot_v) < trans_stopped_vel_) {
		v_is_zero = true;
	}
	if (fabs(robot_w) < rot_stopped_vel_ || rot_stopped_vel_ < 0.0) {
		omega_is_zero = true;
	}

	ROS_INFO("stack_rot_vel - robot_w = %f - %f", rot_stopped_vel_, fabs(robot_w));
	ROS_INFO("v_zero - omega_zero = %d - %d", v_is_zero, omega_is_zero);

	if (v_is_zero && omega_is_zero) {
		if (is_robot_moving_) {
			stopped_time_ = ros::Time::now();
			is_robot_moving_ = false;
		}
	}
	else {
		is_robot_moving_ = true;
	}

	std::string robot_status = getRobotStatusString();
	if (robot_status != robot_status_past) {
		ROS_INFO("[RSM] robot status %s.", robot_status.c_str());
	}
	robot_status_past = robot_status;

}/*}}}*/

void RobotStatusManager::setStoppedVel(double trans_stopped_vel, double rot_stopped_vel)
{/*{{{*/
	trans_stopped_vel_ = trans_stopped_vel;
	rot_stopped_vel_ = rot_stopped_vel;
}/*}}}*/

void RobotStatusManager::clearStoppedTime()
{/*{{{*/
	stopped_time_ = ros::Time::now();
}/*}}}*/

ros::Duration RobotStatusManager::getTimeWhileStopped()
{/*{{{*/
	if (is_goal_reached_) {
		return ros::Duration(0.0);
	}

	return ros::Time::now() - stopped_time_;
}/*}}}*/

std::string RobotStatusManager::getRobotStatusString()
{/*{{{*/
	std::string ans;
	if (is_goal_reached_) {
		ans = "goal_reached";
	}
	else if (is_robot_moving_) {
		ans = "moving";
	}
	else {
		ans = "stopped";
	}

	return ans;
}/*}}}*/

void RobotStatusManager::movebaseStatusCallback (const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{/*{{{*/
	if (msg->status_list.empty())
		return;

	actionlib_msgs::GoalStatus status = msg->status_list[0];

	if (status.status == actionlib_msgs::GoalStatus::SUCCEEDED) {
		is_goal_reached_ = true;
	}
	else {
		if (is_goal_reached_) {
			stopped_time_ = ros::Time::now();
			is_goal_reached_ = false;
		}
	}
}/*}}}*/

void RobotStatusManager::cmdVelCallback (const geometry_msgs::Twist& msg)
{/*{{{*/
	updateRobotStatus(msg.linear.x, msg.angular.z);
}/*}}}*/

}// namespace ymglp
